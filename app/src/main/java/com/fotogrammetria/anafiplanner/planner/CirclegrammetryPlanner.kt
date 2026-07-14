package com.fotogrammetria.anafiplanner.planner

import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.sin

class CirclegrammetryPlanner {
    fun plan(
        parameters: CirclegrammetryParameters,
        title: String,
        takeoffPoint: TakeoffPoint? = null,
    ): CirclegrammetryPlan {
        val projectedPolygon = projectPolygon(parameters.polygon)
        val bounds = LocalBounds.from(projectedPolygon)
        val scanRange = overlapScanRange(parameters)
        val baseGeometry = CirclegrammetryMath.resolveGeometry(parameters)
        val radiusAdjustment = adjustRadiusForExtension(
            baseGeometry = baseGeometry,
            bounds = bounds,
            parameters = parameters,
            scanRange = scanRange,
        )
        val geometry = baseGeometry.copy(
            radiusM = radiusAdjustment.radiusM,
            offNadirAngleDeg = CirclegrammetryMath.offNadirAngleFromRadius(
                flightHeightM = baseGeometry.flightHeightM,
                focusHeightM = baseGeometry.focusHeightM,
                radiusM = radiusAdjustment.radiusM,
            ),
        )
        val effectiveParameters = parameters.copy(
            radiusM = geometry.radiusM,
            cameraTiltDeg = if (parameters.cameraAngleLocked) {
                parameters.cameraTiltDeg
            } else {
                (90.0 - geometry.offNadirAngleDeg).coerceIn(20.0, 85.0)
            },
            pointsPerCircle = if (parameters.pointsLocked) {
                parameters.pointsPerCircle
            } else {
                autoCirclePointsPerCircle(
                    radiusM = geometry.radiusM,
                    targetSegmentLengthM = parameters.targetSegmentLengthM ?: defaultSegmentLengthM,
                )
            },
        )
        val optimizationResults = buildOptimizationResults(
            bounds = bounds,
            parameters = effectiveParameters,
            radiusM = geometry.radiusM,
            scanRange = scanRange,
        )
        val selectedOptimization = selectOptimizationResult(effectiveParameters, optimizationResults)
        val origin = Point2D(
            x = bounds.minX - selectedOptimization.extensionXEachSideM + geometry.radiusM,
            y = bounds.minY - selectedOptimization.extensionYEachSideM + geometry.radiusM,
        )
        val centers = chooseOrderedCenters(
            origin = origin,
            circlesX = selectedOptimization.circlesX,
            circlesY = selectedOptimization.circlesY,
            spacingM = selectedOptimization.spacingM,
            polygon = projectedPolygon,
            projection = GeoProjection(parameters.polygon.first()),
            takeoffPoint = takeoffPoint,
        )
        require(centers.isNotEmpty()) { "No circle centers were generated inside the selected polygon." }
        val actualCirclesY = centers.map { it.row }.distinct().size
        val actualCirclesX = centers.groupBy { it.row }.maxOfOrNull { it.value.size } ?: 0
        val actualTotalCircles = centers.size
        val layout = CircleMissionLayout(
            centers = centers,
            circlesX = actualCirclesX,
            circlesY = actualCirclesY,
            totalCircles = actualTotalCircles,
            spacingM = selectedOptimization.spacingM,
            footprint = CircleFootprint(
                coveredLengthM = selectedOptimization.coveredLengthM,
                coveredWidthM = selectedOptimization.coveredWidthM,
                extensionXEachSideM = selectedOptimization.extensionXEachSideM,
                extensionYEachSideM = selectedOptimization.extensionYEachSideM,
            ),
            optimization = selectedOptimization,
        )
        val selectedSpeedMps = computeSelectedSpeed(geometry.radiusM, effectiveParameters)
        val estimate = CircleMissionEstimate(
            waypoints = selectedOptimization.estimatedWaypoints,
            photos = selectedOptimization.estimatedPhotos,
            flightTimeSec = selectedOptimization.estimatedFlightTimeSec,
            shotsPerCircle = effectiveParameters.pointsPerCircle,
            selectedPeriodSec = computeSelectedPeriodSec(geometry.radiusM, effectiveParameters),
            selectedSpeedMps = selectedSpeedMps,
            maxSupportedSpeedMps = computeMaxSupportedSpeed(geometry.radiusM, effectiveParameters),
            warning = buildWarningMessage(
                geometry = geometry,
                parameters = effectiveParameters,
                optimization = selectedOptimization,
                adjustmentWarning = radiusAdjustment.warning,
            ),
        )
        val projection = GeoProjection(parameters.polygon.first())
        val waypoints = buildWaypoints(
            centers = centers,
            projection = projection,
            takeoffPoint = takeoffPoint,
            altitudeM = effectiveParameters.altitudeM,
            radiusM = geometry.radiusM,
            pointsPerCircle = effectiveParameters.pointsPerCircle,
            speedMps = selectedSpeedMps,
            tiltDeg = effectiveParameters.cameraTiltDeg,
            tiltSpeedDegSec = effectiveParameters.cameraTiltSpeedDegSec,
            photoMode = effectiveParameters.photoMode,
            periodSec = estimate.selectedPeriodSec,
            rotationStrategy = effectiveParameters.rotationStrategy,
        )
        val estimatedDistanceM = estimateRouteDistance(waypoints, projection)
        val estimatedFlightTimeSec = if (selectedSpeedMps > 0.0) {
            estimatedDistanceM / selectedSpeedMps
        } else {
            0.0
        }
        val estimatedPhotoCount = actualTotalCircles * effectiveParameters.pointsPerCircle
        val actualEstimate = estimate.copy(
            waypoints = waypoints.size,
            photos = estimatedPhotoCount,
            flightTimeSec = estimatedFlightTimeSec,
        )

        return CirclegrammetryPlan(
            title = title,
            parameters = effectiveParameters,
            geometry = geometry,
            layout = layout,
            estimate = actualEstimate,
            waypoints = waypoints,
            estimatedDistanceM = estimatedDistanceM,
        )
    }

    private fun buildOptimizationResults(
        bounds: LocalBounds,
        parameters: CirclegrammetryParameters,
        radiusM: Double,
        scanRange: Pair<Double, Double>,
    ): List<CirclegrammetryOptimizationResult> {
        return CirclegrammetryMath.scanCircleOverlaps(
            lengthM = bounds.lengthM,
            widthM = bounds.widthM,
            radiusM = radiusM,
            pointsPerCircle = parameters.pointsPerCircle,
            speedMode = parameters.speedMode,
            requestedSpeedMps = parameters.requestedSpeedMps,
            photoMode = parameters.photoMode,
            minOverlap = scanRange.first,
            maxOverlap = scanRange.second,
            step = if (scanRange.first == scanRange.second) 1.0 else 0.05,
            maxExtensionM = parameters.maxExtensionOutsideAreaM,
        )
    }

    private fun adjustRadiusForExtension(
        baseGeometry: CircleGeometry,
        bounds: LocalBounds,
        parameters: CirclegrammetryParameters,
        scanRange: Pair<Double, Double>,
    ): RadiusAdjustment {
        val maxExtensionM = parameters.maxExtensionOutsideAreaM ?: return RadiusAdjustment(
            radiusM = baseGeometry.radiusM,
            warning = null,
        )
        val baseResults = buildOptimizationResults(
            bounds = bounds,
            parameters = parameters,
            radiusM = baseGeometry.radiusM,
            scanRange = scanRange,
        )
        if (baseResults.any { it.warning == null }) {
            return RadiusAdjustment(radiusM = baseGeometry.radiusM, warning = null)
        }
        if (parameters.radiusLocked) {
            return RadiusAdjustment(
                radiusM = baseGeometry.radiusM,
                warning = "Locked radius exceeds the configured outside extension limit.",
            )
        }

        var candidateRadiusM = baseGeometry.radiusM
        repeat(18) {
            candidateRadiusM *= 0.92
            if (candidateRadiusM < minAutoRadiusM) {
                return@repeat
            }
            val candidateResults = buildOptimizationResults(
                bounds = bounds,
                parameters = parameters,
                radiusM = candidateRadiusM,
                scanRange = scanRange,
            )
            if (candidateResults.any { it.warning == null }) {
                return RadiusAdjustment(
                    radiusM = candidateRadiusM,
                    warning = "Radius auto-reduced from ${formatMeters(baseGeometry.radiusM)} to ${formatMeters(candidateRadiusM)} to respect outside extension.",
                )
            }
        }

        return RadiusAdjustment(
            radiusM = candidateRadiusM.coerceAtLeast(minAutoRadiusM),
            warning = "Outside extension limit could not be fully satisfied; review radius or overlap.",
        )
    }

    private fun projectPolygon(polygon: List<GeoPoint>): List<Point2D> {
        val projection = GeoProjection(polygon.first())
        return polygon.map(projection::toLocalMeters)
    }

    private fun selectOptimizationResult(
        parameters: CirclegrammetryParameters,
        results: List<CirclegrammetryOptimizationResult>,
    ): CirclegrammetryOptimizationResult {
        val feasible = results.filter { it.warning == null }
        val selected = when {
            parameters.optimizedOverlap != null -> {
                results.minByOrNull { kotlin.math.abs(it.overlap - parameters.optimizedOverlap) }
            }

            parameters.optimizationMode == CircleOptimizationMode.USER_FIXED_OVERLAP -> {
                results.minByOrNull { kotlin.math.abs(it.overlap - parameters.requestedOverlap) }
            }

            parameters.optimizationMode == CircleOptimizationMode.QUALITY_50_PERCENT -> {
                feasible.minByOrNull { kotlin.math.abs(it.overlap - 0.50) }
            }

            parameters.optimizationMode == CircleOptimizationMode.BALANCED_THRESHOLD -> {
                feasible.filter { it.isRecommended }
                    .minByOrNull { kotlin.math.abs(it.overlap - 0.50) }
            }

            parameters.optimizationMode == CircleOptimizationMode.FASTEST -> {
                feasible.minWithOrNull(
                    compareBy<CirclegrammetryOptimizationResult> { it.totalCircles }
                        .thenBy { it.estimatedFlightTimeSec }
                        .thenBy { it.overlap },
                )
            }

            else -> {
                feasible.filter { it.isRecommended }.maxByOrNull { it.overlap }
            }
        }

        return selected ?: results.first()
    }

    private fun chooseOrderedCenters(
        origin: Point2D,
        circlesX: Int,
        circlesY: Int,
        spacingM: Double,
        polygon: List<Point2D>,
        projection: GeoProjection,
        takeoffPoint: TakeoffPoint?,
    ): List<CircleCenter> {
        val xCoords = List(circlesX) { origin.x + it * spacingM }
        val yCoords = List(circlesY) { origin.y + it * spacingM }
        val candidates = listOf(
            generateOrderedCentersVariant(xCoords, yCoords, polygon, rowAscending = true, startLeftToRight = true),
            generateOrderedCentersVariant(xCoords, yCoords, polygon, rowAscending = true, startLeftToRight = false),
            generateOrderedCentersVariant(xCoords, yCoords, polygon, rowAscending = false, startLeftToRight = true),
            generateOrderedCentersVariant(xCoords, yCoords, polygon, rowAscending = false, startLeftToRight = false),
        )
        if (takeoffPoint == null) {
            return candidates.first()
        }

        val takeoffLocal = projection.toLocalMeters(GeoPoint(takeoffPoint.lat, takeoffPoint.lon))
        return candidates.minByOrNull { candidate ->
            candidate.firstOrNull()?.let { center ->
                Point2D(center.xM, center.yM).distanceTo(takeoffLocal)
            } ?: Double.POSITIVE_INFINITY
        } ?: candidates.first()
    }

    private fun generateOrderedCentersVariant(
        xCoords: List<Double>,
        yCoords: List<Double>,
        polygon: List<Point2D>,
        rowAscending: Boolean,
        startLeftToRight: Boolean,
    ): List<CircleCenter> {
        val rowIndices = if (rowAscending) {
            yCoords.indices.toList()
        } else {
            yCoords.indices.reversed()
        }
        return buildList {
            var routeRowIndex = 0
            rowIndices.forEach { sourceRowIndex ->
                val rowCenters = xCoords.indices.mapNotNull { sourceColIndex ->
                    val center = Point2D(
                        x = xCoords[sourceColIndex],
                        y = yCoords[sourceRowIndex],
                    )
                    if (containsPointInclusive(polygon, center)) {
                        CircleCenter(
                            row = routeRowIndex,
                            col = sourceColIndex,
                            xM = center.x,
                            yM = center.y,
                        )
                    } else {
                        null
                    }
                }
                if (rowCenters.isEmpty()) {
                    return@forEach
                }

                val leftToRight = if (routeRowIndex % 2 == 0) startLeftToRight else !startLeftToRight
                addAll(
                    if (leftToRight) {
                        rowCenters.sortedBy { it.col }
                    } else {
                        rowCenters.sortedByDescending { it.col }
                    },
                )
                routeRowIndex += 1
            }
        }
    }

    private fun computeSelectedPeriodSec(
        radiusM: Double,
        parameters: CirclegrammetryParameters,
    ): Int {
        val circumferenceM = 2.0 * PI * radiusM
        val photoSpacingM = circumferenceM / parameters.pointsPerCircle
        val rawPeriod = ceil(photoSpacingM / computeSelectedSpeed(radiusM, parameters)).toInt()
        return rawPeriod.coerceAtLeast(parameters.photoMode.minPeriodSec)
    }

    private fun computeMaxSupportedSpeed(
        radiusM: Double,
        parameters: CirclegrammetryParameters,
    ): Double {
        val circumferenceM = 2.0 * PI * radiusM
        val photoSpacingM = circumferenceM / parameters.pointsPerCircle
        return photoSpacingM / parameters.photoMode.minPeriodSec
    }

    private fun computeSelectedSpeed(
        radiusM: Double,
        parameters: CirclegrammetryParameters,
    ): Double {
        val maxSupportedSpeedMps = computeMaxSupportedSpeed(radiusM, parameters)
        return when (parameters.speedMode) {
            SpeedMode.RECOMMENDED -> maxSupportedSpeedMps
            SpeedMode.CUSTOM -> minOf(parameters.requestedSpeedMps ?: maxSupportedSpeedMps, maxSupportedSpeedMps)
        }
    }

    private fun buildWaypoints(
        centers: List<CircleCenter>,
        projection: GeoProjection,
        takeoffPoint: TakeoffPoint?,
        altitudeM: Double,
        radiusM: Double,
        pointsPerCircle: Int,
        speedMps: Double,
        tiltDeg: Double,
        tiltSpeedDegSec: Double,
        photoMode: FreeFlightPhotoMode,
        periodSec: Int,
        rotationStrategy: CircleRotationStrategy,
    ): List<FlightWaypoint> {
        val waypoints = mutableListOf<FlightWaypoint>()
        var previousPoint: Point2D? = takeoffPoint?.let { projection.toLocalMeters(GeoPoint(it.lat, it.lon)) }
        val captureAction = ImageStartCaptureAction(
            periodSec = periodSec,
            resolution = photoMode.jsonResolution,
        )

        centers.forEachIndexed { centerIndex, center ->
            val centerPoint = Point2D(center.xM, center.yM)
            val clockwise = CirclegrammetryMath.isClockwiseForRow(center.row, rotationStrategy)
            val direction = if (clockwise) -1.0 else 1.0
            val angleStep = 2.0 * PI / pointsPerCircle
            val localPoints = (0 until pointsPerCircle).map { index ->
                val angle = direction * index * angleStep
                Point2D(
                    x = centerPoint.x + radiusM * cos(angle),
                    y = centerPoint.y + radiusM * sin(angle),
                )
            }
            val startIndex = selectNearestPointIndex(localPoints, previousPoint)
            val orderedPoints = rotatePoints(localPoints, startIndex)
            val closedLoop = orderedPoints + orderedPoints.first()
            val circleWaypoints = closedLoop.mapIndexed { pointIndex, point ->
                val geo = projection.toGeoPoint(point)
                FlightWaypoint(
                    latitude = geo.lat,
                    longitude = geo.lon,
                    altitudeM = altitudeM,
                    yawDeg = freeFlightYawDegrees(point, centerPoint),
                    speedMps = speedMps,
                    actions = when {
                        pointIndex == 0 && centerIndex == 0 -> listOf(
                            TiltAction(
                                angleDeg = tiltDeg,
                                speedDegSec = tiltSpeedDegSec,
                            ),
                            DelayAction(
                                delaySec = 3,
                            ),
                            captureAction,
                        )

                        pointIndex == 0 -> listOf(captureAction)
                        pointIndex == closedLoop.lastIndex -> listOf(ImageStopCaptureAction)
                        else -> emptyList()
                    },
                    segmentTypeToNext = if (pointIndex == closedLoop.lastIndex && centerIndex < centers.lastIndex) {
                        WaypointSegmentType.LINEAR
                    } else {
                        WaypointSegmentType.CIRCLE_ARC
                    },
                )
            }
            waypoints += circleWaypoints
            previousPoint = closedLoop.lastOrNull()
        }

        return waypoints
    }

    private fun selectNearestPointIndex(
        points: List<Point2D>,
        referencePoint: Point2D?,
    ): Int {
        if (referencePoint == null || points.isEmpty()) {
            return 0
        }
        return points.indices.minByOrNull { index ->
            points[index].distanceTo(referencePoint)
        } ?: 0
    }

    private fun rotatePoints(
        points: List<Point2D>,
        startIndex: Int,
    ): List<Point2D> {
        if (points.isEmpty() || startIndex == 0) {
            return points
        }
        return buildList(points.size) {
            addAll(points.drop(startIndex))
            addAll(points.take(startIndex))
        }
    }

    private fun buildWarningMessage(
        geometry: CircleGeometry,
        parameters: CirclegrammetryParameters,
        optimization: CirclegrammetryOptimizationResult,
        adjustmentWarning: String?,
    ): String? {
        val warnings = mutableListOf<String>()
        adjustmentWarning?.let(warnings::add)
        optimization.warning?.let(warnings::add)
        if (geometry.offNadirAngleDeg < 20.0) {
            warnings += "Off-nadir angle is shallow; Circlegrammetry may not provide enough oblique geometry."
        }
        if (geometry.offNadirAngleDeg > 55.0) {
            warnings += "Off-nadir angle is very lateral; footprint and occlusions may increase."
        }
        if (
            parameters.speedMode == SpeedMode.CUSTOM &&
            parameters.requestedSpeedMps != null &&
            parameters.requestedSpeedMps > computeMaxSupportedSpeed(geometry.radiusM, parameters)
        ) {
            warnings += "Requested speed was clamped to the maximum speed supported by the selected photo mode."
        }
        if (parameters.photoMode == FreeFlightPhotoMode.DNG) {
            warnings += "DNG requires a 6s minimum capture period; expect a slow mission."
        }
        if (optimization.overlap < 0.50) {
            warnings += "Pure Circlegrammetry below 50% overlap can underrepresent lower canopy."
        }
        return warnings.distinct().takeIf { it.isNotEmpty() }?.joinToString(separator = "\n")
    }

    private fun autoCirclePointsPerCircle(
        radiusM: Double,
        targetSegmentLengthM: Double,
    ): Int {
        val circumferenceM = 2.0 * PI * radiusM
        val rawPoints = kotlin.math.ceil(circumferenceM / targetSegmentLengthM).toInt()
        val snappedPoints = ((rawPoints + 3) / 4) * 4
        return snappedPoints.coerceIn(12, 96)
    }

    private fun formatMeters(value: Double): String {
        return String.format(java.util.Locale.US, "%.1f m", value)
    }

    private fun overlapScanRange(parameters: CirclegrammetryParameters): Pair<Double, Double> {
        val fixedOverlap = parameters.optimizedOverlap ?: when (parameters.optimizationMode) {
            CircleOptimizationMode.USER_FIXED_OVERLAP -> parameters.requestedOverlap
            CircleOptimizationMode.QUALITY_50_PERCENT -> 0.50
            else -> null
        }
        return if (fixedOverlap != null) {
            fixedOverlap to fixedOverlap
        } else {
            0.25 to 0.60
        }
    }

    private fun compassBearingDegrees(from: Point2D, to: Point2D): Double {
        return (Math.toDegrees(kotlin.math.atan2(to.x - from.x, to.y - from.y)) + 360.0) % 360.0
    }

    private fun freeFlightYawDegrees(from: Point2D, to: Point2D): Double {
        val compassBearing = compassBearingDegrees(from, to)
        return (360.0 - compassBearing) % 360.0
    }

    private fun containsPointInclusive(polygon: List<Point2D>, point: Point2D): Boolean {
        if (polygon.isEmpty()) {
            return false
        }

        var inside = false
        for (index in polygon.indices) {
            val start = polygon[index]
            val end = polygon[(index + 1) % polygon.size]
            if (pointOnSegment(start, end, point)) {
                return true
            }

            val intersects = ((start.y > point.y) != (end.y > point.y)) &&
                (point.x < (end.x - start.x) * (point.y - start.y) / ((end.y - start.y).takeIf { kotlin.math.abs(it) > epsilon } ?: 1.0) + start.x)
            if (intersects) {
                inside = !inside
            }
        }
        return inside
    }

    private fun pointOnSegment(start: Point2D, end: Point2D, point: Point2D): Boolean {
        val cross = (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x)
        if (kotlin.math.abs(cross) > epsilon) {
            return false
        }
        return point.x >= minOf(start.x, end.x) - epsilon &&
            point.x <= maxOf(start.x, end.x) + epsilon &&
            point.y >= minOf(start.y, end.y) - epsilon &&
            point.y <= maxOf(start.y, end.y) + epsilon
    }

    private fun estimateRouteDistance(
        waypoints: List<FlightWaypoint>,
        projection: GeoProjection,
    ): Double {
        return waypoints
            .zipWithNext()
            .sumOf { (start, end) ->
                projection.toLocalMeters(GeoPoint(start.latitude, start.longitude))
                    .distanceTo(projection.toLocalMeters(GeoPoint(end.latitude, end.longitude)))
            }
    }

    private data class LocalBounds(
        val minX: Double,
        val maxX: Double,
        val minY: Double,
        val maxY: Double,
    ) {
        val lengthM: Double get() = maxX - minX
        val widthM: Double get() = maxY - minY

        companion object {
            fun from(points: List<Point2D>): LocalBounds {
                return LocalBounds(
                    minX = points.minOf { it.x },
                    maxX = points.maxOf { it.x },
                    minY = points.minOf { it.y },
                    maxY = points.maxOf { it.y },
                )
            }
        }
    }

    private data class RadiusAdjustment(
        val radiusM: Double,
        val warning: String?,
    )

    private companion object {
        const val minAutoRadiusM = 6.0
        const val defaultSegmentLengthM = 12.0
        const val epsilon = 1e-6
    }
}
