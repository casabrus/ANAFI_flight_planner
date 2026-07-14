package com.fotogrammetria.anafiplanner.planner

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.max

class GridSurveyPlanner {
    fun generatePlan(
        parameters: GridSurveyParameters,
        title: String,
        takeoffPoint: TakeoffPoint? = null,
    ): GridSurveyPlan {
        val closedPolygon = closePolygonIfNeeded(parameters.polygon)
        val projection = GeoProjection(closedPolygon.first())
        val rotationRad = Math.toRadians(-parameters.gridAngleDeg)
        val inverseRotationRad = Math.toRadians(parameters.gridAngleDeg)
        val rotatedPolygon = closedPolygon
            .map(projection::toLocalMeters)
            .map { it.rotate(rotationRad) }

        val metrics = PlannerMath.computeSurveyMetrics(parameters)
        val scanlineRows = mutableListOf<ScanlineRow>()
        val boundingBox = BoundingBox.from(rotatedPolygon)

        var y = boundingBox.minY
        while (y <= boundingBox.maxY + epsilon) {
            val rowSegments = clipScanline(rotatedPolygon, y)
            if (rowSegments.isNotEmpty()) {
                scanlineRows += ScanlineRow(
                    y = y,
                    segments = rowSegments,
                )
            }
            y += metrics.lineSpacingM
        }

        require(scanlineRows.isNotEmpty()) { "No survey legs were generated for the selected polygon." }

        val orderedSegments = chooseOrderedSegments(
            rows = scanlineRows,
            projection = projection,
            rotationRad = rotationRad,
            takeoffPoint = takeoffPoint,
        )

        val orderedPoints = mutableListOf<Point2D>()
        val waypoints = mutableListOf<FlightWaypoint>()
        val captureAction = ImageStartCaptureAction(
            periodSec = metrics.timing.selectedPeriodSec,
            resolution = parameters.cameraProfile.photoMode.jsonResolution,
        )

        orderedSegments.forEachIndexed { index, segment ->
            val originalStart = segment.start.rotate(inverseRotationRad)
            val originalEnd = segment.end.rotate(inverseRotationRad)
            val yawDeg = freeFlightYawDegrees(originalStart, originalEnd)
            val startGeo = projection.toGeoPoint(originalStart)
            val endGeo = projection.toGeoPoint(originalEnd)
            orderedPoints += originalStart
            orderedPoints += originalEnd
            val startActions = buildList {
                if (index == 0) {
                    add(
                        TiltAction(
                            angleDeg = parameters.gimbalSettings.tiltDeg,
                            speedDegSec = parameters.gimbalSettings.tiltSpeedDegSec,
                        ),
                    )
                    add(
                        DelayAction(
                            delaySec = 3,
                        ),
                    )
                }
                add(captureAction)
            }
            waypoints += FlightWaypoint(
                latitude = startGeo.lat,
                longitude = startGeo.lon,
                altitudeM = parameters.altitudeM,
                yawDeg = yawDeg,
                speedMps = metrics.timing.selectedSpeedMps,
                actions = startActions,
            )
            waypoints += FlightWaypoint(
                latitude = endGeo.lat,
                longitude = endGeo.lon,
                altitudeM = parameters.altitudeM,
                yawDeg = yawDeg,
                speedMps = metrics.timing.selectedSpeedMps,
                actions = listOf(ImageStopCaptureAction),
            )
        }

        val estimatedDistanceM = orderedPoints
            .zipWithNext()
            .sumOf { (start, end) -> start.distanceTo(end) }

        val estimatedPhotoCount = orderedSegments.sumOf { segment ->
            max(1, ceil(segment.length() / metrics.photoSpacingM).toInt())
        }

        return GridSurveyPlan(
            title = title,
            polygon = parameters.polygon,
            cameraProfile = parameters.cameraProfile,
            parameters = parameters,
            metrics = metrics,
            waypoints = waypoints,
            estimatedDistanceM = estimatedDistanceM,
            estimatedDurationSec = estimatedDistanceM / metrics.timing.selectedSpeedMps,
            estimatedPhotoCount = estimatedPhotoCount,
        )
    }

    private fun closePolygonIfNeeded(polygon: List<GeoPoint>): List<GeoPoint> {
        return if (polygon.first() == polygon.last()) polygon else polygon + polygon.first()
    }

    private fun clipScanline(polygon: List<Point2D>, y: Double): List<LineSegment> {
        val intersections = mutableListOf<Double>()
        for (index in 0 until polygon.lastIndex) {
            val start = polygon[index]
            val end = polygon[index + 1]
            val minY = minOf(start.y, end.y)
            val maxY = maxOf(start.y, end.y)

            if (y < minY || y >= maxY || abs(start.y - end.y) <= epsilon) {
                continue
            }

            val ratio = (y - start.y) / (end.y - start.y)
            intersections += start.x + ratio * (end.x - start.x)
        }

        return intersections
            .sorted()
            .chunked(2)
            .mapNotNull { pair ->
                if (pair.size < 2 || pair[1] - pair[0] <= epsilon) {
                    null
                } else {
                    LineSegment(
                        start = Point2D(pair[0], y),
                        end = Point2D(pair[1], y),
                    )
                }
            }
    }

    private fun compassBearingDegrees(start: Point2D, end: Point2D): Double {
        val angleRad = atan2(end.x - start.x, end.y - start.y)
        return (Math.toDegrees(angleRad) + 360.0) % 360.0
    }

    private fun freeFlightYawDegrees(start: Point2D, end: Point2D): Double {
        val compassBearing = compassBearingDegrees(start, end)
        return (360.0 - compassBearing) % 360.0
    }

    private fun chooseOrderedSegments(
        rows: List<ScanlineRow>,
        projection: GeoProjection,
        rotationRad: Double,
        takeoffPoint: TakeoffPoint?,
    ): List<LineSegment> {
        val candidates = listOf(
            buildBoustrophedon(rows, ascending = true, startLeftToRight = true),
            buildBoustrophedon(rows, ascending = true, startLeftToRight = false),
            buildBoustrophedon(rows, ascending = false, startLeftToRight = true),
            buildBoustrophedon(rows, ascending = false, startLeftToRight = false),
        )
        if (takeoffPoint == null) {
            return candidates.first()
        }

        val takeoffLocal = projection.toLocalMeters(GeoPoint(takeoffPoint.lat, takeoffPoint.lon)).rotate(rotationRad)
        return candidates.minByOrNull { candidate ->
            candidate.firstOrNull()?.start?.distanceTo(takeoffLocal) ?: Double.POSITIVE_INFINITY
        } ?: candidates.first()
    }

    private fun buildBoustrophedon(
        rows: List<ScanlineRow>,
        ascending: Boolean,
        startLeftToRight: Boolean,
    ): List<LineSegment> {
        val sortedRows = if (ascending) {
            rows.sortedBy { it.y }
        } else {
            rows.sortedByDescending { it.y }
        }

        return buildList {
            sortedRows.forEachIndexed { index, row ->
            val leftToRight = if (index % 2 == 0) startLeftToRight else !startLeftToRight
                val rowSegments = if (leftToRight) {
                    row.segments
                        .sortedBy { it.minX() }
                        .map { it.leftToRight() }
                } else {
                    row.segments
                        .sortedByDescending { it.maxX() }
                        .map { it.rightToLeft() }
                }
                addAll(rowSegments)
            }
        }
    }

    private data class LineSegment(
        val start: Point2D,
        val end: Point2D,
    ) {
        fun length(): Double = start.distanceTo(end)

        fun centerY(): Double = (start.y + end.y) / 2.0

        fun minX(): Double = minOf(start.x, end.x)

        fun maxX(): Double = maxOf(start.x, end.x)

        fun leftToRight(): LineSegment = if (start.x <= end.x) this else LineSegment(end, start)

        fun rightToLeft(): LineSegment = if (start.x >= end.x) this else LineSegment(end, start)
    }

    private data class ScanlineRow(
        val y: Double,
        val segments: List<LineSegment>,
    )

    private data class BoundingBox(
        val minY: Double,
        val maxY: Double,
    ) {
        companion object {
            fun from(points: List<Point2D>): BoundingBox {
                return BoundingBox(
                    minY = points.minOf { it.y },
                    maxY = points.maxOf { it.y },
                )
            }
        }
    }

    private companion object {
        private const val epsilon = 1e-6
    }
}
