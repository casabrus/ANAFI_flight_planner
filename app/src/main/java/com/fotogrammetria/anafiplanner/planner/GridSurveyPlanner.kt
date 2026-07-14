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
            polygon = rotatedPolygon,
            takeoffPoint = takeoffPoint,
        )

        val routePoints = mutableListOf<Point2D>()
        val waypoints = mutableListOf<FlightWaypoint>()
        val captureAction = ImageStartCaptureAction(
            periodSec = metrics.timing.selectedPeriodSec,
            resolution = parameters.cameraProfile.photoMode.jsonResolution,
        )

        var previousSegmentEnd: Point2D? = null
        orderedSegments.forEachIndexed { index, segment ->
            val transferPath = previousSegmentEnd?.let { previousEnd ->
                buildInteriorTransferPath(
                    start = previousEnd,
                    end = segment.start,
                    polygon = rotatedPolygon,
                )
            }.orEmpty()
            transferPath.drop(1).dropLast(1).forEachIndexed { transferIndex, transferPoint ->
                val nextPoint = transferPath[transferIndex + 2]
                val originalPoint = transferPoint.rotate(inverseRotationRad)
                val originalNext = nextPoint.rotate(inverseRotationRad)
                val transferGeo = projection.toGeoPoint(originalPoint)
                routePoints += originalPoint
                waypoints += FlightWaypoint(
                    latitude = transferGeo.lat,
                    longitude = transferGeo.lon,
                    altitudeM = parameters.altitudeM,
                    yawDeg = freeFlightYawDegrees(originalPoint, originalNext),
                    speedMps = metrics.timing.selectedSpeedMps,
                    actions = emptyList(),
                )
            }

            val originalStart = segment.start.rotate(inverseRotationRad)
            val originalEnd = segment.end.rotate(inverseRotationRad)
            val yawDeg = freeFlightYawDegrees(originalStart, originalEnd)
            val startGeo = projection.toGeoPoint(originalStart)
            val endGeo = projection.toGeoPoint(originalEnd)
            routePoints += originalStart
            routePoints += originalEnd
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
            previousSegmentEnd = segment.end
        }

        val estimatedDistanceM = routePoints
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
        polygon: List<Point2D>,
        takeoffPoint: TakeoffPoint?,
    ): List<LineSegment> {
        val candidates = listOf(
            buildBoustrophedon(rows, ascending = true, startLeftToRight = true),
            buildBoustrophedon(rows, ascending = true, startLeftToRight = false),
            buildBoustrophedon(rows, ascending = false, startLeftToRight = true),
            buildBoustrophedon(rows, ascending = false, startLeftToRight = false),
        )

        val takeoffLocal = takeoffPoint
            ?.let { projection.toLocalMeters(GeoPoint(it.lat, it.lon)).rotate(rotationRad) }
        return candidates.minByOrNull { candidate ->
            val takeoffDistance = takeoffLocal?.let { local ->
                candidate.firstOrNull()?.start?.distanceTo(local) ?: Double.POSITIVE_INFINITY
            } ?: 0.0
            takeoffDistance + routeDistance(candidate, polygon)
        } ?: candidates.first()
    }

    private fun routeDistance(
        segments: List<LineSegment>,
        polygon: List<Point2D>,
    ): Double {
        var distance = 0.0
        var previousEnd: Point2D? = null
        segments.forEach { segment ->
            previousEnd?.let { previous ->
                distance += buildInteriorTransferPath(previous, segment.start, polygon)
                    .zipWithNext()
                    .sumOf { (start, end) -> start.distanceTo(end) }
            }
            distance += segment.length()
            previousEnd = segment.end
        }
        return distance
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

    private fun buildInteriorTransferPath(
        start: Point2D,
        end: Point2D,
        polygon: List<Point2D>,
    ): List<Point2D> {
        if (start.distanceTo(end) <= geometryEpsilon) {
            return listOf(start)
        }
        if (segmentStaysInsidePolygon(start, end, polygon)) {
            return listOf(start, end)
        }

        val vertices = polygonVertices(polygon)
        val nodes = buildList {
            add(start)
            add(end)
            vertices.forEach { vertex ->
                if (!vertex.sameLocation(start) && !vertex.sameLocation(end)) {
                    add(vertex)
                }
            }
        }
        val distances = DoubleArray(nodes.size) { Double.POSITIVE_INFINITY }
        val previous = IntArray(nodes.size) { -1 }
        val visited = BooleanArray(nodes.size)
        distances[0] = 0.0

        while (true) {
            val currentIndex = nodes.indices
                .filterNot { visited[it] }
                .minByOrNull { distances[it] }
                ?: break
            if (currentIndex == 1 || distances[currentIndex] == Double.POSITIVE_INFINITY) {
                break
            }

            visited[currentIndex] = true
            nodes.indices
                .filterNot { it == currentIndex || visited[it] }
                .forEach { candidateIndex ->
                    if (!segmentStaysInsidePolygon(nodes[currentIndex], nodes[candidateIndex], polygon)) {
                        return@forEach
                    }
                    val candidateDistance = distances[currentIndex] +
                        nodes[currentIndex].distanceTo(nodes[candidateIndex])
                    if (candidateDistance < distances[candidateIndex]) {
                        distances[candidateIndex] = candidateDistance
                        previous[candidateIndex] = currentIndex
                    }
                }
        }

        if (previous[1] == -1) {
            return listOf(start, end)
        }

        val path = mutableListOf<Point2D>()
        var cursor = 1
        while (cursor != -1) {
            path += nodes[cursor]
            cursor = previous[cursor]
        }
        return path.asReversed()
    }

    private fun segmentStaysInsidePolygon(
        start: Point2D,
        end: Point2D,
        polygon: List<Point2D>,
    ): Boolean {
        val vertices = polygonVertices(polygon)
        if (vertices.size < 3) {
            return false
        }
        if (!pointInsideOrOnPolygon(start, vertices) || !pointInsideOrOnPolygon(end, vertices)) {
            return false
        }

        val checkpoints = mutableListOf(0.0, 1.0)
        vertices.forEachIndexed { index, edgeStart ->
            val edgeEnd = vertices[(index + 1) % vertices.size]
            if (segmentsProperlyIntersect(start, end, edgeStart, edgeEnd)) {
                return false
            }
            segmentIntersectionParameter(start, end, edgeStart, edgeEnd)?.let(checkpoints::add)
            parameterForPointOnSegment(edgeStart, start, end)?.let(checkpoints::add)
            parameterForPointOnSegment(edgeEnd, start, end)?.let(checkpoints::add)
        }

        val sortedCheckpoints = checkpoints
            .map { it.coerceIn(0.0, 1.0) }
            .sorted()
            .fold(mutableListOf<Double>()) { distinct, value ->
                if (distinct.isEmpty() || abs(value - distinct.last()) > parameterEpsilon) {
                    distinct += value
                }
                distinct
            }
        sortedCheckpoints.zipWithNext().forEach { (from, to) ->
            if (to - from <= parameterEpsilon) {
                return@forEach
            }
            val midpoint = interpolate(start, end, (from + to) / 2.0)
            if (!pointInsideOrOnPolygon(midpoint, vertices)) {
                return false
            }
        }
        return true
    }

    private fun polygonVertices(polygon: List<Point2D>): List<Point2D> {
        return if (polygon.size >= 2 && polygon.first().sameLocation(polygon.last())) {
            polygon.dropLast(1)
        } else {
            polygon
        }
    }

    private fun pointInsideOrOnPolygon(point: Point2D, vertices: List<Point2D>): Boolean {
        vertices.forEachIndexed { index, start ->
            val end = vertices[(index + 1) % vertices.size]
            if (pointOnSegment(point, start, end)) {
                return true
            }
        }

        var inside = false
        var previous = vertices.last()
        vertices.forEach { current ->
            val crossesRay = (current.y > point.y) != (previous.y > point.y)
            if (crossesRay) {
                val intersectionX = current.x +
                    ((point.y - current.y) * (previous.x - current.x) / (previous.y - current.y))
                if (point.x < intersectionX) {
                    inside = !inside
                }
            }
            previous = current
        }
        return inside
    }

    private fun segmentsProperlyIntersect(
        firstStart: Point2D,
        firstEnd: Point2D,
        secondStart: Point2D,
        secondEnd: Point2D,
    ): Boolean {
        val firstSecondStart = orientationSign(firstStart, firstEnd, secondStart)
        val firstSecondEnd = orientationSign(firstStart, firstEnd, secondEnd)
        val secondFirstStart = orientationSign(secondStart, secondEnd, firstStart)
        val secondFirstEnd = orientationSign(secondStart, secondEnd, firstEnd)
        return firstSecondStart * firstSecondEnd < 0 && secondFirstStart * secondFirstEnd < 0
    }

    private fun segmentIntersectionParameter(
        start: Point2D,
        end: Point2D,
        edgeStart: Point2D,
        edgeEnd: Point2D,
    ): Double? {
        val segmentX = end.x - start.x
        val segmentY = end.y - start.y
        val edgeX = edgeEnd.x - edgeStart.x
        val edgeY = edgeEnd.y - edgeStart.y
        val denominator = cross(segmentX, segmentY, edgeX, edgeY)
        if (abs(denominator) <= geometryEpsilon) {
            return null
        }

        val offsetX = edgeStart.x - start.x
        val offsetY = edgeStart.y - start.y
        val segmentParameter = cross(offsetX, offsetY, edgeX, edgeY) / denominator
        val edgeParameter = cross(offsetX, offsetY, segmentX, segmentY) / denominator
        return if (
            segmentParameter in -parameterEpsilon..(1.0 + parameterEpsilon) &&
            edgeParameter in -parameterEpsilon..(1.0 + parameterEpsilon)
        ) {
            segmentParameter.coerceIn(0.0, 1.0)
        } else {
            null
        }
    }

    private fun parameterForPointOnSegment(
        point: Point2D,
        start: Point2D,
        end: Point2D,
    ): Double? {
        if (!pointOnSegment(point, start, end)) {
            return null
        }

        val deltaX = end.x - start.x
        val deltaY = end.y - start.y
        return if (abs(deltaX) >= abs(deltaY)) {
            if (abs(deltaX) <= geometryEpsilon) 0.0 else (point.x - start.x) / deltaX
        } else {
            (point.y - start.y) / deltaY
        }.coerceIn(0.0, 1.0)
    }

    private fun pointOnSegment(point: Point2D, start: Point2D, end: Point2D): Boolean {
        val tolerance = geometryEpsilon * start.distanceTo(end).coerceAtLeast(1.0)
        return abs(cross(start, end, point)) <= tolerance &&
            point.x >= minOf(start.x, end.x) - geometryEpsilon &&
            point.x <= maxOf(start.x, end.x) + geometryEpsilon &&
            point.y >= minOf(start.y, end.y) - geometryEpsilon &&
            point.y <= maxOf(start.y, end.y) + geometryEpsilon
    }

    private fun orientationSign(start: Point2D, end: Point2D, point: Point2D): Int {
        val value = cross(start, end, point)
        val tolerance = geometryEpsilon * max(start.distanceTo(end), 1.0)
        return when {
            value > tolerance -> 1
            value < -tolerance -> -1
            else -> 0
        }
    }

    private fun interpolate(start: Point2D, end: Point2D, parameter: Double): Point2D {
        return Point2D(
            x = start.x + ((end.x - start.x) * parameter),
            y = start.y + ((end.y - start.y) * parameter),
        )
    }

    private fun cross(start: Point2D, end: Point2D, point: Point2D): Double {
        return cross(
            x1 = end.x - start.x,
            y1 = end.y - start.y,
            x2 = point.x - start.x,
            y2 = point.y - start.y,
        )
    }

    private fun cross(
        x1: Double,
        y1: Double,
        x2: Double,
        y2: Double,
    ): Double {
        return (x1 * y2) - (y1 * x2)
    }

    private fun Point2D.sameLocation(other: Point2D): Boolean {
        return distanceTo(other) <= geometryEpsilon
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
        private const val geometryEpsilon = 1e-5
        private const val parameterEpsilon = 1e-7
    }
}
