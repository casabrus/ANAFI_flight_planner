package com.fotogrammetria.anafiplanner.planner

import kotlin.math.ceil

data class ElevationSummary(
    val totalSamples: Int,
    val validSamples: Int,
    val minElevationM: Double?,
    val maxElevationM: Double?,
    val averageElevationM: Double?,
)

data class RouteSamplePoint(
    val segmentStartIndex: Int,
    val ratio: Double,
    val point: GeoPoint,
)

object TerrainSampling {
    fun sampleRouteBySpacing(
        waypoints: List<FlightWaypoint>,
        maxSpacingM: Double,
    ): List<GeoPoint> {
        val intermediateSamples = sampleRouteSegmentsBySpacing(waypoints, maxSpacingM)
        if (waypoints.isEmpty() || maxSpacingM <= 0.0) {
            return emptyList()
        }
        if (waypoints.size == 1) {
            return listOf(GeoPoint(waypoints.first().latitude, waypoints.first().longitude))
        }

        val projection = GeoProjection(
            GeoPoint(
                lat = waypoints.first().latitude,
                lon = waypoints.first().longitude,
            ),
        )
        val localPoints = waypoints.map { waypoint ->
            projection.toLocalMeters(GeoPoint(waypoint.latitude, waypoint.longitude))
        }

        return buildList {
            add(GeoPoint(waypoints.first().latitude, waypoints.first().longitude))
            intermediateSamples.forEach { add(it.point) }
            add(GeoPoint(waypoints.last().latitude, waypoints.last().longitude))
        }
    }

    fun sampleRouteSegmentsBySpacing(
        waypoints: List<FlightWaypoint>,
        maxSpacingM: Double,
    ): List<RouteSamplePoint> {
        if (waypoints.isEmpty() || maxSpacingM <= 0.0) {
            return emptyList()
        }
        if (waypoints.size == 1) {
            return emptyList()
        }

        val projection = GeoProjection(
            GeoPoint(
                lat = waypoints.first().latitude,
                lon = waypoints.first().longitude,
            ),
        )
        val localPoints = waypoints.map { waypoint ->
            projection.toLocalMeters(GeoPoint(waypoint.latitude, waypoint.longitude))
        }

        val sampledPoints = mutableListOf<RouteSamplePoint>()
        for (index in 0 until localPoints.lastIndex) {
            val start = localPoints[index]
            val end = localPoints[index + 1]
            val segmentLength = start.distanceTo(end)
            if (segmentLength <= epsilon) {
                continue
            }

            val subdivisions = ceil(segmentLength / maxSpacingM).toInt().coerceAtLeast(1)
            for (stepIndex in 1 until subdivisions) {
                val ratio = stepIndex.toDouble() / subdivisions
                val interpolatedPoint = Point2D(
                    x = start.x + (end.x - start.x) * ratio,
                    y = start.y + (end.y - start.y) * ratio,
                )
                sampledPoints += RouteSamplePoint(
                    segmentStartIndex = index,
                    ratio = ratio,
                    point = projection.toGeoPoint(interpolatedPoint),
                )
            }
        }

        return sampledPoints
    }

    fun summarizeElevations(elevations: List<Double?>): ElevationSummary {
        val validElevations = elevations.filterNotNull()
        return ElevationSummary(
            totalSamples = elevations.size,
            validSamples = validElevations.size,
            minElevationM = validElevations.minOrNull(),
            maxElevationM = validElevations.maxOrNull(),
            averageElevationM = if (validElevations.isEmpty()) {
                null
            } else {
                validElevations.average()
            },
        )
    }

    private const val epsilon = 1e-6
}
