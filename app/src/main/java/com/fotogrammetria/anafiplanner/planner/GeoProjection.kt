package com.fotogrammetria.anafiplanner.planner

import kotlin.math.cos

class GeoProjection(
    private val origin: GeoPoint,
) {
    fun toLocalMeters(point: GeoPoint): Point2D {
        val latRad = Math.toRadians(point.lat)
        val originLatRad = Math.toRadians(origin.lat)
        val originLonRad = Math.toRadians(origin.lon)
        val lonRad = Math.toRadians(point.lon)
        val x = (lonRad - originLonRad) * earthRadiusM * cos((latRad + originLatRad) / 2.0)
        val y = (latRad - originLatRad) * earthRadiusM
        return Point2D(x = x, y = y)
    }

    fun toGeoPoint(point: Point2D): GeoPoint {
        val originLatRad = Math.toRadians(origin.lat)
        val latRad = originLatRad + (point.y / earthRadiusM)
        val lonRad = Math.toRadians(origin.lon) + (point.x / (earthRadiusM * cos((latRad + originLatRad) / 2.0)))
        return GeoPoint(
            lat = Math.toDegrees(latRad),
            lon = Math.toDegrees(lonRad),
        )
    }

    private companion object {
        private const val earthRadiusM = 6_378_137.0
    }
}
