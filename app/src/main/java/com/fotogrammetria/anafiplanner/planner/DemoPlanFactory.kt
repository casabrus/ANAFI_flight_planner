package com.fotogrammetria.anafiplanner.planner

object DemoPlanFactory {
    fun samplePolygon(): List<GeoPoint> {
        val origin = GeoPoint(lat = 45.0703, lon = 7.6869)
        val projection = GeoProjection(origin)
        return listOf(
            Point2D(0.0, 0.0),
            Point2D(85.0, 8.0),
            Point2D(72.0, 68.0),
            Point2D(-10.0, 58.0),
        ).map(projection::toGeoPoint)
    }

    fun defaultGimbalSettings(): GimbalSettings {
        return GimbalSettings(
            tiltDeg = 90.0,
            tiltSpeedDegSec = 30.0,
        )
    }
}
