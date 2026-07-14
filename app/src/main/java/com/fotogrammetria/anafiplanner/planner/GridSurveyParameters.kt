package com.fotogrammetria.anafiplanner.planner

data class GimbalSettings(
    val tiltDeg: Double,
    val tiltSpeedDegSec: Double,
)

enum class SpeedMode(
    val label: String,
) {
    RECOMMENDED("Keep overlap, adjust speed"),
    CUSTOM("Keep speed, validate overlap"),
}

data class GridSurveyParameters(
    val polygon: List<GeoPoint>,
    val altitudeM: Double,
    val frontOverlap: Double,
    val sideOverlap: Double,
    val cameraProfile: CameraProfile,
    val speedMode: SpeedMode,
    val requestedSpeedMps: Double?,
    val gridAngleDeg: Double,
    val gimbalSettings: GimbalSettings,
) {
    init {
        require(polygon.size >= 3) { "A survey polygon needs at least three points." }
        require(altitudeM > 0.0) { "Altitude must be positive." }
        require(frontOverlap in 0.0..0.99) { "Front overlap must be within [0.0, 0.99]." }
        require(sideOverlap in 0.0..0.99) { "Side overlap must be within [0.0, 0.99]." }
        require(requestedSpeedMps == null || requestedSpeedMps > 0.0) { "Requested speed must be positive." }
        require(gimbalSettings.tiltSpeedDegSec > 0.0) { "Tilt speed must be positive." }
    }
}
