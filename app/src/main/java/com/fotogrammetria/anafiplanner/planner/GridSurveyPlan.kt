package com.fotogrammetria.anafiplanner.planner

sealed interface WaypointAction

data class TiltAction(
    val angleDeg: Double,
    val speedDegSec: Double,
) : WaypointAction

data class DelayAction(
    val delaySec: Int,
) : WaypointAction

data class ImageStartCaptureAction(
    val periodSec: Int,
    val resolution: Double,
    val nbOfPictures: Int = 0,
) : WaypointAction

data object ImageStopCaptureAction : WaypointAction

enum class WaypointSegmentType {
    LINEAR,
    CIRCLE_ARC,
}

data class FlightWaypoint(
    val latitude: Double,
    val longitude: Double,
    val altitudeM: Double,
    val yawDeg: Double,
    val speedMps: Double,
    val actions: List<WaypointAction>,
    val segmentTypeToNext: WaypointSegmentType = WaypointSegmentType.LINEAR,
)

data class GridSurveyPlan(
    val title: String,
    val polygon: List<GeoPoint>,
    val cameraProfile: CameraProfile,
    val parameters: GridSurveyParameters,
    val metrics: SurveyMetrics,
    val waypoints: List<FlightWaypoint>,
    val estimatedDistanceM: Double,
    val estimatedDurationSec: Double,
    val estimatedPhotoCount: Int,
)
