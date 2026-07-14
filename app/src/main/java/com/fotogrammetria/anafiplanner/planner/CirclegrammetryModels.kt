package com.fotogrammetria.anafiplanner.planner

data class FocusTarget(
    val mode: FocusTargetMode,
    val heightAboveTerrainM: Double,
)

enum class FocusTargetMode {
    GROUND,
    CANOPY_CENTER,
    OBJECT_CENTER,
    CUSTOM,
}

data class CircleGeometry(
    val flightHeightM: Double,
    val focusHeightM: Double,
    val radiusM: Double,
    val offNadirAngleDeg: Double,
    val mode: CircleGeometryMode,
)

enum class CircleGeometryMode {
    LOCK_CAMERA_TO_CENTER,
    LOCK_RADIUS,
    PAPER_PRESET,
    OPTIMIZE_FOR_AREA,
}

enum class CircleRotationStrategy {
    CLOCKWISE,
    COUNTERCLOCKWISE,
    ALTERNATING_ROWS,
}

enum class CircleShotMode {
    AUTO,
    FIXED_PERIOD,
    FIXED_SHOTS_PER_CIRCLE,
}

enum class CircleOptimizationMode {
    USER_FIXED_OVERLAP,
    FASTEST,
    BALANCED_THRESHOLD,
    QUALITY_50_PERCENT,
    CUSTOM_RANGE_SCAN,
}

enum class BeltSides {
    TWO_LONG_SIDES,
    FOUR_SIDES,
}

data class OuterBeltPassParameters(
    val enabled: Boolean,
    val distanceOutsidePolygonM: Double,
    val altitudeM: Double,
    val tiltDeg: Double,
    val speedMps: Double,
    val photoMode: FreeFlightPhotoMode,
    val periodSec: Int,
    val sides: BeltSides,
)

data class CirclegrammetryParameters(
    val polygon: List<GeoPoint>,
    val altitudeM: Double,
    val radiusM: Double,
    val radiusLocked: Boolean = false,
    val requestedOverlap: Double,
    val optimizedOverlap: Double?,
    val maxExtensionOutsideAreaM: Double?,
    val speedMode: SpeedMode,
    val requestedSpeedMps: Double?,
    val cameraTiltDeg: Double,
    val cameraAngleLocked: Boolean = false,
    val cameraTiltSpeedDegSec: Double,
    val photoMode: FreeFlightPhotoMode,
    val cameraProfile: CameraProfile,
    val pointsPerCircle: Int,
    val pointsLocked: Boolean = false,
    val targetSegmentLengthM: Double? = null,
    val rotationStrategy: CircleRotationStrategy,
    val shotMode: CircleShotMode,
    val optimizationMode: CircleOptimizationMode,
    val focusTarget: FocusTarget,
    val outerBeltPass: OuterBeltPassParameters? = null,
) {
    init {
        require(polygon.size >= 3) { "A survey polygon needs at least three points." }
        require(altitudeM > 0.0) { "Altitude must be positive." }
        require(radiusM > 0.0) { "Circle radius must be positive." }
        require(requestedOverlap in 0.0..0.99) { "Circle overlap must be within [0.0, 0.99]." }
        require(optimizedOverlap == null || optimizedOverlap in 0.0..0.99) {
            "Optimized overlap must be within [0.0, 0.99]."
        }
        require(requestedSpeedMps == null || requestedSpeedMps > 0.0) { "Requested speed must be positive." }
        require(cameraTiltSpeedDegSec > 0.0) { "Tilt speed must be positive." }
        require(pointsPerCircle >= 12) { "At least 12 points per circle are required." }
        require(pointsPerCircle <= 96) { "Points per circle exceed supported limits." }
        require(targetSegmentLengthM == null || targetSegmentLengthM > 0.0) {
            "Target segment length must be positive."
        }
        require(focusTarget.heightAboveTerrainM >= 0.0) { "Focus target height cannot be negative." }
    }
}

data class CircleGridCount(
    val circlesX: Int,
    val circlesY: Int,
    val totalCircles: Int,
    val spacingM: Double,
)

data class CircleFootprint(
    val coveredLengthM: Double,
    val coveredWidthM: Double,
    val extensionXEachSideM: Double,
    val extensionYEachSideM: Double,
)

data class CirclegrammetryOptimizationResult(
    val overlap: Double,
    val radiusM: Double,
    val spacingM: Double,
    val circlesX: Int,
    val circlesY: Int,
    val totalCircles: Int,
    val coveredLengthM: Double,
    val coveredWidthM: Double,
    val extensionXEachSideM: Double,
    val extensionYEachSideM: Double,
    val estimatedWaypoints: Int,
    val estimatedPhotos: Int,
    val estimatedFlightTimeSec: Double,
    val isBeforeThreshold: Boolean,
    val isRecommended: Boolean,
    val warning: String?,
)

data class CircleCenter(
    val row: Int,
    val col: Int,
    val xM: Double,
    val yM: Double,
)

data class CircleMissionLayout(
    val centers: List<CircleCenter>,
    val circlesX: Int,
    val circlesY: Int,
    val totalCircles: Int,
    val spacingM: Double,
    val footprint: CircleFootprint,
    val optimization: CirclegrammetryOptimizationResult?,
)

data class CircleMissionEstimate(
    val waypoints: Int,
    val photos: Int,
    val flightTimeSec: Double,
    val shotsPerCircle: Int,
    val selectedPeriodSec: Int,
    val selectedSpeedMps: Double,
    val maxSupportedSpeedMps: Double,
    val warning: String?,
)

data class CirclegrammetryPlan(
    val title: String,
    val parameters: CirclegrammetryParameters,
    val geometry: CircleGeometry,
    val layout: CircleMissionLayout,
    val estimate: CircleMissionEstimate,
    val waypoints: List<FlightWaypoint>,
    val estimatedDistanceM: Double,
)
