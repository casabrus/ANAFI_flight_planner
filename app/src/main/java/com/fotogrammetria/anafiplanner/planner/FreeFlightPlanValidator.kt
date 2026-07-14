package com.fotogrammetria.anafiplanner.planner

import java.util.Locale
import kotlin.math.abs

data class FreeFlightValidationResult(
    val errors: List<String>,
    val warnings: List<String>,
) {
    val isValid: Boolean
        get() = errors.isEmpty()
}

data class FreeFlightValidationConfig(
    val minAltitudeM: Double = 0.0,
    val maxAltitudeWarningM: Double? = null,
)

class FreeFlightPlanValidator(
    private val config: FreeFlightValidationConfig = FreeFlightValidationConfig(),
) {
    fun validate(waypoints: List<FlightWaypoint>): FreeFlightValidationResult {
        val errors = mutableListOf<String>()
        val warnings = mutableListOf<String>()

        if (waypoints.size < 2) {
            errors += "FreeFlight export requires at least 2 waypoints."
        }

        validateWaypoints(waypoints, errors, warnings)
        validateCaptureActions(waypoints, errors, warnings)

        return FreeFlightValidationResult(
            errors = errors.distinct(),
            warnings = warnings.distinct(),
        )
    }

    private fun validateWaypoints(
        waypoints: List<FlightWaypoint>,
        errors: MutableList<String>,
        warnings: MutableList<String>,
    ) {
        waypoints.forEachIndexed { index, waypoint ->
            val label = "Waypoint ${index + 1}"
            if (!waypoint.latitude.isFinite() || waypoint.latitude !in -90.0..90.0) {
                errors += "$label has an invalid latitude."
            }
            if (!waypoint.longitude.isFinite() || waypoint.longitude !in -180.0..180.0) {
                errors += "$label has an invalid longitude."
            }
            if (!waypoint.altitudeM.isFinite() || waypoint.altitudeM <= config.minAltitudeM) {
                errors += "$label altitude must be greater than ${formatMeters(config.minAltitudeM)}."
            }
            if (!waypoint.yawDeg.isFinite()) {
                errors += "$label has an invalid yaw."
            }
            if (!waypoint.speedMps.isFinite() || waypoint.speedMps <= 0.0) {
                errors += "$label speed must be greater than 0 m/s."
            }
            config.maxAltitudeWarningM?.let { maxAltitude ->
                if (waypoint.altitudeM > maxAltitude) {
                    warnings += "$label altitude exceeds ${formatMeters(maxAltitude)}; review legal and operational limits."
                }
            }
            validateActions(label, waypoint.actions, errors, warnings)
        }
    }

    private fun validateActions(
        waypointLabel: String,
        actions: List<WaypointAction>,
        errors: MutableList<String>,
        warnings: MutableList<String>,
    ) {
        actions.forEach { action ->
            when (action) {
                is TiltAction -> {
                    if (!action.angleDeg.isFinite() || abs(action.angleDeg) > MAX_TILT_ABS_DEG) {
                        errors += "$waypointLabel tilt angle must be within +/-${MAX_TILT_ABS_DEG.toInt()} degrees."
                    }
                    if (!action.speedDegSec.isFinite() || action.speedDegSec !in MIN_TILT_SPEED_DEG_SEC..MAX_TILT_SPEED_DEG_SEC) {
                        errors += "$waypointLabel tilt speed must be within ${MIN_TILT_SPEED_DEG_SEC.toInt()}-${MAX_TILT_SPEED_DEG_SEC.toInt()} deg/s."
                    }
                }

                is DelayAction -> {
                    if (action.delaySec <= 0) {
                        errors += "$waypointLabel delay must be greater than 0 seconds."
                    }
                }

                is ImageStartCaptureAction -> {
                    validateImageStartAction(waypointLabel, action, errors, warnings)
                }

                ImageStopCaptureAction -> Unit
            }
        }
    }

    private fun validateImageStartAction(
        waypointLabel: String,
        action: ImageStartCaptureAction,
        errors: MutableList<String>,
        warnings: MutableList<String>,
    ) {
        if (action.periodSec <= 0) {
            errors += "$waypointLabel photo period must be greater than 0 seconds."
        }
        val photoMode = photoModeForResolution(action.resolution)
        if (photoMode == null) {
            warnings += "$waypointLabel uses an unknown FreeFlight photo resolution value."
            return
        }
        if (action.periodSec < photoMode.minPeriodSec) {
            errors += "$waypointLabel ${photoMode.label} photo period must be at least ${photoMode.minPeriodSec}s."
        }
        if (photoMode == FreeFlightPhotoMode.DNG) {
            warnings += "DNG requires a 6s minimum capture period; expect a slow mission."
        }
    }

    private fun validateCaptureActions(
        waypoints: List<FlightWaypoint>,
        errors: MutableList<String>,
        warnings: MutableList<String>,
    ) {
        var captureActive = false
        var startCount = 0
        var stopCount = 0

        waypoints.forEachIndexed { index, waypoint ->
            waypoint.actions.forEach { action ->
                when (action) {
                    is ImageStartCaptureAction -> {
                        startCount += 1
                        if (captureActive) {
                            errors += "Waypoint ${index + 1} starts photo capture while a previous capture is still active."
                        }
                        captureActive = true
                    }

                    ImageStopCaptureAction -> {
                        stopCount += 1
                        if (!captureActive) {
                            errors += "Waypoint ${index + 1} stops photo capture before any capture was started."
                        }
                        captureActive = false
                    }

                    else -> Unit
                }
            }
        }

        if (startCount == 0) {
            errors += "FreeFlight export requires an ImageStartCapture action."
        }
        if (stopCount == 0) {
            errors += "FreeFlight export requires an ImageStopCapture action."
        }
        if (captureActive) {
            errors += "Photo capture is still active at the end of the mission."
        }
        if (startCount != stopCount) {
            warnings += "ImageStartCapture/ImageStopCapture counts differ; review capture segments."
        }
    }

    private fun photoModeForResolution(resolution: Double): FreeFlightPhotoMode? {
        return FreeFlightPhotoMode.entries.firstOrNull { mode ->
            abs(mode.jsonResolution - resolution) <= RESOLUTION_EPSILON
        }
    }

    private fun formatMeters(value: Double): String {
        return if (value % 1.0 == 0.0) {
            "${value.toInt()} m"
        } else {
            String.format(Locale.US, "%.1f m", value)
        }
    }

    private companion object {
        private const val RESOLUTION_EPSILON = 1e-6
        private const val MAX_TILT_ABS_DEG = 90.0
        private const val MIN_TILT_SPEED_DEG_SEC = 1.0
        private const val MAX_TILT_SPEED_DEG_SEC = 180.0
    }
}
