package com.fotogrammetria.anafiplanner.planner

import kotlin.math.floor
import kotlin.math.tan

object PlannerMath {
    fun footprint(cameraProfile: CameraProfile, altitudeM: Double): FootprintMetrics {
        require(altitudeM > 0.0) { "Altitude must be positive." }
        require(cameraProfile.horizontalFovDeg in 0.1..179.0) { "Horizontal FOV must be within (0, 179]." }
        require(cameraProfile.imageWidthPx > 0 && cameraProfile.imageHeightPx > 0) { "Image dimensions must be positive." }

        val footprintWidthM = 2.0 * altitudeM * tan(Math.toRadians(cameraProfile.horizontalFovDeg / 2.0))
        val footprintHeightM = footprintWidthM * cameraProfile.imageHeightPx / cameraProfile.imageWidthPx
        val gsdCmPerPixel = (footprintWidthM / cameraProfile.imageWidthPx) * 100.0

        return FootprintMetrics(
            footprintWidthM = footprintWidthM,
            footprintHeightM = footprintHeightM,
            gsdCmPerPixel = gsdCmPerPixel,
        )
    }

    fun computeSurveyMetrics(parameters: GridSurveyParameters): SurveyMetrics {
        val footprint = footprint(
            cameraProfile = parameters.cameraProfile,
            altitudeM = parameters.altitudeM,
        )
        val lineSpacingM = footprint.footprintWidthM * (1.0 - parameters.sideOverlap)
        val photoSpacingM = footprint.footprintHeightM * (1.0 - parameters.frontOverlap)
        require(lineSpacingM > 0.0) { "Line spacing must be positive." }
        require(photoSpacingM > 0.0) { "Photo spacing must be positive." }

        return SurveyMetrics(
            footprint = footprint,
            lineSpacingM = lineSpacingM,
            photoSpacingM = photoSpacingM,
            timing = planTiming(
                photoSpacingM = photoSpacingM,
                speedMode = parameters.speedMode,
                requestedSpeedMps = parameters.requestedSpeedMps,
                photoMode = parameters.cameraProfile.photoMode,
            ),
        )
    }

    fun planTiming(
        photoSpacingM: Double,
        speedMode: SpeedMode,
        requestedSpeedMps: Double?,
        photoMode: FreeFlightPhotoMode,
    ): TimingResult {
        val maxSupportedSpeedMps = photoSpacingM / photoMode.minPeriodSec

        return when (speedMode) {
            SpeedMode.RECOMMENDED -> TimingResult(
                feasible = true,
                selectedPeriodSec = photoMode.minPeriodSec,
                selectedSpeedMps = maxSupportedSpeedMps,
                maxSupportedSpeedMps = maxSupportedSpeedMps,
                warning = null,
            )

            SpeedMode.CUSTOM -> {
                val speedMps = requireNotNull(requestedSpeedMps) { "Custom speed mode requires a requested speed." }
                val maxAllowedPeriod = photoSpacingM / speedMps
                if (maxAllowedPeriod < photoMode.minPeriodSec) {
                    TimingResult(
                        feasible = false,
                        selectedPeriodSec = photoMode.minPeriodSec,
                        selectedSpeedMps = maxSupportedSpeedMps,
                        maxSupportedSpeedMps = maxSupportedSpeedMps,
                        warning = "Requested speed is too high for ${photoMode.label} and the selected front overlap.",
                    )
                } else {
                    TimingResult(
                        feasible = true,
                        selectedPeriodSec = floor(maxAllowedPeriod).toInt().coerceAtLeast(photoMode.minPeriodSec),
                        selectedSpeedMps = speedMps,
                        maxSupportedSpeedMps = maxSupportedSpeedMps,
                        warning = null,
                    )
                }
            }
        }
    }
}
