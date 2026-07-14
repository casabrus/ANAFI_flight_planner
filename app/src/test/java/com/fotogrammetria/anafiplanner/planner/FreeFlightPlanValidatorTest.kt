package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class FreeFlightPlanValidatorTest {
    @Test
    fun acceptsBalancedPhotoCaptureSegment() {
        val result = FreeFlightPlanValidator().validate(
            listOf(
                waypoint(
                    actions = listOf(
                        TiltAction(angleDeg = 90.0, speedDegSec = 30.0),
                        ImageStartCaptureAction(
                            periodSec = FreeFlightPhotoMode.JPEG_RECTILINEAR.minPeriodSec,
                            resolution = FreeFlightPhotoMode.JPEG_RECTILINEAR.jsonResolution,
                        ),
                    ),
                ),
                waypoint(
                    longitude = 9.001,
                    actions = listOf(ImageStopCaptureAction),
                ),
            ),
        )

        assertTrue(result.errors.toString(), result.isValid)
        assertTrue(result.warnings.isEmpty())
    }

    @Test
    fun blocksDuplicateStartWithoutStop() {
        val result = FreeFlightPlanValidator().validate(
            listOf(
                waypoint(
                    actions = listOf(
                        ImageStartCaptureAction(
                            periodSec = FreeFlightPhotoMode.JPEG_WIDE.minPeriodSec,
                            resolution = FreeFlightPhotoMode.JPEG_WIDE.jsonResolution,
                        ),
                    ),
                ),
                waypoint(
                    longitude = 9.001,
                    actions = listOf(
                        ImageStartCaptureAction(
                            periodSec = FreeFlightPhotoMode.JPEG_WIDE.minPeriodSec,
                            resolution = FreeFlightPhotoMode.JPEG_WIDE.jsonResolution,
                        ),
                    ),
                ),
            ),
        )

        assertFalse(result.isValid)
        assertTrue(result.errors.any { it.contains("starts photo capture", ignoreCase = true) })
        assertTrue(result.errors.any { it.contains("end of the mission", ignoreCase = true) })
    }

    @Test
    fun blocksInvalidWaypointAltitudeAndSpeed() {
        val result = FreeFlightPlanValidator().validate(
            listOf(
                waypoint(
                    altitudeM = 0.0,
                    speedMps = 0.0,
                    actions = listOf(
                        ImageStartCaptureAction(
                            periodSec = FreeFlightPhotoMode.JPEG_RECTILINEAR.minPeriodSec,
                            resolution = FreeFlightPhotoMode.JPEG_RECTILINEAR.jsonResolution,
                        ),
                    ),
                ),
                waypoint(
                    longitude = 9.001,
                    actions = listOf(ImageStopCaptureAction),
                ),
            ),
        )

        assertFalse(result.isValid)
        assertTrue(result.errors.any { it.contains("altitude", ignoreCase = true) })
        assertTrue(result.errors.any { it.contains("speed", ignoreCase = true) })
    }

    @Test
    fun warnsForDngMinimumPeriod() {
        val result = FreeFlightPlanValidator().validate(
            listOf(
                waypoint(
                    actions = listOf(
                        ImageStartCaptureAction(
                            periodSec = FreeFlightPhotoMode.DNG.minPeriodSec,
                            resolution = FreeFlightPhotoMode.DNG.jsonResolution,
                        ),
                    ),
                ),
                waypoint(
                    longitude = 9.001,
                    actions = listOf(ImageStopCaptureAction),
                ),
            ),
        )

        assertTrue(result.errors.toString(), result.isValid)
        assertTrue(result.warnings.any { it.contains("DNG", ignoreCase = true) })
    }

    @Test
    fun blocksPhotoPeriodBelowObservedMinimum() {
        val result = FreeFlightPlanValidator().validate(
            listOf(
                waypoint(
                    actions = listOf(
                        ImageStartCaptureAction(
                            periodSec = 1,
                            resolution = FreeFlightPhotoMode.DNG.jsonResolution,
                        ),
                    ),
                ),
                waypoint(
                    longitude = 9.001,
                    actions = listOf(ImageStopCaptureAction),
                ),
            ),
        )

        assertFalse(result.isValid)
        assertTrue(result.errors.any { it.contains("at least 6s", ignoreCase = true) })
    }

    private fun waypoint(
        latitude: Double = 45.0,
        longitude: Double = 9.0,
        altitudeM: Double = 30.0,
        speedMps: Double = 3.0,
        actions: List<WaypointAction> = emptyList(),
    ): FlightWaypoint {
        return FlightWaypoint(
            latitude = latitude,
            longitude = longitude,
            altitudeM = altitudeM,
            yawDeg = 90.0,
            speedMps = speedMps,
            actions = actions,
        )
    }
}
