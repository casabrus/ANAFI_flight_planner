package com.fotogrammetria.anafiplanner.planner

import kotlin.math.abs
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PlannerMathTest {
    @Test
    fun computesFootprintFromAltitudeAndFov() {
        val footprint = PlannerMath.footprint(
            cameraProfile = CameraProfiles.anafiJpegWide,
            altitudeM = 50.0,
        )

        assertClose(89.94, footprint.footprintWidthM, tolerance = 0.2)
        assertClose(67.57, footprint.footprintHeightM, tolerance = 0.2)
        assertClose(1.68, footprint.gsdCmPerPixel, tolerance = 0.05)
    }

    @Test
    fun recommendedModeUsesObservedMinimumPhotoPeriod() {
        val timing = PlannerMath.planTiming(
            photoSpacingM = 15.0,
            speedMode = SpeedMode.RECOMMENDED,
            requestedSpeedMps = null,
            photoMode = FreeFlightPhotoMode.JPEG_WIDE,
        )

        assertTrue(timing.feasible)
        assertEquals(2, timing.selectedPeriodSec)
        assertClose(7.5, timing.selectedSpeedMps)
        assertNull(timing.warning)
    }

    @Test
    fun customModeFlagsInfeasibleSpeed() {
        val timing = PlannerMath.planTiming(
            photoSpacingM = 8.0,
            speedMode = SpeedMode.CUSTOM,
            requestedSpeedMps = 10.0,
            photoMode = FreeFlightPhotoMode.DNG,
        )

        assertFalse(timing.feasible)
        assertEquals(6, timing.selectedPeriodSec)
        assertClose(1.333333, timing.selectedSpeedMps, tolerance = 1e-3)
        assertTrue(timing.warning?.contains("too high", ignoreCase = true) == true)
    }

    private fun assertClose(expected: Double, actual: Double, tolerance: Double = 1e-6) {
        assertTrue("Expected $expected but was $actual", abs(expected - actual) <= tolerance)
    }
}
