package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class CirclegrammetryMathTest {
    @Test
    fun computesGridCountFromPaperEquation() {
        val grid = CirclegrammetryMath.computeCircleGridCount(
            lengthM = 131.0,
            widthM = 93.0,
            radiusM = 30.0,
            overlap = 0.50,
        )

        assertEquals(5, grid.circlesX)
        assertEquals(4, grid.circlesY)
        assertEquals(20, grid.totalCircles)
        assertEquals(30.0, grid.spacingM, 0.0001)
    }

    @Test
    fun computesRadiusHeightTiltGeometry() {
        val radius = CirclegrammetryMath.radiusFromOffNadirAngle(
            flightHeightM = 50.0,
            focusHeightM = 3.0,
            offNadirAngleDeg = 35.0,
        )

        assertEquals(32.9, radius, 0.2)
        assertEquals(
            35.0,
            CirclegrammetryMath.offNadirAngleFromRadius(50.0, 3.0, radius),
            0.0001,
        )
    }

    @Test
    fun marksRecommendedOverlapWithinSameThresholdGroup() {
        val results = CirclegrammetryMath.scanCircleOverlaps(
            lengthM = 100.0,
            widthM = 80.0,
            radiusM = 30.0,
            pointsPerCircle = 36,
            speedMode = SpeedMode.CUSTOM,
            requestedSpeedMps = 4.0,
            photoMode = FreeFlightPhotoMode.JPEG_RECTILINEAR,
            minOverlap = 0.25,
            maxOverlap = 0.35,
            step = 0.05,
        )

        assertTrue(results.any { it.isRecommended })
        assertTrue(results.any { it.isBeforeThreshold || it.isRecommended })
    }

    @Test
    fun alternatesRowsForCircleCenters() {
        val centers = CirclegrammetryMath.generateOrderedCircleCenters(
            origin = Point2D(0.0, 0.0),
            circlesX = 3,
            circlesY = 2,
            spacingM = 20.0,
        )

        assertEquals(listOf(0, 1, 2, 2, 1, 0), centers.map { it.col })
        assertTrue(CirclegrammetryMath.isClockwiseForRow(0, CircleRotationStrategy.ALTERNATING_ROWS))
        assertTrue(!CirclegrammetryMath.isClockwiseForRow(1, CircleRotationStrategy.ALTERNATING_ROWS))
    }
}
