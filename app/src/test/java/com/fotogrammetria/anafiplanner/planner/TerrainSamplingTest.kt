package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TerrainSamplingTest {
    @Test
    fun sampleRouteBySpacingSplitsLongLegsEveryTenMetersOrLess() {
        val waypoints = (0 until 20).map { index ->
            FlightWaypoint(
                latitude = 45.0,
                longitude = 11.0 + index * 0.0002,
                altitudeM = 50.0,
                yawDeg = 90.0,
                speedMps = 3.0,
                actions = emptyList(),
            )
        }

        val samples = TerrainSampling.sampleRouteBySpacing(waypoints, maxSpacingM = 10.0)

        assertTrue(samples.size > waypoints.size)
        assertEquals(waypoints.first().latitude, samples.first().lat, 0.0)
        assertEquals(waypoints.last().longitude, samples.last().lon, 0.000001)
    }

    @Test
    fun sampleRouteSegmentsBySpacingKeepsSegmentMetadata() {
        val waypoints = listOf(
            FlightWaypoint(
                latitude = 45.0,
                longitude = 11.0,
                altitudeM = 50.0,
                yawDeg = 90.0,
                speedMps = 3.0,
                actions = emptyList(),
            ),
            FlightWaypoint(
                latitude = 45.0,
                longitude = 11.0006,
                altitudeM = 50.0,
                yawDeg = 90.0,
                speedMps = 3.0,
                actions = emptyList(),
            ),
        )

        val samples = TerrainSampling.sampleRouteSegmentsBySpacing(waypoints, maxSpacingM = 10.0)

        assertTrue(samples.isNotEmpty())
        assertTrue(samples.all { it.segmentStartIndex == 0 })
        assertTrue(samples.all { it.ratio in 0.0..1.0 })
    }

    @Test
    fun summarizeElevationsIgnoresMissingValues() {
        val summary = TerrainSampling.summarizeElevations(
            listOf(102.4, null, 98.0, 101.6, null),
        )

        assertEquals(5, summary.totalSamples)
        assertEquals(3, summary.validSamples)
        assertEquals(98.0, summary.minElevationM ?: 0.0, 0.0)
        assertEquals(102.4, summary.maxElevationM ?: 0.0, 0.0)
        assertEquals(100.6666666667, summary.averageElevationM ?: 0.0, 0.0001)
    }

    @Test
    fun summarizeElevationsHandlesNoCoverage() {
        val summary = TerrainSampling.summarizeElevations(listOf(null, null))

        assertEquals(2, summary.totalSamples)
        assertEquals(0, summary.validSamples)
        assertNull(summary.minElevationM)
        assertNull(summary.maxElevationM)
        assertNull(summary.averageElevationM)
        assertTrue(summary.validSamples < summary.totalSamples)
    }
}
