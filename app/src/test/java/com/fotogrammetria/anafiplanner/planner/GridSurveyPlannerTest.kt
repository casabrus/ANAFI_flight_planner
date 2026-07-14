package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class GridSurveyPlannerTest {
    private val origin = GeoPoint(45.0, 9.0)
    private val projection = GeoProjection(origin)

    @Test
    fun generatesBoustrophedonWaypointsInsidePolygon() {
        val polygon = listOf(
            projection.toGeoPoint(Point2D(0.0, 0.0)),
            projection.toGeoPoint(Point2D(60.0, 0.0)),
            projection.toGeoPoint(Point2D(60.0, 60.0)),
            projection.toGeoPoint(Point2D(0.0, 60.0)),
        )

        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = polygon,
                altitudeM = 50.0,
                frontOverlap = 0.80,
                sideOverlap = 0.70,
                cameraProfile = CameraProfiles.anafiJpegWide,
                speedMode = SpeedMode.RECOMMENDED,
                requestedSpeedMps = null,
                gridAngleDeg = 0.0,
                gimbalSettings = GimbalSettings(tiltDeg = 90.0, tiltSpeedDegSec = 30.0),
            ),
            title = "Test Plan",
        )

        assertEquals(4, plan.waypoints.first().actions.size + plan.waypoints.last().actions.size)
        assertTrue(plan.waypoints.size >= 4)
        assertTrue(plan.estimatedDistanceM > 120.0)
        assertTrue(plan.estimatedPhotoCount >= 4)

        val localWaypoints = plan.waypoints.map {
            projection.toLocalMeters(GeoPoint(it.latitude, it.longitude))
        }
        assertTrue(localWaypoints.all { it.x in -0.5..60.5 && it.y in -0.5..60.5 })
        assertTrue(localWaypoints[0].x < localWaypoints[1].x)
        assertEquals(270.0, plan.waypoints.first().yawDeg, 15.0)
        assertTrue(plan.waypoints.zipWithNext().all { (current, next) ->
            current.yawDeg in 0.0..360.0 && next.yawDeg in 0.0..360.0
        })
        assertEquals(
            "Both endpoints of the first survey strip must keep the same heading",
            plan.waypoints[0].yawDeg,
            plan.waypoints[1].yawDeg,
            0.0001,
        )
        assertEquals(
            "The next strip start must flip heading so rotation happens on the border leg",
            (plan.waypoints[1].yawDeg + 180.0) % 360.0,
            plan.waypoints[2].yawDeg,
            15.0,
        )
    }

    @Test
    fun choosesSurveyStartClosestToTakeoff() {
        val polygon = listOf(
            projection.toGeoPoint(Point2D(0.0, 0.0)),
            projection.toGeoPoint(Point2D(60.0, 0.0)),
            projection.toGeoPoint(Point2D(60.0, 60.0)),
            projection.toGeoPoint(Point2D(0.0, 60.0)),
        )
        val takeoff = TakeoffPoint(
            lat = projection.toGeoPoint(Point2D(62.0, 58.0)).lat,
            lon = projection.toGeoPoint(Point2D(62.0, 58.0)).lon,
            isUserConfirmed = true,
        )

        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = polygon,
                altitudeM = 50.0,
                frontOverlap = 0.80,
                sideOverlap = 0.70,
                cameraProfile = CameraProfiles.anafiJpegWide,
                speedMode = SpeedMode.RECOMMENDED,
                requestedSpeedMps = null,
                gridAngleDeg = 0.0,
                gimbalSettings = GimbalSettings(tiltDeg = 90.0, tiltSpeedDegSec = 30.0),
            ),
            title = "Takeoff Start Test",
            takeoffPoint = takeoff,
        )

        val firstLocal = projection.toLocalMeters(GeoPoint(plan.waypoints.first().latitude, plan.waypoints.first().longitude))
        assertTrue(firstLocal.x > 50.0)
        assertTrue(firstLocal.y > 40.0)
    }

    @Test
    fun keepsTransfersInsideConcavePolygon() {
        val localPolygon = listOf(
            Point2D(0.0, 0.0),
            Point2D(80.0, 0.0),
            Point2D(80.0, 30.0),
            Point2D(30.0, 30.0),
            Point2D(30.0, 80.0),
            Point2D(0.0, 80.0),
        )
        val takeoffGeo = projection.toGeoPoint(Point2D(82.0, 2.0))

        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = localPolygon.map(projection::toGeoPoint),
                altitudeM = 30.0,
                frontOverlap = 0.80,
                sideOverlap = 0.70,
                cameraProfile = CameraProfiles.anafiJpegWide,
                speedMode = SpeedMode.RECOMMENDED,
                requestedSpeedMps = null,
                gridAngleDeg = 0.0,
                gimbalSettings = GimbalSettings(tiltDeg = 90.0, tiltSpeedDegSec = 30.0),
            ),
            title = "Concave Transfer Test",
            takeoffPoint = TakeoffPoint(
                lat = takeoffGeo.lat,
                lon = takeoffGeo.lon,
                isUserConfirmed = true,
            ),
        )

        val localWaypoints = plan.waypoints.map {
            projection.toLocalMeters(GeoPoint(it.latitude, it.longitude))
        }
        assertTrue("Concave transfer routing should add internal waypoint anchors", localWaypoints.size > 10)
        localWaypoints.zipWithNext().forEachIndexed { index, (start, end) ->
            assertTrue(
                "Route segment $index leaves the concave polygon: $start -> $end",
                segmentInsideOrOnPolygon(start, end, localPolygon),
            )
        }
    }

    private fun segmentInsideOrOnPolygon(
        start: Point2D,
        end: Point2D,
        polygon: List<Point2D>,
    ): Boolean {
        return (0..40).all { step ->
            val ratio = step / 40.0
            val point = Point2D(
                x = start.x + ((end.x - start.x) * ratio),
                y = start.y + ((end.y - start.y) * ratio),
            )
            pointInsideOrOnPolygon(point, polygon)
        }
    }

    private fun pointInsideOrOnPolygon(point: Point2D, polygon: List<Point2D>): Boolean {
        polygon.forEachIndexed { index, start ->
            val end = polygon[(index + 1) % polygon.size]
            if (pointOnSegment(point, start, end)) {
                return true
            }
        }

        var inside = false
        var previous = polygon.last()
        polygon.forEach { current ->
            if ((current.y > point.y) != (previous.y > point.y)) {
                val intersectionX = current.x +
                    ((point.y - current.y) * (previous.x - current.x) / (previous.y - current.y))
                if (point.x < intersectionX) {
                    inside = !inside
                }
            }
            previous = current
        }
        return inside
    }

    private fun pointOnSegment(point: Point2D, start: Point2D, end: Point2D): Boolean {
        val cross = ((end.x - start.x) * (point.y - start.y)) -
            ((end.y - start.y) * (point.x - start.x))
        return kotlin.math.abs(cross) <= 0.05 &&
            point.x >= minOf(start.x, end.x) - 0.05 &&
            point.x <= maxOf(start.x, end.x) + 0.05 &&
            point.y >= minOf(start.y, end.y) - 0.05 &&
            point.y <= maxOf(start.y, end.y) + 0.05
    }
}
