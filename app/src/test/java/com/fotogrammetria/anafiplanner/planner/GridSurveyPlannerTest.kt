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
}
