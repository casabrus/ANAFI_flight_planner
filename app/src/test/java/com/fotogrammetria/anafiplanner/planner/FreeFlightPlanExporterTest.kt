package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FreeFlightPlanExporterTest {
    @Test
    fun exportsImageCaptureActionsAndWaypointArray() {
        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = DemoPlanFactory.samplePolygon(),
                altitudeM = 50.0,
                frontOverlap = 0.80,
                sideOverlap = 0.70,
                cameraProfile = CameraProfiles.anafiJpegRectilinear,
                speedMode = SpeedMode.RECOMMENDED,
                requestedSpeedMps = null,
                gridAngleDeg = 15.0,
                gimbalSettings = DemoPlanFactory.defaultGimbalSettings(),
            ),
            title = "Export Test",
        )

        val json = FreeFlightPlanExporter().export(plan)

        assertTrue(json.contains("\"product\": \"ANAFI_4K\""))
        assertTrue(json.contains("\"wayPoints\""))
        assertTrue(json.contains("\"type\": \"ImageStartCapture\""))
        assertTrue(json.contains("\"type\": \"ImageStopCapture\""))
        assertTrue(json.contains("\"resolution\": 12.58291244506836"))
    }

    @Test
    fun prependsTakeoffTransferWaypointWhenTakeoffDiffersFromSurveyStart() {
        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = DemoPlanFactory.samplePolygon(),
                altitudeM = 50.0,
                frontOverlap = 0.80,
                sideOverlap = 0.70,
                cameraProfile = CameraProfiles.anafiJpegRectilinear,
                speedMode = SpeedMode.RECOMMENDED,
                requestedSpeedMps = null,
                gridAngleDeg = 15.0,
                gimbalSettings = DemoPlanFactory.defaultGimbalSettings(),
            ),
            title = "Export Test With Takeoff",
        )
        val takeoffPoint = TakeoffPoint(
            lat = 45.0715,
            lon = 7.6858,
            isUserConfirmed = true,
        )

        val exportWaypoints = FreeFlightPlanExporter().buildWaypointsForExport(plan, takeoffPoint)

        assertEquals(plan.waypoints.size + 1, exportWaypoints.size)
        assertEquals(takeoffPoint.lat, exportWaypoints.first().latitude, 0.0)
        assertEquals(takeoffPoint.lon, exportWaypoints.first().longitude, 0.0)
        assertTrue(exportWaypoints.first().actions.isEmpty())
        assertTrue(exportWaypoints[1].actions.any { it is TiltAction })
        assertTrue(exportWaypoints[1].actions.any { it is DelayAction })
        assertTrue(exportWaypoints[1].actions.any { it is ImageStartCaptureAction })
    }

    @Test
    fun exportsRoundedAltitudeAndNegativeTiltAngle() {
        val json = FreeFlightPlanExporter().export(
            title = "Rounded Export",
            polygon = DemoPlanFactory.samplePolygon(),
            waypoints = listOf(
                FlightWaypoint(
                    latitude = 45.0,
                    longitude = 9.0,
                    altitudeM = 130.5,
                    yawDeg = 90.0,
                    speedMps = 3.0,
                    actions = listOf(TiltAction(angleDeg = 55.0, speedDegSec = 10.0)),
                ),
                FlightWaypoint(
                    latitude = 45.0,
                    longitude = 9.001,
                    altitudeM = 130.4,
                    yawDeg = 270.0,
                    speedMps = 3.0,
                    actions = emptyList(),
                ),
            ),
        )

        assertTrue(json.contains(""""altitude": 131"""))
        assertTrue(json.contains(""""altitude": 130"""))
        assertTrue(json.contains(""""yaw": 270"""))
        assertTrue(json.contains(""""angle": -55.0"""))
    }
}
