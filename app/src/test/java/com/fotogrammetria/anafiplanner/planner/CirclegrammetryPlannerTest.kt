package com.fotogrammetria.anafiplanner.planner

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class CirclegrammetryPlannerTest {
    private val projection = GeoProjection(DemoPlanFactory.samplePolygon().first())

    @Test
    fun generatesClosedCircleWaypointsWithCaptureActions() {
        val parameters = CirclegrammetryParameters(
            polygon = DemoPlanFactory.samplePolygon(),
            altitudeM = 45.0,
            radiusM = 30.0,
            requestedOverlap = 0.50,
            optimizedOverlap = null,
            maxExtensionOutsideAreaM = 30.0,
            speedMode = SpeedMode.RECOMMENDED,
            requestedSpeedMps = null,
            cameraTiltDeg = 55.0,
            cameraTiltSpeedDegSec = 30.0,
            photoMode = FreeFlightPhotoMode.JPEG_RECTILINEAR,
            cameraProfile = CameraProfiles.anafiJpegRectilinear,
            pointsPerCircle = 24,
            rotationStrategy = CircleRotationStrategy.ALTERNATING_ROWS,
            shotMode = CircleShotMode.AUTO,
            optimizationMode = CircleOptimizationMode.QUALITY_50_PERCENT,
            focusTarget = FocusTarget(FocusTargetMode.CUSTOM, 3.0),
        )

        val plan = CirclegrammetryPlanner().plan(parameters, "Circle Test")

        assertTrue(plan.waypoints.isNotEmpty())
        assertTrue(plan.waypoints.first().actions.any { it is TiltAction })
        assertTrue(plan.waypoints.first().actions.any { it is DelayAction })
        assertTrue(plan.waypoints.first().actions.any { it is ImageStartCaptureAction })
        assertEquals(ImageStopCaptureAction, plan.waypoints.last().actions.single())
        assertTrue(plan.estimate.selectedSpeedMps <= plan.estimate.maxSupportedSpeedMps + 1e-9)
    }

    @Test
    fun choosesCircleStartNearTakeoff() {
        val polygon = DemoPlanFactory.samplePolygon()
        val projection = GeoProjection(polygon.first())
        val takeoffGeo = projection.toGeoPoint(Point2D(120.0, 20.0))
        val parameters = CirclegrammetryParameters(
            polygon = polygon,
            altitudeM = 45.0,
            radiusM = 30.0,
            requestedOverlap = 0.50,
            optimizedOverlap = null,
            maxExtensionOutsideAreaM = 30.0,
            speedMode = SpeedMode.RECOMMENDED,
            requestedSpeedMps = null,
            cameraTiltDeg = 55.0,
            cameraTiltSpeedDegSec = 30.0,
            photoMode = FreeFlightPhotoMode.JPEG_RECTILINEAR,
            cameraProfile = CameraProfiles.anafiJpegRectilinear,
            pointsPerCircle = 24,
            rotationStrategy = CircleRotationStrategy.ALTERNATING_ROWS,
            shotMode = CircleShotMode.AUTO,
            optimizationMode = CircleOptimizationMode.QUALITY_50_PERCENT,
            focusTarget = FocusTarget(FocusTargetMode.CUSTOM, 3.0),
        )

        val plan = CirclegrammetryPlanner().plan(
            parameters = parameters,
            title = "Circle Start Test",
            takeoffPoint = TakeoffPoint(takeoffGeo.lat, takeoffGeo.lon, true),
        )

        val firstLocal = projection.toLocalMeters(GeoPoint(plan.waypoints.first().latitude, plan.waypoints.first().longitude))
        assertTrue(firstLocal.x > 50.0)
    }

    @Test
    fun exportsCircleWaypointYawInFreeFlightConvention() {
        val parameters = CirclegrammetryParameters(
            polygon = DemoPlanFactory.samplePolygon(),
            altitudeM = 45.0,
            radiusM = 30.0,
            requestedOverlap = 0.50,
            optimizedOverlap = 0.50,
            maxExtensionOutsideAreaM = 30.0,
            speedMode = SpeedMode.RECOMMENDED,
            requestedSpeedMps = null,
            cameraTiltDeg = 55.0,
            cameraTiltSpeedDegSec = 30.0,
            photoMode = FreeFlightPhotoMode.JPEG_RECTILINEAR,
            cameraProfile = CameraProfiles.anafiJpegRectilinear,
            pointsPerCircle = 24,
            rotationStrategy = CircleRotationStrategy.ALTERNATING_ROWS,
            shotMode = CircleShotMode.AUTO,
            optimizationMode = CircleOptimizationMode.USER_FIXED_OVERLAP,
            focusTarget = FocusTarget(FocusTargetMode.CUSTOM, 3.0),
        )

        val plan = CirclegrammetryPlanner().plan(parameters, "Circle Yaw Test")
        val circleWaypoints = plan.waypoints.take(25)
        val center = Point2D(
            x = circleWaypoints.map { projection.toLocalMeters(GeoPoint(it.latitude, it.longitude)).x }.average(),
            y = circleWaypoints.map { projection.toLocalMeters(GeoPoint(it.latitude, it.longitude)).y }.average(),
        )

        val westWaypoint = circleWaypoints.minByOrNull {
            projection.toLocalMeters(GeoPoint(it.latitude, it.longitude)).x
        } ?: error("missing west waypoint")

        assertEquals(270.0, westWaypoint.yawDeg, 1.0)

        val westPoint = projection.toLocalMeters(GeoPoint(westWaypoint.latitude, westWaypoint.longitude))
        assertTrue(center.x > westPoint.x)
    }
}
