package com.fotogrammetria.anafiplanner.planner

import java.util.UUID
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put

class FreeFlightPlanExporter {
    fun export(
        plan: GridSurveyPlan,
        takeoffPoint: TakeoffPoint? = null,
        transferAltitudeM: Double? = null,
    ): String {
        return export(
            title = plan.title,
            polygon = plan.polygon,
            waypoints = plan.waypoints,
            takeoffPoint = takeoffPoint,
            transferAltitudeM = transferAltitudeM,
        )
    }

    fun export(
        title: String,
        polygon: List<GeoPoint>,
        waypoints: List<FlightWaypoint>,
        takeoffPoint: TakeoffPoint? = null,
        transferAltitudeM: Double? = null,
    ): String {
        val exportWaypoints = buildWaypointsForExport(waypoints, takeoffPoint, transferAltitudeM)
        return exportPrepared(
            title = title,
            polygon = polygon,
            waypoints = exportWaypoints,
            takeoffPoint = takeoffPoint,
        )
    }

    fun exportPrepared(
        title: String,
        polygon: List<GeoPoint>,
        waypoints: List<FlightWaypoint>,
        takeoffPoint: TakeoffPoint? = null,
    ): String {
        val bounds = GeoBounds.from(
            points = polygon +
                waypoints.map { GeoPoint(it.latitude, it.longitude) } +
                listOfNotNull(takeoffPoint?.let { GeoPoint(it.lat, it.lon) }),
        )
        val root = buildJsonObject {
            put("version", 1)
            put("title", title)
            put("product", "ANAFI_4K")
            put("productId", 2324)
            put("uuid", UUID.randomUUID().toString())
            put("date", System.currentTimeMillis())
            put("progressive_course_activated", true)
            put("dirty", false)
            put("longitude", bounds.centerLon)
            put("latitude", bounds.centerLat)
            put("longitudeDelta", bounds.longitudeDelta)
            put("latitudeDelta", bounds.latitudeDelta)
            put("zoomLevel", estimateZoomLevel(bounds))
            put("rotation", 0)
            put("tilt", 0)
            put("mapType", 4)
            put(
                "plan",
                buildJsonObject {
                    put("takeoff", JsonArray(emptyList()))
                    put("poi", JsonArray(emptyList()))
                    put(
                        "wayPoints",
                        buildJsonArray {
                            waypoints.forEach { waypoint ->
                                add(exportWaypoint(waypoint))
                            }
                        },
                    )
                },
            )
        }
        return json.encodeToString(JsonObject.serializer(), root)
    }

    fun buildWaypointsForExport(
        plan: GridSurveyPlan,
        takeoffPoint: TakeoffPoint? = null,
        transferAltitudeM: Double? = null,
    ): List<FlightWaypoint> {
        return buildWaypointsForExport(plan.waypoints, takeoffPoint, transferAltitudeM)
    }

    fun buildWaypointsForExport(
        waypoints: List<FlightWaypoint>,
        takeoffPoint: TakeoffPoint? = null,
        transferAltitudeM: Double? = null,
    ): List<FlightWaypoint> {
        if (takeoffPoint == null) {
            return waypoints
        }

        val originalWaypoints = waypoints
        val firstSurveyWaypoint = originalWaypoints.firstOrNull() ?: return originalWaypoints
        if (isSameLocation(
                takeoffLat = takeoffPoint.lat,
                takeoffLon = takeoffPoint.lon,
                waypoint = firstSurveyWaypoint,
            )
        ) {
            return originalWaypoints
        }

        val transferYaw = freeFlightYawDegrees(
            from = GeoPoint(takeoffPoint.lat, takeoffPoint.lon),
            to = GeoPoint(firstSurveyWaypoint.latitude, firstSurveyWaypoint.longitude),
        )

        val transferWaypoint = FlightWaypoint(
            latitude = takeoffPoint.lat,
            longitude = takeoffPoint.lon,
            altitudeM = transferAltitudeM ?: firstSurveyWaypoint.altitudeM,
            yawDeg = transferYaw,
            speedMps = firstSurveyWaypoint.speedMps,
            actions = emptyList(),
            segmentTypeToNext = WaypointSegmentType.LINEAR,
        )
        return listOf(transferWaypoint) + originalWaypoints
    }

    private fun exportWaypoint(waypoint: FlightWaypoint): JsonObject {
        return buildJsonObject {
            put("latitude", waypoint.latitude)
            put("longitude", waypoint.longitude)
            put("altitude", waypoint.altitudeM.roundToInt())
            put("yaw", normalizeYawForExport(waypoint.yawDeg).roundToInt())
            put("speed", waypoint.speedMps)
            put("continue", true)
            put("followPOI", false)
            put("follow", 1)
            put("lastYaw", 0)
            put(
                "actions",
                buildJsonArray {
                    waypoint.actions.forEach { action ->
                        add(exportAction(action))
                    }
                },
            )
        }
    }

    private fun exportAction(action: WaypointAction): JsonObject {
        return when (action) {
            is TiltAction -> buildJsonObject {
                put("type", "Tilt")
                put("angle", -abs(action.angleDeg))
                put("speed", action.speedDegSec)
            }

            is DelayAction -> buildJsonObject {
                put("type", "Delay")
                put("delay", action.delaySec)
            }

            is ImageStartCaptureAction -> buildJsonObject {
                put("type", "ImageStartCapture")
                put("period", action.periodSec)
                put("resolution", action.resolution)
                put("nbOfPictures", action.nbOfPictures)
            }

            ImageStopCaptureAction -> buildJsonObject {
                put("type", "ImageStopCapture")
            }
        }
    }

    private fun estimateZoomLevel(bounds: GeoBounds): Double {
        val maxDelta = maxOf(bounds.latitudeDelta, bounds.longitudeDelta)
        return when {
            maxDelta <= 0.0004 -> 19.0
            maxDelta <= 0.001 -> 18.0
            maxDelta <= 0.003 -> 17.0
            else -> 16.0
        }
    }

    private fun normalizeYawForExport(yawDeg: Double): Double {
        return ((yawDeg % 360.0) + 360.0) % 360.0
    }

    private fun compassBearingDegrees(from: GeoPoint, to: GeoPoint): Double {
        val lat1 = Math.toRadians(from.lat)
        val lat2 = Math.toRadians(to.lat)
        val deltaLon = Math.toRadians(to.lon - from.lon)
        val y = kotlin.math.sin(deltaLon) * kotlin.math.cos(lat2)
        val x = kotlin.math.cos(lat1) * kotlin.math.sin(lat2) -
            kotlin.math.sin(lat1) * kotlin.math.cos(lat2) * kotlin.math.cos(deltaLon)
        return (Math.toDegrees(kotlin.math.atan2(y, x)) + 360.0) % 360.0
    }

    private fun freeFlightYawDegrees(from: GeoPoint, to: GeoPoint): Double {
        val compassBearing = compassBearingDegrees(from, to)
        return (360.0 - compassBearing) % 360.0
    }

    private fun isSameLocation(
        takeoffLat: Double,
        takeoffLon: Double,
        waypoint: FlightWaypoint,
    ): Boolean {
        return kotlin.math.abs(takeoffLat - waypoint.latitude) < 1e-7 &&
            kotlin.math.abs(takeoffLon - waypoint.longitude) < 1e-7
    }

    private data class GeoBounds(
        val centerLat: Double,
        val centerLon: Double,
        val latitudeDelta: Double,
        val longitudeDelta: Double,
    ) {
        companion object {
            fun from(points: List<GeoPoint>): GeoBounds {
                val minLat = points.minOf { it.lat }
                val maxLat = points.maxOf { it.lat }
                val minLon = points.minOf { it.lon }
                val maxLon = points.maxOf { it.lon }
                val latDelta = (maxLat - minLat).coerceAtLeast(0.0002)
                val lonDelta = (maxLon - minLon).coerceAtLeast(0.0002)
                return GeoBounds(
                    centerLat = (minLat + maxLat) / 2.0,
                    centerLon = (minLon + maxLon) / 2.0,
                    latitudeDelta = latDelta,
                    longitudeDelta = lonDelta,
                )
            }
        }
    }

    private companion object {
        private val json = Json { prettyPrint = true }
    }
}
