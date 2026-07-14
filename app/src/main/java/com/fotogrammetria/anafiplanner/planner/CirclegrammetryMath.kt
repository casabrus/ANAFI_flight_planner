package com.fotogrammetria.anafiplanner.planner

import kotlin.math.ceil
import kotlin.math.round
import kotlin.math.tan

object CirclegrammetryMath {
    fun radiusFromOffNadirAngle(
        flightHeightM: Double,
        focusHeightM: Double,
        offNadirAngleDeg: Double,
    ): Double {
        require(flightHeightM > focusHeightM) { "Flight height must exceed focus height." }
        require(offNadirAngleDeg in 0.0..<90.0) { "Off-nadir angle must be within [0, 90)." }
        return (flightHeightM - focusHeightM) * tan(Math.toRadians(offNadirAngleDeg))
    }

    fun offNadirAngleFromRadius(
        flightHeightM: Double,
        focusHeightM: Double,
        radiusM: Double,
    ): Double {
        require(flightHeightM > focusHeightM) { "Flight height must exceed focus height." }
        require(radiusM >= 0.0) { "Radius must be non-negative." }
        return Math.toDegrees(kotlin.math.atan(radiusM / (flightHeightM - focusHeightM)))
    }

    fun resolveGeometry(parameters: CirclegrammetryParameters): CircleGeometry {
        val focusHeightM = parameters.focusTarget.heightAboveTerrainM
        return CircleGeometry(
            flightHeightM = parameters.altitudeM,
            focusHeightM = focusHeightM,
            radiusM = parameters.radiusM,
            offNadirAngleDeg = offNadirAngleFromRadius(
                flightHeightM = parameters.altitudeM,
                focusHeightM = focusHeightM,
                radiusM = parameters.radiusM,
            ),
            mode = CircleGeometryMode.LOCK_RADIUS,
        )
    }

    fun computeCircleGridCount(
        lengthM: Double,
        widthM: Double,
        radiusM: Double,
        overlap: Double,
    ): CircleGridCount {
        require(lengthM > 0.0)
        require(widthM > 0.0)
        require(radiusM > 0.0)
        require(overlap in 0.0..<1.0)

        val diameterM = 2.0 * radiusM
        val spacingM = diameterM * (1.0 - overlap)
        val circlesX = ceil(lengthM / spacingM).toInt().coerceAtLeast(1)
        val circlesY = ceil(widthM / spacingM).toInt().coerceAtLeast(1)

        return CircleGridCount(
            circlesX = circlesX,
            circlesY = circlesY,
            totalCircles = circlesX * circlesY,
            spacingM = spacingM,
        )
    }

    fun computeCircleFootprint(
        lengthM: Double,
        widthM: Double,
        radiusM: Double,
        circlesX: Int,
        circlesY: Int,
        spacingM: Double,
    ): CircleFootprint {
        val diameterM = 2.0 * radiusM
        val coveredLengthM = diameterM + (circlesX - 1) * spacingM
        val coveredWidthM = diameterM + (circlesY - 1) * spacingM
        return CircleFootprint(
            coveredLengthM = coveredLengthM,
            coveredWidthM = coveredWidthM,
            extensionXEachSideM = maxOf(0.0, (coveredLengthM - lengthM) / 2.0),
            extensionYEachSideM = maxOf(0.0, (coveredWidthM - widthM) / 2.0),
        )
    }

    fun generateOrderedCircleCenters(
        origin: Point2D,
        circlesX: Int,
        circlesY: Int,
        spacingM: Double,
    ): List<CircleCenter> {
        val centers = mutableListOf<CircleCenter>()
        for (row in 0 until circlesY) {
            val columns = if (row % 2 == 0) {
                0 until circlesX
            } else {
                (circlesX - 1 downTo 0)
            }
            for (col in columns) {
                centers += CircleCenter(
                    row = row,
                    col = col,
                    xM = origin.x + col * spacingM,
                    yM = origin.y + row * spacingM,
                )
            }
        }
        return centers
    }

    fun isClockwiseForRow(
        row: Int,
        strategy: CircleRotationStrategy,
    ): Boolean {
        return when (strategy) {
            CircleRotationStrategy.CLOCKWISE -> true
            CircleRotationStrategy.COUNTERCLOCKWISE -> false
            CircleRotationStrategy.ALTERNATING_ROWS -> row % 2 == 0
        }
    }

    fun scanCircleOverlaps(
        lengthM: Double,
        widthM: Double,
        radiusM: Double,
        pointsPerCircle: Int,
        speedMode: SpeedMode,
        requestedSpeedMps: Double?,
        photoMode: FreeFlightPhotoMode,
        minOverlap: Double = 0.10,
        maxOverlap: Double = 0.80,
        step: Double = 0.01,
        maxExtensionM: Double? = null,
    ): List<CirclegrammetryOptimizationResult> {
        val results = mutableListOf<CirclegrammetryOptimizationResult>()
        var overlap = minOverlap
        while (overlap <= maxOverlap + epsilon) {
            val grid = computeCircleGridCount(lengthM, widthM, radiusM, overlap)
            val footprint = computeCircleFootprint(
                lengthM = lengthM,
                widthM = widthM,
                radiusM = radiusM,
                circlesX = grid.circlesX,
                circlesY = grid.circlesY,
                spacingM = grid.spacingM,
            )
            val circumferenceM = 2.0 * Math.PI * radiusM
            val photoSpacingM = circumferenceM / pointsPerCircle
            val maxSupportedSpeedMps = photoSpacingM / photoMode.minPeriodSec
            val selectedSpeedMps = selectedSpeedMps(
                speedMode = speedMode,
                requestedSpeedMps = requestedSpeedMps,
                maxSupportedSpeedMps = maxSupportedSpeedMps,
            )
            val estimatedFlightTimeSec = grid.totalCircles * (circumferenceM / selectedSpeedMps)
            val warning = when {
                maxExtensionM != null &&
                    (footprint.extensionXEachSideM > maxExtensionM ||
                        footprint.extensionYEachSideM > maxExtensionM) -> {
                    "Mission footprint extends beyond configured limit"
                }

                speedMode == SpeedMode.CUSTOM &&
                    requestedSpeedMps != null &&
                    requestedSpeedMps > maxSupportedSpeedMps + epsilon -> {
                    "Requested speed exceeds FreeFlight timing constraints"
                }

                else -> null
            }
            results += CirclegrammetryOptimizationResult(
                overlap = roundOverlap(overlap),
                radiusM = radiusM,
                spacingM = grid.spacingM,
                circlesX = grid.circlesX,
                circlesY = grid.circlesY,
                totalCircles = grid.totalCircles,
                coveredLengthM = footprint.coveredLengthM,
                coveredWidthM = footprint.coveredWidthM,
                extensionXEachSideM = footprint.extensionXEachSideM,
                extensionYEachSideM = footprint.extensionYEachSideM,
                estimatedWaypoints = grid.totalCircles * (pointsPerCircle + 1),
                estimatedPhotos = grid.totalCircles * pointsPerCircle,
                estimatedFlightTimeSec = estimatedFlightTimeSec,
                isBeforeThreshold = false,
                isRecommended = false,
                warning = warning,
            )
            overlap += step
        }
        return markThresholdsAndRecommendations(results)
    }

    private fun markThresholdsAndRecommendations(
        results: List<CirclegrammetryOptimizationResult>,
    ): List<CirclegrammetryOptimizationResult> {
        if (results.isEmpty()) {
            return results
        }

        val recommendationsByGroup = results
            .filter { it.warning == null }
            .groupBy { Triple(it.circlesX, it.circlesY, it.totalCircles) }
            .mapValues { (_, group) -> group.maxByOrNull { it.overlap } }

        return results.mapIndexed { index, result ->
            val previous = results.getOrNull(index - 1)
            val isBeforeThreshold = previous == null ||
                previous.totalCircles == result.totalCircles &&
                results.getOrNull(index + 1)?.totalCircles != result.totalCircles
            val key = Triple(result.circlesX, result.circlesY, result.totalCircles)
            result.copy(
                isBeforeThreshold = isBeforeThreshold,
                isRecommended = recommendationsByGroup[key] === result,
            )
        }
    }

    private fun selectedSpeedMps(
        speedMode: SpeedMode,
        requestedSpeedMps: Double?,
        maxSupportedSpeedMps: Double,
    ): Double {
        return when (speedMode) {
            SpeedMode.RECOMMENDED -> maxSupportedSpeedMps
            SpeedMode.CUSTOM -> minOf(requestedSpeedMps ?: maxSupportedSpeedMps, maxSupportedSpeedMps)
        }
    }

    private fun roundOverlap(value: Double): Double {
        return round(value * 100.0) / 100.0
    }

    private const val epsilon = 1e-9
}
