package com.fotogrammetria.anafiplanner.planner

data class FootprintMetrics(
    val footprintWidthM: Double,
    val footprintHeightM: Double,
    val gsdCmPerPixel: Double,
)

data class TimingResult(
    val feasible: Boolean,
    val selectedPeriodSec: Int,
    val selectedSpeedMps: Double,
    val maxSupportedSpeedMps: Double,
    val warning: String?,
)

data class SurveyMetrics(
    val footprint: FootprintMetrics,
    val lineSpacingM: Double,
    val photoSpacingM: Double,
    val timing: TimingResult,
)
