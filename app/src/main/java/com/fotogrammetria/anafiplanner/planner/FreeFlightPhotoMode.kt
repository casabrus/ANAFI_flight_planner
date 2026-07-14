package com.fotogrammetria.anafiplanner.planner

enum class FreeFlightPhotoMode(
    val jsonResolution: Double,
    val minPeriodSec: Int,
    val label: String,
) {
    DNG(
        jsonResolution = 14.0,
        minPeriodSec = 6,
        label = "RAW",
    ),
    JPEG_WIDE(
        jsonResolution = 13.600000381469727,
        minPeriodSec = 2,
        label = "WIDE JPG",
    ),
    JPEG_RECTILINEAR(
        jsonResolution = 12.58291244506836,
        minPeriodSec = 1,
        label = "JPG",
    ),
}
