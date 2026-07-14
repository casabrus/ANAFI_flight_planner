package com.fotogrammetria.anafiplanner.planner

object CameraProfiles {
    val anafiDngWide = CameraProfile(
        id = "anafi_dng_wide",
        label = "RAW",
        photoMode = FreeFlightPhotoMode.DNG,
        imageWidthPx = 5344,
        imageHeightPx = 4016,
        horizontalFovDeg = 84.0,
        notes = "DNG wide, min observed period 6s",
    )

    val anafiJpegWide = CameraProfile(
        id = "anafi_jpeg_wide",
        label = "WIDE JPG",
        photoMode = FreeFlightPhotoMode.JPEG_WIDE,
        imageWidthPx = 5344,
        imageHeightPx = 4016,
        horizontalFovDeg = 84.0,
        notes = "JPEG wide, min observed period 2s",
    )

    val anafiJpegRectilinear = CameraProfile(
        id = "anafi_jpeg_rectilinear",
        label = "JPG",
        photoMode = FreeFlightPhotoMode.JPEG_RECTILINEAR,
        imageWidthPx = 4096,
        imageHeightPx = 3072,
        horizontalFovDeg = 75.5,
        notes = "Rectilinear profile derived from observed FreeFlight JSON",
    )

    val all: List<CameraProfile> = listOf(
        anafiDngWide,
        anafiJpegWide,
        anafiJpegRectilinear,
    )
}
