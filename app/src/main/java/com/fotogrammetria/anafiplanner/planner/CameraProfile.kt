package com.fotogrammetria.anafiplanner.planner

data class CameraProfile(
    val id: String,
    val label: String,
    val photoMode: FreeFlightPhotoMode,
    val imageWidthPx: Int,
    val imageHeightPx: Int,
    val horizontalFovDeg: Double,
    val notes: String,
)
