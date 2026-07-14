package com.fotogrammetria.anafiplanner.planner

data class TakeoffPoint(
    val lat: Double,
    val lon: Double,
    val isUserConfirmed: Boolean = true,
)
