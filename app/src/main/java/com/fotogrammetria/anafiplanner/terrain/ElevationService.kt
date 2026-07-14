package com.fotogrammetria.anafiplanner.terrain

import java.io.IOException

interface ElevationService {
    val sourceLabel: String

    @Throws(IOException::class)
    fun fetchElevation(lat: Double, lon: Double): Double?
}
