package com.fotogrammetria.anafiplanner.terrain

import java.io.IOException

data class ElevationSample(
    val elevationM: Double,
    val resolution: DemResolution?,
)

interface ElevationService {
    val sourceLabel: String

    @Throws(IOException::class)
    fun fetchElevation(lat: Double, lon: Double): Double? {
        return fetchElevationSample(lat, lon)?.elevationM
    }

    @Throws(IOException::class)
    fun fetchElevationSample(lat: Double, lon: Double): ElevationSample?
}
