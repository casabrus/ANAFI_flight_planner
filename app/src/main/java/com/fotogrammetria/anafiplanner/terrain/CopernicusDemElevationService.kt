package com.fotogrammetria.anafiplanner.terrain

import android.content.Context
import java.io.IOException
import java.util.Locale
import java.util.concurrent.ConcurrentHashMap
import kotlin.math.round

class CopernicusDemElevationService(
    context: Context,
    private val cache: DemCache = DemCache(context),
    private val downloader: DemDownloader = DemDownloader(),
    private val reader: GeoTiffElevationReader = GeoTiffElevationReader(),
) : ElevationService {
    private val elevationCache = ConcurrentHashMap<String, ElevationSample>()

    override val sourceLabel: String = "Copernicus DEM tiles"

    @Throws(IOException::class)
    override fun fetchElevationSample(lat: Double, lon: Double): ElevationSample? {
        val cacheKey = cacheKey(lat, lon)
        elevationCache[cacheKey]?.let { return it }

        var lastFailure: IOException? = null
        for (resolution in listOf(DemResolution.GLO30, DemResolution.GLO90)) {
            try {
                val tile = CopernicusDemTiles.tileFor(lat, lon, resolution)
                val file = downloader.downloadIfMissing(tile, cache) ?: continue
                val elevation = reader.readElevation(file, lat, lon)?.let { round(it * 10.0) / 10.0 }
                if (elevation != null) {
                    val sample = ElevationSample(
                        elevationM = elevation,
                        resolution = resolution,
                    )
                    elevationCache[cacheKey] = sample
                    return sample
                }
            } catch (error: IOException) {
                val wrapped = IOException("${resolution.name}: ${error.message}", error)
                lastFailure?.addSuppressed(wrapped) ?: run {
                    lastFailure = wrapped
                }
            }
        }

        if (lastFailure != null) {
            throw lastFailure as IOException
        }

        return null
    }

    private fun cacheKey(lat: Double, lon: Double): String {
        return String.format(Locale.US, "%.6f,%.6f", lat, lon)
    }
}
