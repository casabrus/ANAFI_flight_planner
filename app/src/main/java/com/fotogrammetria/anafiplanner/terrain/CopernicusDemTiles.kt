package com.fotogrammetria.anafiplanner.terrain

import kotlin.math.abs
import kotlin.math.floor

enum class DemResolution(
    val arcSecondToken: String,
    val bucketName: String,
    val nominalResolutionM: Int,
) {
    GLO30(
        arcSecondToken = "10",
        bucketName = "copernicus-dem-30m",
        nominalResolutionM = 30,
    ),
    GLO90(
        arcSecondToken = "30",
        bucketName = "copernicus-dem-90m",
        nominalResolutionM = 90,
    ),
}

data class DemTileId(
    val resolution: DemResolution,
    val latDeg: Int,
    val lonDeg: Int,
    val name: String,
    val url: String,
)

object CopernicusDemTiles {
    fun tileFor(lat: Double, lon: Double, resolution: DemResolution): DemTileId {
        require(lat in -90.0..90.0) { "Invalid latitude: $lat" }
        require(lon in -180.0..180.0) { "Invalid longitude: $lon" }

        val latDeg = floor(lat).toInt()
        val lonDeg = floor(lon).toInt()
        val northSouth = if (latDeg >= 0) "N" else "S"
        val eastWest = if (lonDeg >= 0) "E" else "W"
        val latAbs = abs(latDeg).toString().padStart(2, '0')
        val lonAbs = abs(lonDeg).toString().padStart(3, '0')
        val name = "Copernicus_DSM_COG_${resolution.arcSecondToken}_${northSouth}${latAbs}_00_${eastWest}${lonAbs}_00_DEM"
        val url = "https://${resolution.bucketName}.s3.amazonaws.com/$name/$name.tif"

        return DemTileId(
            resolution = resolution,
            latDeg = latDeg,
            lonDeg = lonDeg,
            name = name,
            url = url,
        )
    }
}
