package com.fotogrammetria.anafiplanner.terrain

import java.io.File
import java.io.IOException
import kotlin.math.abs
import kotlin.math.floor
import mil.nga.tiff.FieldTagType
import mil.nga.tiff.FileDirectory
import mil.nga.tiff.ImageWindow
import mil.nga.tiff.Rasters
import mil.nga.tiff.TiffReader

class GeoTiffElevationReader(
    private val maxLoadedTiles: Int = 2,
) {
    private val loadedTiles = object : LinkedHashMap<String, LoadedGeoTiffTile>(4, 0.75f, true) {
        override fun removeEldestEntry(eldest: MutableMap.MutableEntry<String, LoadedGeoTiffTile>?): Boolean {
            return size > maxLoadedTiles
        }
    }

    @Synchronized
    @Throws(IOException::class)
    fun readElevation(file: File, lat: Double, lon: Double): Double? {
        val tile = loadTile(file)
        return sampleElevation(tile, lat, lon)
    }

    @Synchronized
    @Throws(IOException::class)
    private fun loadTile(file: File): LoadedGeoTiffTile {
        val cacheKey = file.absolutePath
        loadedTiles[cacheKey]?.let { return it }

        val image = TiffReader.readTiff(file, true)
        val directory = image.fileDirectories.firstOrNull()
            ?: throw IOException("GeoTIFF has no file directories: ${file.name}")
        val metadata = extractMetadata(directory)
        val loadedTile = LoadedGeoTiffTile(
            directory = directory,
            metadata = metadata,
        )
        loadedTiles[cacheKey] = loadedTile
        return loadedTile
    }

    internal fun sampleElevation(tile: LoadedGeoTiffTile, lat: Double, lon: Double): Double? {
        val pixel = pixelCoordinates(tile.metadata, lat, lon) ?: return null
        return bilinearSample(tile, pixel.x, pixel.y)
    }

    internal fun pixelCoordinates(
        metadata: GeoTiffTileMetadata,
        lat: Double,
        lon: Double,
    ): PixelCoordinate? {
        val x = metadata.tieRasterX + (lon - metadata.tieLon) / metadata.pixelScaleLon
        val y = metadata.tieRasterY + (metadata.tieLat - lat) / metadata.pixelScaleLat
        if (!x.isFinite() || !y.isFinite()) {
            return null
        }
        if (x < 0.0 || y < 0.0 || x > metadata.width - 1 || y > metadata.height - 1) {
            return null
        }
        return PixelCoordinate(x = x, y = y)
    }

    internal fun bilinearValue(
        x: Double,
        y: Double,
        topLeft: Double,
        topRight: Double,
        bottomLeft: Double,
        bottomRight: Double,
    ): Double {
        val fractionX = x - floor(x)
        val fractionY = y - floor(y)
        val top = topLeft * (1.0 - fractionX) + topRight * fractionX
        val bottom = bottomLeft * (1.0 - fractionX) + bottomRight * fractionX
        return top * (1.0 - fractionY) + bottom * fractionY
    }

    private fun bilinearSample(tile: LoadedGeoTiffTile, x: Double, y: Double): Double? {
        val metadata = tile.metadata
        val x0 = floor(x).toInt().coerceIn(0, metadata.width - 1)
        val y0 = floor(y).toInt().coerceIn(0, metadata.height - 1)
        val x1 = (x0 + 1).coerceAtMost(metadata.width - 1)
        val y1 = (y0 + 1).coerceAtMost(metadata.height - 1)

        if (x0 == x1 || y0 == y1) {
            return readNearest(tile, x0, y0)
        }

        val window = ImageWindow(x0, y0, x1 + 1, y1 + 1)
        val rasters = tile.directory.readRasters(window, intArrayOf(0), true, false)
        val topLeft = valueOrNull(rasters.getFirstPixelSample(0, 0), metadata.noData)
        val topRight = valueOrNull(rasters.getFirstPixelSample(1, 0), metadata.noData)
        val bottomLeft = valueOrNull(rasters.getFirstPixelSample(0, 1), metadata.noData)
        val bottomRight = valueOrNull(rasters.getFirstPixelSample(1, 1), metadata.noData)

        if (topLeft != null && topRight != null && bottomLeft != null && bottomRight != null) {
            return bilinearValue(x, y, topLeft, topRight, bottomLeft, bottomRight)
        }

        return nearestValidValue(
            x = x,
            y = y,
            candidates = listOf(
                SampleCandidate(x0.toDouble(), y0.toDouble(), topLeft),
                SampleCandidate(x1.toDouble(), y0.toDouble(), topRight),
                SampleCandidate(x0.toDouble(), y1.toDouble(), bottomLeft),
                SampleCandidate(x1.toDouble(), y1.toDouble(), bottomRight),
            ),
        )
    }

    private fun readNearest(tile: LoadedGeoTiffTile, x: Int, y: Int): Double? {
        val rasters = tile.directory.readRasters(ImageWindow(x, y), intArrayOf(0), true, false)
        return valueOrNull(rasters.getFirstPixelSample(0, 0), tile.metadata.noData)
    }

    private fun nearestValidValue(
        x: Double,
        y: Double,
        candidates: List<SampleCandidate>,
    ): Double? {
        return candidates
            .filter { it.value != null }
            .minByOrNull { candidate ->
                val dx = x - candidate.x
                val dy = y - candidate.y
                dx * dx + dy * dy
            }
            ?.value
    }

    private fun valueOrNull(rawValue: Number?, noData: Double?): Double? {
        val value = rawValue?.toDouble() ?: return null
        if (!value.isFinite()) {
            return null
        }
        if (noData != null && abs(value - noData) <= 1e-6) {
            return null
        }
        return value
    }

    private fun extractMetadata(directory: FileDirectory): GeoTiffTileMetadata {
        val modelPixelScale = directory.modelPixelScale
            ?: throw IOException("GeoTIFF missing ModelPixelScale tag")
        val modelTiepoint = directory.modelTiepoint
            ?: throw IOException("GeoTIFF missing ModelTiepoint tag")
        if (modelPixelScale.size < 2 || modelTiepoint.size < 6) {
            throw IOException("GeoTIFF georeferencing tags are incomplete")
        }

        val tieRasterX = modelTiepoint[0]
        val tieRasterY = modelTiepoint[1]
        val tieLon = modelTiepoint[3]
        val tieLat = modelTiepoint[4]
        val pixelScaleLon = modelPixelScale[0]
        val pixelScaleLat = modelPixelScale[1]
        if (pixelScaleLon <= 0.0 || pixelScaleLat <= 0.0) {
            throw IOException("GeoTIFF has invalid pixel scale")
        }

        return GeoTiffTileMetadata(
            width = directory.imageWidth.toInt(),
            height = directory.imageHeight.toInt(),
            tieRasterX = tieRasterX,
            tieRasterY = tieRasterY,
            tieLon = tieLon,
            tieLat = tieLat,
            pixelScaleLon = pixelScaleLon,
            pixelScaleLat = pixelScaleLat,
            noData = directory.getStringEntryValue(FieldTagType.GDAL_NODATA)?.toDoubleOrNull(),
        )
    }
}

internal data class LoadedGeoTiffTile(
    val directory: FileDirectory,
    val metadata: GeoTiffTileMetadata,
)

internal data class GeoTiffTileMetadata(
    val width: Int,
    val height: Int,
    val tieRasterX: Double,
    val tieRasterY: Double,
    val tieLon: Double,
    val tieLat: Double,
    val pixelScaleLon: Double,
    val pixelScaleLat: Double,
    val noData: Double?,
)

internal data class PixelCoordinate(
    val x: Double,
    val y: Double,
)

private data class SampleCandidate(
    val x: Double,
    val y: Double,
    val value: Double?,
)
