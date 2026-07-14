package com.fotogrammetria.anafiplanner.terrain

import android.content.Context
import java.io.File

class DemCache(
    context: Context,
) {
    private val rootDir = File(context.filesDir, "dem/copernicus")

    fun fileFor(tile: DemTileId): File {
        return File(directoryFor(tile.resolution), "${tile.name}.tif")
    }

    fun tempFileFor(tile: DemTileId): File {
        return File(directoryFor(tile.resolution), "${tile.name}.tif.download")
    }

    private fun directoryFor(resolution: DemResolution): File {
        val directoryName = when (resolution) {
            DemResolution.GLO30 -> "glo30"
            DemResolution.GLO90 -> "glo90"
        }
        val directory = File(rootDir, directoryName)
        if (!directory.exists()) {
            directory.mkdirs()
        }
        return directory
    }
}
