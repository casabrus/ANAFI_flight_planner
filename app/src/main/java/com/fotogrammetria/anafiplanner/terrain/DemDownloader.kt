package com.fotogrammetria.anafiplanner.terrain

import java.io.File
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL

class DemDownloader(
    private val connectTimeoutMs: Int = 12000,
    private val readTimeoutMs: Int = 12000,
) {
    @Throws(IOException::class)
    fun downloadIfMissing(tile: DemTileId, cache: DemCache): File? {
        val finalFile = cache.fileFor(tile)
        if (finalFile.exists() && finalFile.length() > 0L) {
            return finalFile
        }

        val tempFile = cache.tempFileFor(tile)
        tempFile.delete()

        val connection = (URL(tile.url).openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            connectTimeout = connectTimeoutMs
            readTimeout = readTimeoutMs
            instanceFollowRedirects = true
        }

        try {
            val statusCode = connection.responseCode
            when {
                statusCode == HttpURLConnection.HTTP_NOT_FOUND -> {
                    tempFile.delete()
                    return null
                }

                statusCode !in 200..299 -> {
                    tempFile.delete()
                    throw IOException("DEM download ${tile.resolution.name} $statusCode")
                }
            }

            connection.inputStream.use { input ->
                tempFile.outputStream().use { output ->
                    input.copyTo(output)
                }
            }
        } finally {
            connection.disconnect()
        }

        if (tempFile.length() <= 0L) {
            tempFile.delete()
            throw IOException("DEM download produced an empty file for ${tile.name}")
        }

        if (!tempFile.renameTo(finalFile)) {
            tempFile.copyTo(finalFile, overwrite = true)
            tempFile.delete()
        }

        return finalFile
    }
}
