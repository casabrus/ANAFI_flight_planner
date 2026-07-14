package com.fotogrammetria.anafiplanner.terrain

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class GeoTiffElevationReaderTest {
    private val reader = GeoTiffElevationReader()

    @Test
    fun `maps geographic coordinates to pixel coordinates`() {
        val pixel = reader.pixelCoordinates(
            metadata = GeoTiffTileMetadata(
                width = 3601,
                height = 3601,
                tieRasterX = 0.0,
                tieRasterY = 0.0,
                tieLon = 9.0,
                tieLat = 46.0,
                pixelScaleLon = 1.0 / 3600.0,
                pixelScaleLat = 1.0 / 3600.0,
                noData = null,
            ),
            lat = 45.75,
            lon = 9.25,
        )

        assertEquals(900.0, pixel?.x ?: error("missing x"), 0.001)
        assertEquals(900.0, pixel.y, 0.001)
    }

    @Test
    fun `returns null outside tile extent`() {
        val pixel = reader.pixelCoordinates(
            metadata = GeoTiffTileMetadata(
                width = 100,
                height = 100,
                tieRasterX = 0.0,
                tieRasterY = 0.0,
                tieLon = 9.0,
                tieLat = 46.0,
                pixelScaleLon = 0.001,
                pixelScaleLat = 0.001,
                noData = null,
            ),
            lat = 40.0,
            lon = 9.25,
        )

        assertNull(pixel)
    }

    @Test
    fun `interpolates bilinear value`() {
        val value = reader.bilinearValue(
            x = 10.25,
            y = 20.5,
            topLeft = 10.0,
            topRight = 20.0,
            bottomLeft = 30.0,
            bottomRight = 40.0,
        )

        assertEquals(22.5, value, 0.0001)
    }
}
