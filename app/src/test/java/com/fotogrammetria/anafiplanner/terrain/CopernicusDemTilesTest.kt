package com.fotogrammetria.anafiplanner.terrain

import org.junit.Assert.assertEquals
import org.junit.Test

class CopernicusDemTilesTest {
    @Test
    fun `builds GLO30 tile for milan`() {
        val tile = CopernicusDemTiles.tileFor(
            lat = 45.4642,
            lon = 9.19,
            resolution = DemResolution.GLO30,
        )

        assertEquals("Copernicus_DSM_COG_10_N45_00_E009_00_DEM", tile.name)
        assertEquals(
            "https://copernicus-dem-30m.s3.amazonaws.com/" +
                "Copernicus_DSM_COG_10_N45_00_E009_00_DEM/" +
                "Copernicus_DSM_COG_10_N45_00_E009_00_DEM.tif",
            tile.url,
        )
    }

    @Test
    fun `uses floor for negative longitude`() {
        val tile = CopernicusDemTiles.tileFor(
            lat = 40.7128,
            lon = -74.006,
            resolution = DemResolution.GLO30,
        )

        assertEquals("Copernicus_DSM_COG_10_N40_00_W075_00_DEM", tile.name)
    }

    @Test
    fun `builds GLO90 tile for negative latitude and longitude`() {
        val tile = CopernicusDemTiles.tileFor(
            lat = -1.2,
            lon = -78.5,
            resolution = DemResolution.GLO90,
        )

        assertEquals("Copernicus_DSM_COG_30_S02_00_W079_00_DEM", tile.name)
        assertEquals(
            "https://copernicus-dem-90m.s3.amazonaws.com/" +
                "Copernicus_DSM_COG_30_S02_00_W079_00_DEM/" +
                "Copernicus_DSM_COG_30_S02_00_W079_00_DEM.tif",
            tile.url,
        )
    }
}
