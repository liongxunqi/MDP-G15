package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaConfig
import com.mdp.g15.arena.domain.GridCoordinate
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class ArenaGeometryTest {
    private val geometry = ArenaGeometry().apply {
        update(
            viewWidth = 500,
            viewHeight = 500,
            config = ArenaConfig(),
            axisPadding = 40f,
            outerPadding = 10f,
        )
    }

    @Test
    fun `bottom-left origin converts all four corners`() {
        assertEquals(GridCoordinate(0, 19), geometry.coordinateAt(41f, 11f))
        assertEquals(GridCoordinate(19, 19), geometry.coordinateAt(489f, 11f))
        assertEquals(GridCoordinate(0, 0), geometry.coordinateAt(41f, 459f))
        assertEquals(GridCoordinate(19, 0), geometry.coordinateAt(489f, 459f))
    }

    @Test
    fun `coordinates outside arena return null`() {
        assertNull(geometry.coordinateAt(39f, 100f))
        assertNull(geometry.coordinateAt(100f, 490f))
    }

    @Test
    fun `cell center round trips to same coordinate`() {
        val coordinate = GridCoordinate(7, 12)
        val bounds = geometry.cellBounds(coordinate)
        assertEquals(coordinate, geometry.coordinateAt(bounds.centerX, bounds.centerY))
    }
}
