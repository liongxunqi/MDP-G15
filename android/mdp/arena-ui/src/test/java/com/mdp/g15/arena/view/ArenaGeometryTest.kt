package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaConfig
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.RobotPose
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
        assertEquals(GridCoordinate(0, 19), geometry.coordinateAt(41f, 41f))
        assertEquals(GridCoordinate(19, 19), geometry.coordinateAt(459f, 41f))
        assertEquals(GridCoordinate(0, 0), geometry.coordinateAt(41f, 459f))
        assertEquals(GridCoordinate(19, 0), geometry.coordinateAt(459f, 459f))
    }

    @Test
    fun `coordinates outside arena return null`() {
        assertNull(geometry.coordinateAt(39f, 100f))
        assertNull(geometry.coordinateAt(100f, 460f))
    }

    @Test
    fun `cell center round trips to same coordinate`() {
        val coordinate = GridCoordinate(7, 12)
        val bounds = geometry.cellBounds(coordinate)
        assertEquals(coordinate, geometry.coordinateAt(bounds.centerX, bounds.centerY))
    }

    @Test
    fun `robot bounds use bottom left and rectangular dimensions`() {
        val bounds = geometry.robotBounds(RobotPose(0.0, 0.0, 0.0), ArenaConfig())
        val bottomLeft = geometry.cellBounds(GridCoordinate(0, 0))
        assertEquals(bottomLeft.left, bounds.left, 0.001f)
        assertEquals(bottomLeft.bottom, bounds.bottom, 0.001f)
        assertEquals(geometry.arenaLeft + 2f * geometry.cellSize, bounds.right, 0.001f)
        assertEquals(geometry.arenaTop + (20f - 2.1f) * geometry.cellSize, bounds.top, 0.001f)
    }

    @Test
    fun `arena remains square and centered in rectangular viewport`() {
        geometry.update(800, 500, ArenaConfig(), axisPadding = 40f, outerPadding = 10f)

        assertEquals(420f, geometry.arenaWidth, 0.001f)
        assertEquals(geometry.arenaWidth, geometry.arenaHeight, 0.001f)
        assertEquals((800f - geometry.arenaWidth) / 2f, geometry.arenaLeft, 0.001f)
        assertEquals((500f - geometry.arenaHeight) / 2f, geometry.arenaTop, 0.001f)
    }
}
