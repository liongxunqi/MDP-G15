package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaConfig
import com.mdp.g15.arena.domain.GridCoordinate
import kotlin.math.floor
import kotlin.math.min

data class CellBounds(
    val left: Float,
    val top: Float,
    val right: Float,
    val bottom: Float,
) {
    val centerX: Float get() = (left + right) / 2f
    val centerY: Float get() = (top + bottom) / 2f
}

class ArenaGeometry {
    var arenaLeft: Float = 0f
        private set
    var arenaTop: Float = 0f
        private set
    var arenaWidth: Float = 0f
        private set
    var arenaHeight: Float = 0f
        private set
    var cellSize: Float = 0f
        private set

    private var config: ArenaConfig = ArenaConfig()

    fun update(
        viewWidth: Int,
        viewHeight: Int,
        config: ArenaConfig,
        axisPadding: Float,
        outerPadding: Float,
    ) {
        this.config = config
        val availableWidth = (viewWidth - axisPadding - outerPadding).coerceAtLeast(0f)
        val availableHeight = (viewHeight - axisPadding - outerPadding).coerceAtLeast(0f)
        cellSize = min(availableWidth / config.columns, availableHeight / config.rows)
        arenaWidth = cellSize * config.columns
        arenaHeight = cellSize * config.rows
        arenaLeft = axisPadding + ((availableWidth - arenaWidth) / 2f)
        arenaTop = outerPadding + ((availableHeight - arenaHeight) / 2f)
    }

    fun coordinateAt(x: Float, y: Float): GridCoordinate? {
        if (cellSize <= 0f) return null
        if (x < arenaLeft || x >= arenaLeft + arenaWidth) return null
        if (y < arenaTop || y >= arenaTop + arenaHeight) return null

        val column = floor((x - arenaLeft) / cellSize).toInt()
        val displayRow = floor((y - arenaTop) / cellSize).toInt()
        val coordinate = GridCoordinate(column, config.rows - 1 - displayRow)
        return coordinate.takeIf(config::contains)
    }

    fun cellBounds(coordinate: GridCoordinate): CellBounds {
        require(config.contains(coordinate)) { "Coordinate must be inside the configured arena." }
        val left = arenaLeft + coordinate.x * cellSize
        val top = arenaTop + (config.rows - 1 - coordinate.y) * cellSize
        return CellBounds(left, top, left + cellSize, top + cellSize)
    }
}
