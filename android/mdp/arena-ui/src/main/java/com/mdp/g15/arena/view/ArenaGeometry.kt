package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.ArenaConfig
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.footprint
import kotlin.math.floor
import kotlin.math.max
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
        // Reserve the same label-safe inset on every side. This keeps the square
        // arena visually centred even though labels are drawn only left/bottom.
        val contentPadding = max(axisPadding, outerPadding)
        val availableWidth = (viewWidth - contentPadding * 2f).coerceAtLeast(0f)
        val availableHeight = (viewHeight - contentPadding * 2f).coerceAtLeast(0f)
        cellSize = min(availableWidth / config.columns, availableHeight / config.rows)
        arenaWidth = cellSize * config.columns
        arenaHeight = cellSize * config.rows
        arenaLeft = contentPadding + ((availableWidth - arenaWidth) / 2f)
        arenaTop = contentPadding + ((availableHeight - arenaHeight) / 2f)
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

    /**
     * Pixel bounds spanning a [size] x [size] footprint anchored (bottom-left, grid space) at
     * [anchor]. Null if any cell of the footprint falls outside the configured arena.
     */
    fun footprintBounds(anchor: GridCoordinate, size: Int): CellBounds? {
        val cells = anchor.footprint(size)
        if (cells.any { !config.contains(it) }) return null
        val bottomLeft = cellBounds(anchor)
        val topRight = cellBounds(GridCoordinate(anchor.x + size - 1, anchor.y + size - 1))
        return CellBounds(
            left = bottomLeft.left,
            top = topRight.top,
            right = topRight.right,
            bottom = bottomLeft.bottom,
        )
    }
}
