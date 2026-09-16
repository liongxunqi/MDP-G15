package com.mdp.g15.arena.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.os.Handler
import android.os.Looper
import android.util.AttributeSet
import android.util.TypedValue
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import android.view.ViewConfiguration
import androidx.core.content.ContextCompat
import com.mdp.g15.arena.R
import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.Obstacle
import com.mdp.g15.arena.domain.footprint
import kotlin.math.hypot
import kotlin.math.min

class ArenaGridView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {
    var interactionListener: ArenaInteractionListener? = null

    private var state = ArenaState()
    private var placementMode = false
    private val geometry = ArenaGeometry()
    private val density = resources.displayMetrics.density
    private val axisPadding = 30f * density
    private val outerPadding = 8f * density
    private val handler = Handler(Looper.getMainLooper())
    private val touchSlop = ViewConfiguration.get(context).scaledTouchSlop.toFloat()

    private var scale = 1f
    private var panX = 0f
    private var panY = 0f
    private var lastFocusX = 0f
    private var lastFocusY = 0f
    private val scaleGestureDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
                lastFocusX = detector.focusX
                lastFocusY = detector.focusY
                return true
            }

            override fun onScale(detector: ScaleGestureDetector): Boolean {
                // Anchor on the point under the fingers so scale and two-finger pan compose naturally.
                val contentX = (lastFocusX - panX) / scale
                val contentY = (lastFocusY - panY) / scale
                scale = (scale * detector.scaleFactor).coerceIn(MIN_SCALE, MAX_SCALE)
                panX = detector.focusX - contentX * scale
                panY = detector.focusY - contentY * scale
                lastFocusX = detector.focusX
                lastFocusY = detector.focusY
                clampPan()
                invalidate()
                return true
            }
        },
    )

    private val backgroundPaint = fillPaint(R.color.arena_background)
    private val gridPaint = strokePaint(R.color.arena_grid_line, 1f * density)
    private val axisTextPaint = textPaint(R.color.arena_axis_text, 10f, Paint.Align.CENTER)
    private val obstaclePaint = fillPaint(R.color.arena_obstacle)
    private val obstacleTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = sp(12f)
        textAlign = Paint.Align.CENTER
        typeface = android.graphics.Typeface.DEFAULT_BOLD
    }
    private val selectedPaint = strokePaint(R.color.arena_selected, 3f * density)
    private val facePaint = strokePaint(R.color.arena_target_face, 4f * density)
    private val robotPaint = fillPaint(R.color.arena_robot)
    private val robotDirectionPaint = fillPaint(R.color.arena_robot_direction)
    private val dragPaint = Paint(obstaclePaint).apply { alpha = 165 }
    private val dragInvalidPaint = Paint().apply {
        color = ContextCompat.getColor(context, R.color.arena_target_face)
        style = Paint.Style.STROKE
        strokeWidth = 3f * density
    }

    private var downX = 0f
    private var downY = 0f
    private var dragX = 0f
    private var dragY = 0f
    private var pressedObstacleId: Int? = null
    private var draggingObstacleId: Int? = null
    private var robotPressed = false
    private var draggingRobot = false
    /** Offset, in cells, of the pressed point from the robot's footprint anchor (bottom-left). */
    private var robotGrabOffsetX = 0
    private var robotGrabOffsetY = 0
    private var longPressTriggered = false
    private var multiTouchOccurred = false

    private val longPressRunnable = Runnable {
        when {
            pressedObstacleId != null -> {
                draggingObstacleId = pressedObstacleId
                announceForAccessibility("Moving obstacle $draggingObstacleId")
            }
            robotPressed -> {
                draggingRobot = true
                announceForAccessibility("Moving robot")
            }
            else -> return@Runnable
        }
        longPressTriggered = true
        parent?.requestDisallowInterceptTouchEvent(true)
        invalidate()
    }

    init {
        isFocusable = true
        isClickable = true
        contentDescription = context.getString(R.string.arena_content_description)
    }

    fun render(state: ArenaState, placementMode: Boolean) {
        this.state = state
        this.placementMode = placementMode
        contentDescription = buildContentDescription(state)
        requestLayout()
        invalidate()
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val desired = (520f * density).toInt()
        val measuredWidth = resolveSize(desired, widthMeasureSpec)
        val measuredHeight = resolveSize(desired, heightMeasureSpec)
        setMeasuredDimension(measuredWidth, measuredHeight)
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        updateGeometry(w, h)
        clampPan()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        updateGeometry(width, height)
        val saveCount = canvas.save()
        canvas.translate(panX, panY)
        canvas.scale(scale, scale)
        drawArena(canvas)
        drawObstacles(canvas)
        if (!draggingRobot) {
            state.robot?.let { drawRobot(canvas, it.position, it.direction) }
        }
        drawDragPreview(canvas)
        canvas.restoreToCount(saveCount)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleGestureDetector.onTouchEvent(event)

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                downX = event.x
                downY = event.y
                dragX = event.x
                dragY = event.y
                longPressTriggered = false
                multiTouchOccurred = false
                val coordinate = geometry.coordinateAt(toContentX(event.x), toContentY(event.y))
                val robot = state.robot
                val robotHere = coordinate != null && robot != null &&
                    coordinate in robot.position.footprint(state.config.robotFootprintCells)
                pressedObstacleId = if (robotHere) null else coordinate?.let(::obstacleAt)?.id
                robotPressed = robotHere
                if (robotHere) {
                    robotGrabOffsetX = coordinate!!.x - robot!!.position.x
                    robotGrabOffsetY = coordinate.y - robot.position.y
                }
                if (pressedObstacleId != null || robotPressed) {
                    handler.postDelayed(longPressRunnable, ViewConfiguration.getLongPressTimeout().toLong())
                }
                return coordinate != null || pressedObstacleId != null || robotPressed
            }

            MotionEvent.ACTION_POINTER_DOWN -> {
                // A second finger means pinch/pan, not a drag: cancel any pending gesture.
                handler.removeCallbacks(longPressRunnable)
                pressedObstacleId = null
                draggingObstacleId = null
                robotPressed = false
                draggingRobot = false
                longPressTriggered = false
                multiTouchOccurred = true
                parent?.requestDisallowInterceptTouchEvent(true)
                invalidate()
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                if (event.pointerCount > 1) return true
                dragX = event.x
                dragY = event.y
                if (!longPressTriggered && hypot(event.x - downX, event.y - downY) > touchSlop) {
                    handler.removeCallbacks(longPressRunnable)
                }
                if (draggingObstacleId != null || draggingRobot) invalidate()
                return true
            }

            MotionEvent.ACTION_POINTER_UP -> {
                return true
            }

            MotionEvent.ACTION_UP -> {
                handler.removeCallbacks(longPressRunnable)
                val draggedId = draggingObstacleId
                when {
                    draggedId != null -> {
                        val destination = geometry.coordinateAt(toContentX(event.x), toContentY(event.y))
                        if (destination == null) {
                            interactionListener?.onRemoveObstacle(draggedId)
                            announceForAccessibility("Obstacle $draggedId removed")
                        } else {
                            interactionListener?.onMoveObstacle(draggedId, destination)
                        }
                    }
                    draggingRobot -> {
                        val underFinger = geometry.coordinateAt(toContentX(event.x), toContentY(event.y))
                        val destination = underFinger?.let {
                            GridCoordinate(it.x - robotGrabOffsetX, it.y - robotGrabOffsetY)
                        }
                        if (destination != null) {
                            interactionListener?.onMoveRobot(destination)
                        }
                    }
                    multiTouchOccurred -> Unit
                    else -> {
                        performClick()
                        val coordinate = geometry.coordinateAt(toContentX(event.x), toContentY(event.y))
                        if (coordinate != null) {
                            val obstacle = obstacleAt(coordinate)
                            when {
                                placementMode -> interactionListener?.onAddObstacle(coordinate)
                                obstacle != null -> interactionListener?.onSelectObstacle(obstacle.id)
                                else -> interactionListener?.onSelectObstacle(null)
                            }
                        }
                    }
                }
                clearGesture()
                invalidate()
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                handler.removeCallbacks(longPressRunnable)
                clearGesture()
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    override fun onDetachedFromWindow() {
        handler.removeCallbacks(longPressRunnable)
        super.onDetachedFromWindow()
    }

    private fun drawArena(canvas: Canvas) {
        val left = geometry.arenaLeft
        val top = geometry.arenaTop
        val arenaWidth = geometry.arenaWidth
        val arenaHeight = geometry.arenaHeight
        canvas.drawRect(left, top, left + arenaWidth, top + arenaHeight, backgroundPaint)

        for (column in 0..state.config.columns) {
            val x = left + column * geometry.cellSize
            canvas.drawLine(x, top, x, top + arenaHeight, gridPaint)
        }
        for (row in 0..state.config.rows) {
            val y = top + row * geometry.cellSize
            canvas.drawLine(left, y, left + arenaWidth, y, gridPaint)
        }

        axisTextPaint.textSize = min(sp(10f), geometry.cellSize * 0.42f)
        for (column in 0 until state.config.columns) {
            val bounds = geometry.cellBounds(GridCoordinate(column, 0))
            canvas.drawText(
                column.toString(),
                bounds.centerX,
                geometry.arenaTop + geometry.arenaHeight + axisTextPaint.textSize + 2f * density,
                axisTextPaint,
            )
        }
        axisTextPaint.textAlign = Paint.Align.RIGHT
        for (row in 0 until state.config.rows) {
            val bounds = geometry.cellBounds(GridCoordinate(0, row))
            canvas.drawText(
                row.toString(),
                geometry.arenaLeft - 4f * density,
                bounds.centerY - (axisTextPaint.ascent() + axisTextPaint.descent()) / 2f,
                axisTextPaint,
            )
        }
        axisTextPaint.textAlign = Paint.Align.CENTER
    }

    private fun drawObstacles(canvas: Canvas) {
        val draggingId = draggingObstacleId
        state.obstacles.toSortedMap().values.forEach { obstacle ->
            if (obstacle.id != draggingId) drawObstacle(canvas, obstacle)
        }
    }

    private fun drawObstacle(canvas: Canvas, obstacle: Obstacle) {
        val bounds = geometry.cellBounds(obstacle.position)
        val inset = geometry.cellSize * 0.08f
        val rect = RectF(
            bounds.left + inset,
            bounds.top + inset,
            bounds.right - inset,
            bounds.bottom - inset,
        )
        canvas.drawRect(rect, obstaclePaint)

        obstacleTextPaint.textSize = if (obstacle.targetId == null) {
            geometry.cellSize * 0.38f
        } else {
            geometry.cellSize * 0.56f
        }
        val label = obstacle.targetId ?: obstacle.id.toString()
        canvas.drawText(
            label,
            rect.centerX(),
            rect.centerY() - (obstacleTextPaint.ascent() + obstacleTextPaint.descent()) / 2f,
            obstacleTextPaint,
        )

        obstacle.targetFace?.let { drawFace(canvas, rect, it) }
        if (state.selectedObstacleId == obstacle.id) canvas.drawRect(rect, selectedPaint)
    }

    private fun drawFace(canvas: Canvas, rect: RectF, direction: Direction) {
        when (direction) {
            Direction.NORTH -> canvas.drawLine(rect.left, rect.top, rect.right, rect.top, facePaint)
            Direction.EAST -> canvas.drawLine(rect.right, rect.top, rect.right, rect.bottom, facePaint)
            Direction.SOUTH -> canvas.drawLine(rect.left, rect.bottom, rect.right, rect.bottom, facePaint)
            Direction.WEST -> canvas.drawLine(rect.left, rect.top, rect.left, rect.bottom, facePaint)
        }
    }

    /** [anchor] is the robot footprint's bottom-left cell (see [ArenaConfig.robotFootprintCells]). */
    private fun drawRobot(canvas: Canvas, anchor: GridCoordinate, direction: Direction) {
        val bounds = geometry.footprintBounds(anchor, state.config.robotFootprintCells) ?: return
        val radius = (bounds.right - bounds.left) * 0.38f
        canvas.drawCircle(bounds.centerX, bounds.centerY, radius, robotPaint)

        val nose = Path()
        val halfBase = radius * 0.42f
        when (direction) {
            Direction.NORTH -> nose.apply {
                moveTo(bounds.centerX, bounds.centerY - radius * 1.28f)
                lineTo(bounds.centerX - halfBase, bounds.centerY - radius * 0.2f)
                lineTo(bounds.centerX + halfBase, bounds.centerY - radius * 0.2f)
            }
            Direction.EAST -> nose.apply {
                moveTo(bounds.centerX + radius * 1.28f, bounds.centerY)
                lineTo(bounds.centerX + radius * 0.2f, bounds.centerY - halfBase)
                lineTo(bounds.centerX + radius * 0.2f, bounds.centerY + halfBase)
            }
            Direction.SOUTH -> nose.apply {
                moveTo(bounds.centerX, bounds.centerY + radius * 1.28f)
                lineTo(bounds.centerX - halfBase, bounds.centerY + radius * 0.2f)
                lineTo(bounds.centerX + halfBase, bounds.centerY + radius * 0.2f)
            }
            Direction.WEST -> nose.apply {
                moveTo(bounds.centerX - radius * 1.28f, bounds.centerY)
                lineTo(bounds.centerX - radius * 0.2f, bounds.centerY - halfBase)
                lineTo(bounds.centerX - radius * 0.2f, bounds.centerY + halfBase)
            }
        }
        nose.close()
        canvas.drawPath(nose, robotDirectionPaint)
    }

    private fun drawDragPreview(canvas: Canvas) {
        val obstacleId = draggingObstacleId
        when {
            obstacleId != null -> drawObstacleDragPreview(canvas, obstacleId)
            draggingRobot -> drawRobotDragPreview(canvas)
        }
    }

    private fun drawObstacleDragPreview(canvas: Canvas, obstacleId: Int) {
        val contentX = toContentX(dragX)
        val contentY = toContentY(dragY)
        val destination = geometry.coordinateAt(contentX, contentY)
        if (destination == null) {
            drawInvalidDropIndicator(canvas, contentX, contentY)
            return
        }
        val bounds = geometry.cellBounds(destination)
        val inset = geometry.cellSize * 0.08f
        val rect = RectF(bounds.left + inset, bounds.top + inset, bounds.right - inset, bounds.bottom - inset)
        canvas.drawRect(rect, dragPaint)
        obstacleTextPaint.textSize = geometry.cellSize * 0.38f
        canvas.drawText(
            obstacleId.toString(),
            rect.centerX(),
            rect.centerY() - (obstacleTextPaint.ascent() + obstacleTextPaint.descent()) / 2f,
            obstacleTextPaint,
        )
    }

    private fun drawRobotDragPreview(canvas: Canvas) {
        val direction = state.robot?.direction ?: return
        val contentX = toContentX(dragX)
        val contentY = toContentY(dragY)
        val underFinger = geometry.coordinateAt(contentX, contentY)
        val anchor = underFinger?.let { GridCoordinate(it.x - robotGrabOffsetX, it.y - robotGrabOffsetY) }
        val bounds = anchor?.let { geometry.footprintBounds(it, state.config.robotFootprintCells) }
        if (anchor == null || bounds == null) {
            drawInvalidDropIndicator(canvas, contentX, contentY)
            return
        }
        drawRobot(canvas, anchor, direction)
    }

    private fun drawInvalidDropIndicator(canvas: Canvas, x: Float, y: Float) {
        val radius = geometry.cellSize * 0.45f
        canvas.drawCircle(x, y, radius, dragInvalidPaint)
        canvas.drawLine(x - radius, y - radius, x + radius, y + radius, dragInvalidPaint)
    }

    private fun obstacleAt(coordinate: GridCoordinate): Obstacle? =
        state.obstacles.values.firstOrNull { it.position == coordinate }

    private fun clearGesture() {
        pressedObstacleId = null
        draggingObstacleId = null
        robotPressed = false
        draggingRobot = false
        longPressTriggered = false
        multiTouchOccurred = false
        parent?.requestDisallowInterceptTouchEvent(false)
    }

    private fun updateGeometry(width: Int, height: Int) {
        geometry.update(width, height, state.config, axisPadding, outerPadding)
    }

    /** Converts a raw touch/screen coordinate into the unscaled grid space [ArenaGeometry] expects. */
    private fun toContentX(x: Float): Float = (x - panX) / scale
    private fun toContentY(y: Float): Float = (y - panY) / scale

    private fun clampPan() {
        val minPanX = (width - width * scale).coerceAtMost(0f)
        val minPanY = (height - height * scale).coerceAtMost(0f)
        panX = panX.coerceIn(minPanX, 0f)
        panY = panY.coerceIn(minPanY, 0f)
    }

    fun zoomIn() = setScale(scale + ZOOM_STEP)

    fun zoomOut() = setScale(scale - ZOOM_STEP)

    private fun setScale(newScale: Float) {
        val centerX = width / 2f
        val centerY = height / 2f
        val contentX = toContentX(centerX)
        val contentY = toContentY(centerY)
        scale = newScale.coerceIn(MIN_SCALE, MAX_SCALE)
        panX = centerX - contentX * scale
        panY = centerY - contentY * scale
        clampPan()
        invalidate()
    }

    private fun buildContentDescription(state: ArenaState): String {
        val robot = state.robot?.let {
            "Robot at ${it.position.x}, ${it.position.y}, facing ${it.direction.name.lowercase()}."
        } ?: "Robot position unavailable."
        return "${state.config.columns} by ${state.config.rows} exploration arena. " +
            "${state.obstacles.size} obstacles. $robot"
    }

    private fun fillPaint(colorRes: Int): Paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, colorRes)
        style = Paint.Style.FILL
    }

    private fun strokePaint(colorRes: Int, width: Float): Paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, colorRes)
        style = Paint.Style.STROKE
        strokeWidth = width
    }

    private fun textPaint(
        colorRes: Int,
        sizeSp: Float,
        align: Paint.Align,
        bold: Boolean = false,
    ): Paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, colorRes)
        textSize = sp(sizeSp)
        textAlign = align
        typeface = if (bold) android.graphics.Typeface.DEFAULT_BOLD else android.graphics.Typeface.DEFAULT
    }

    private fun sp(value: Float): Float = TypedValue.applyDimension(
        TypedValue.COMPLEX_UNIT_SP,
        value,
        resources.displayMetrics,
    )

    companion object {
        private const val MIN_SCALE = 1f
        private const val MAX_SCALE = 4f
        private const val ZOOM_STEP = 0.5f
    }
}
