package com.mdp.g15.arena.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.os.SystemClock
import android.os.Handler
import android.os.Looper
import android.util.AttributeSet
import android.util.TypedValue
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import android.view.ViewConfiguration
import androidx.core.content.ContextCompat
import androidx.core.content.res.ResourcesCompat
import com.mdp.g15.arena.R
import com.mdp.g15.arena.domain.ArenaReducer
import com.mdp.g15.arena.domain.ArenaAction
import com.mdp.g15.arena.domain.ArenaReduction
import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.Obstacle
import com.mdp.g15.arena.domain.PathOverlay
import com.mdp.g15.arena.domain.PoseSource
import com.mdp.g15.arena.domain.RobotFootprint
import com.mdp.g15.arena.domain.RobotPose
import com.mdp.g15.arena.domain.readableHeading
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin

class ArenaGridView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {
    var interactionListener: ArenaInteractionListener? = null

    private var state = ArenaState()
    private var placementMode = false
    private var pathOverlay: PathOverlay? = null
    private val robotMotion = RobotMotion()
    private val geometry = ArenaGeometry()
    private val density = resources.displayMetrics.density
    private val axisPadding = 36f * density
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
                if (scale == MIN_SCALE) resetToFit() else clampPan()
                invalidate()
                return true
            }
        },
    )

    private val backgroundPaint = fillPaint(R.color.arena_background)
    private val gridPaint = strokePaint(R.color.arena_grid_line, 1f * density)
    // Every fifth line is brighter, like a blueprint's major grid.
    private val gridMajorPaint = strokePaint(R.color.arena_grid_major, 1.5f * density)
    private val framePaint = strokePaint(R.color.arena_frame, 3f * density)
    private val webPaint = strokePaint(R.color.arena_web, 1.5f * density)
    private val inkPaint = strokePaint(R.color.arena_ink, 2.5f * density).apply {
        strokeJoin = Paint.Join.ROUND
    }
    private val axisTextPaint = textPaint(R.color.arena_axis_text, 14f, Paint.Align.CENTER).apply {
        typeface = ResourcesCompat.getFont(context, R.font.press_start_2p)
        isAntiAlias = false
    }
    private val obstaclePaint = fillPaint(R.color.arena_obstacle)
    private val obstacleFoundPaint = fillPaint(R.color.arena_obstacle_found)
    private val obstacleTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.arena_ink)
        textSize = sp(12f)
        textAlign = Paint.Align.CENTER
        typeface = android.graphics.Typeface.DEFAULT_BOLD
    }
    private val selectedPaint = strokePaint(R.color.arena_selected, 4f * density)
    private val facePaint = strokePaint(R.color.arena_target_face, 4f * density)
    private val robotPaint = fillPaint(R.color.arena_robot)
    private val robotDirectionPaint = fillPaint(R.color.arena_robot_direction)
    private val robotEyePaint = fillPaint(R.color.arena_robot_eye)
    private val pathCellPaint = fillPaint(R.color.arena_path_cell)
    private val pathArrowPaint = strokePaint(R.color.arena_path_arrow, 3f * density).apply {
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val pathArrowHeadPaint = fillPaint(R.color.arena_path_arrow)
    private val dragPaint = Paint(obstaclePaint).apply { alpha = 165 }
    private val dragInvalidPaint = Paint().apply {
        color = ContextCompat.getColor(context, R.color.arena_feedback_error)
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
    /** Offset, in cells, of the pressed point from the robot's bottom-left reference. */
    private var robotGrabOffsetX = 0.0
    private var robotGrabOffsetY = 0.0
    private var longPressTriggered = false
    private var multiTouchOccurred = false
    private var tapMoved = false

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

    fun render(state: ArenaState, placementMode: Boolean, pathOverlay: PathOverlay? = null) {
        if ((robotPressed || draggingRobot) && this.state.reportRevision != state.reportRevision) {
            handler.removeCallbacks(longPressRunnable)
            clearGesture()
            multiTouchOccurred = true // Consume the release without committing the old drag.
        }
        if (draggingRobot || this.state.config != state.config || this.state.obstacles != state.obstacles ||
            state.robot == null || (this.state.reportedRobot == null && state.reportedRobot != null) ||
            this.state.robot?.source == PoseSource.OPERATOR ||
            // The path demo already supplies a pose every frame; smoothing would lag it.
            state.robot?.source == PoseSource.SIMULATED) {
            robotMotion.snap(state.robot)
        } else if (this.state.robot != state.robot) {
            robotMotion.retarget(state, SystemClock.uptimeMillis())
        }
        this.state = state
        this.placementMode = placementMode
        this.pathOverlay = pathOverlay
        contentDescription = buildContentDescription(state)
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
        resetToFit()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        updateGeometry(width, height)
        val saveCount = canvas.save()
        canvas.clipRect(0f, 0f, width.toFloat(), height.toFloat())
        canvas.translate(panX, panY)
        canvas.scale(scale, scale)
        drawArena(canvas)
        pathOverlay?.let { drawPathOverlay(canvas, it) }
        drawObstacles(canvas)
        if (!draggingRobot) {
            drawAnimatedRobot(canvas)
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
                tapMoved = false
                val coordinate = coordinateAtTouch(event.x, event.y)
                val robot = state.robot
                val touchPoint = pointAtTouch(event.x, event.y)
                val visual = robotMotion.sample(SystemClock.uptimeMillis())
                val hitPose = visual?.let { RobotPose(it.x.toDouble(), it.y.toDouble(), it.angle.toDouble()) } ?: robot
                val robotHere = touchPoint != null && hitPose != null &&
                    RobotFootprint.contains(hitPose, state.config, touchPoint)
                pressedObstacleId = if (placementMode || robotHere) null else coordinate?.let(::obstacleAt)?.id
                robotPressed = robotHere && !placementMode
                if (robotHere) {
                    robotGrabOffsetX = touchPoint!!.x - hitPose!!.x
                    robotGrabOffsetY = touchPoint.y - hitPose.y
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
                    tapMoved = true
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
                        val destination = coordinateAtTouch(event.x, event.y)
                        if (destination == null) {
                            interactionListener?.onRemoveObstacle(draggedId)
                            announceForAccessibility("Obstacle $draggedId removed")
                        } else {
                            interactionListener?.onMoveObstacle(draggedId, destination)
                        }
                    }
                    draggingRobot -> {
                        val destination = robotDropAt(event.x, event.y)
                        if (destination != null && ArenaReducer().canMoveRobot(state, destination)) {
                            interactionListener?.onMoveRobot(destination)
                        }
                    }
                    multiTouchOccurred || tapMoved || hypot(event.x - downX, event.y - downY) > touchSlop -> Unit
                    else -> {
                        performClick()
                        val coordinate = coordinateAtTouch(event.x, event.y)
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
        robotMotion.snap(state.robot)
        clearGesture()
        super.onDetachedFromWindow()
    }

    private fun drawAnimatedRobot(canvas: Canvas) {
        state.robot ?: return
        val now = SystemClock.uptimeMillis()
        val visual = robotMotion.sample(now) ?: return
        val checkpoint = canvas.save()
        canvas.clipRect(geometry.arenaLeft, geometry.arenaTop,
            geometry.arenaLeft + geometry.arenaWidth, geometry.arenaTop + geometry.arenaHeight)
        drawRobot(canvas, RobotPose(visual.x.toDouble(), visual.y.toDouble(), visual.angle.toDouble()))
        canvas.restoreToCount(checkpoint)
        if (robotMotion.isRunning(now)) postInvalidateOnAnimation()
    }

    private fun drawArena(canvas: Canvas) {
        val left = geometry.arenaLeft
        val top = geometry.arenaTop
        val arenaWidth = geometry.arenaWidth
        val arenaHeight = geometry.arenaHeight
        canvas.drawRect(left, top, left + arenaWidth, top + arenaHeight, backgroundPaint)
        drawWebRadar(canvas)

        for (column in 0..state.config.columns) {
            val x = left + column * geometry.cellWidth
            canvas.drawLine(x, top, x, top + arenaHeight, if (column % 5 == 0) gridMajorPaint else gridPaint)
        }
        // Rows count from the bottom, so major lines follow arena y, not screen y.
        for (row in 0..state.config.rows) {
            val y = top + row * geometry.cellHeight
            val arenaRow = state.config.rows - row
            canvas.drawLine(left, y, left + arenaWidth, y, if (arenaRow % 5 == 0) gridMajorPaint else gridPaint)
        }
        canvas.drawRect(left, top, left + arenaWidth, top + arenaHeight, framePaint)

        axisTextPaint.textSize = min(sp(13f), geometry.cellSize * 0.65f)
        axisTextPaint.textScaleX = 1f
        // Press Start 2P uses full-width glyphs. Size for a two-digit label so adjacent
        // coordinates and the left axis remain distinct at the tablet's native resolution.
        val twoDigitWidth = axisTextPaint.measureText("19")
        val axisLabelWidth = min(geometry.cellWidth * 0.86f,
            geometry.arenaLeft - 4f * density)
        if (twoDigitWidth > axisLabelWidth && axisLabelWidth > 0f) {
            axisTextPaint.textScaleX = axisLabelWidth / twoDigitWidth
        }
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
        val bottomAxisScale = axisTextPaint.textScaleX
        axisTextPaint.textScaleX = bottomAxisScale * 0.85f
        for (row in 0 until state.config.rows) {
            val bounds = geometry.cellBounds(GridCoordinate(0, row))
            canvas.drawText(
                row.toString(),
                geometry.arenaLeft - 1.5f * density,
                bounds.centerY - (axisTextPaint.ascent() + axisTextPaint.descent()) / 2f,
                axisTextPaint,
            )
        }
        axisTextPaint.textScaleX = bottomAxisScale
        axisTextPaint.textAlign = Paint.Align.CENTER
    }

    /** Decoration only: a faint spider-web radar under the grid, in the far corner from the start. */
    private fun drawWebRadar(canvas: Canvas) {
        val radius = geometry.cellSize * 4.5f
        val cx = geometry.arenaLeft + geometry.arenaWidth - radius - geometry.cellSize * 0.5f
        val cy = geometry.arenaTop + radius + geometry.cellSize * 0.5f
        val spokes = 12
        for (ring in 1..4) {
            val r = radius * ring / 4f
            val web = Path()
            for (i in 0..spokes) {
                val a = (2.0 * Math.PI * i / spokes).toFloat()
                val x = cx + r * cos(a)
                val y = cy + r * sin(a)
                if (i == 0) web.moveTo(x, y) else web.lineTo(x, y)
            }
            canvas.drawPath(web, webPaint)
        }
        for (i in 0 until spokes) {
            val a = (2.0 * Math.PI * i / spokes).toFloat()
            canvas.drawLine(cx, cy, cx + radius * cos(a), cy + radius * sin(a), webPaint)
        }
    }

    /** Active demo move: swept cells, then the centre-path arrow pointing the way of travel. */
    private fun drawPathOverlay(canvas: Canvas, overlay: PathOverlay) {
        overlay.cells.forEach { cell ->
            if (!state.config.contains(cell)) return@forEach
            val bounds = geometry.cellBounds(cell)
            canvas.drawRect(bounds.left, bounds.top, bounds.right, bounds.bottom, pathCellPaint)
        }
        val points = overlay.centreline
        if (points.size < 2) return
        val checkpoint = canvas.save()
        canvas.clipRect(geometry.arenaLeft, geometry.arenaTop,
            geometry.arenaLeft + geometry.arenaWidth, geometry.arenaTop + geometry.arenaHeight)
        val tipX = geometry.contentX(points.last().x)
        val tipY = geometry.contentY(points.last().y)
        // Aim along the last stretch that is long enough to have a stable direction.
        val tail = points.asReversed().firstOrNull {
            hypot(geometry.contentX(it.x) - tipX, geometry.contentY(it.y) - tipY) > geometry.cellSize * 0.2f
        } ?: points.first()
        val angle = atan2(tipY - geometry.contentY(tail.y), tipX - geometry.contentX(tail.x))
        val headLength = geometry.cellSize * 0.45f
        val headHalfWidth = geometry.cellSize * 0.28f
        // Stop the shaft at the head's base so the round cap does not poke through the tip.
        val baseX = tipX - headLength * cos(angle)
        val baseY = tipY - headLength * sin(angle)
        val shaft = Path().apply {
            moveTo(geometry.contentX(points.first().x), geometry.contentY(points.first().y))
            points.drop(1).forEach { point ->
                val x = geometry.contentX(point.x)
                val y = geometry.contentY(point.y)
                if (hypot(x - tipX, y - tipY) < headLength) return@forEach
                lineTo(x, y)
            }
            lineTo(baseX, baseY)
        }
        canvas.drawPath(shaft, pathArrowPaint)
        val head = Path().apply {
            moveTo(tipX, tipY)
            lineTo(baseX - headHalfWidth * sin(angle), baseY + headHalfWidth * cos(angle))
            lineTo(baseX + headHalfWidth * sin(angle), baseY - headHalfWidth * cos(angle))
            close()
        }
        canvas.drawPath(head, pathArrowHeadPaint)
        canvas.restoreToCount(checkpoint)
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
        val corner = geometry.cellSize * 0.22f
        canvas.drawRoundRect(rect, corner, corner, if (obstacle.targetId == null) obstaclePaint else obstacleFoundPaint)
        canvas.drawRoundRect(rect, corner, corner, inkPaint)

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

        if (state.selectedObstacleId == obstacle.id) {
            val ring = geometry.cellSize * 0.1f
            canvas.drawRoundRect(RectF(rect.left - ring, rect.top - ring, rect.right + ring, rect.bottom + ring),
                corner + ring, corner + ring, selectedPaint)
        }
        obstacle.targetFace?.let { drawFace(canvas, rect, it) }
    }

    private fun drawFace(canvas: Canvas, rect: RectF, direction: Direction) {
        when (direction) {
            Direction.NORTH -> canvas.drawLine(rect.left, rect.top, rect.right, rect.top, facePaint)
            Direction.EAST -> canvas.drawLine(rect.right, rect.top, rect.right, rect.bottom, facePaint)
            Direction.SOUTH -> canvas.drawLine(rect.left, rect.bottom, rect.right, rect.bottom, facePaint)
            Direction.WEST -> canvas.drawLine(rect.left, rect.top, rect.left, rect.bottom, facePaint)
        }
    }

    private fun drawRobot(canvas: Canvas, pose: RobotPose) {
        val bounds = geometry.robotBounds(pose, state.config)
        val checkpoint = canvas.save()
        canvas.rotate(pose.heading.toFloat(), bounds.centerX, bounds.centerY)
        val cell = geometry.cellSize
        val inset = cell * 0.08f
        val body = RectF(bounds.left + inset, bounds.top + inset, bounds.right - inset, bounds.bottom - inset)
        canvas.drawRoundRect(body, cell * 0.35f, cell * 0.35f, robotPaint)
        canvas.drawRoundRect(body, cell * 0.35f, cell * 0.35f, inkPaint)
        // The front is the top of the unrotated body; canvas.rotate turns it to the heading.
        val nose = Path().apply {
            moveTo(bounds.centerX, bounds.top + inset * 1.5f)
            lineTo(bounds.centerX - cell * 0.28f, bounds.top + cell * 0.42f)
            lineTo(bounds.centerX + cell * 0.28f, bounds.top + cell * 0.42f)
            close()
        }
        canvas.drawPath(nose, robotDirectionPaint)
        val eyeTop = bounds.top + cell * 0.55f
        val eyeWidth = (bounds.right - bounds.left) * 0.3f
        listOf(-1f, 1f).forEach { side ->
            val eyeCentre = bounds.centerX + side * eyeWidth * 0.62f
            val eye = Path().apply {
                // A slanted Spider-Man lens: pointed toward the middle, round at the outside.
                moveTo(eyeCentre - side * eyeWidth * 0.5f, eyeTop + cell * 0.05f)
                quadTo(eyeCentre + side * eyeWidth * 0.55f, eyeTop - cell * 0.05f,
                    eyeCentre + side * eyeWidth * 0.45f, eyeTop + cell * 0.32f)
                quadTo(eyeCentre, eyeTop + cell * 0.48f, eyeCentre - side * eyeWidth * 0.5f, eyeTop + cell * 0.05f)
                close()
            }
            canvas.drawPath(eye, robotEyePaint)
            canvas.drawPath(eye, inkPaint)
        }
        canvas.restoreToCount(checkpoint)
    }

    private fun drawRobotDragImage(canvas: Canvas, pose: RobotPose) {
        val checkpoint = canvas.save()
        canvas.clipRect(geometry.arenaLeft, geometry.arenaTop,
            geometry.arenaLeft + geometry.arenaWidth, geometry.arenaTop + geometry.arenaHeight)
        drawRobot(canvas, pose)
        canvas.restoreToCount(checkpoint)
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
        val destination = coordinateAtTouch(dragX, dragY)
        if (destination == null) {
            drawInvalidDropIndicator(canvas, contentX, contentY)
            return
        }
        if (ArenaReducer().reduce(state, ArenaAction.MoveObstacle(obstacleId, destination)) is ArenaReduction.Failure) {
            state.obstacles[obstacleId]?.let { drawObstacle(canvas, it) }
            drawInvalidDropIndicator(canvas, contentX, contentY)
            return
        }
        val bounds = geometry.cellBounds(destination)
        val inset = geometry.cellSize * 0.08f
        val rect = RectF(bounds.left + inset, bounds.top + inset, bounds.right - inset, bounds.bottom - inset)
        canvas.drawRect(rect, dragPaint)
        val obstacle = state.obstacles[obstacleId] ?: return
        obstacleTextPaint.textSize = geometry.cellSize * 0.48f
        obstacle.targetFace?.let { drawFace(canvas, rect, it) }
        canvas.drawText(
            obstacle.targetId ?: "?",
            rect.centerX(),
            rect.centerY() - (obstacleTextPaint.ascent() + obstacleTextPaint.descent()) / 2f,
            obstacleTextPaint,
        )
    }

    private fun drawRobotDragPreview(canvas: Canvas) {
        val robot = state.robot ?: return
        val contentX = toContentX(dragX)
        val contentY = toContentY(dragY)
        val destination = robotDropAt(dragX, dragY)
        if (destination == null || !ArenaReducer().canMoveRobot(state, destination)) {
            drawRobotDragImage(canvas, robot)
            drawInvalidDropIndicator(canvas, contentX, contentY)
            return
        }
        drawRobotDragImage(canvas, robot.copy(x = destination.x.toDouble(), y = destination.y.toDouble()))
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
        geometry.update(width, height, state.config, axisPadding, outerPadding, stretchToFill = true)
    }

    /** Converts a raw touch/screen coordinate into the unscaled grid space [ArenaGeometry] expects. */
    private fun toContentX(x: Float): Float = (x - panX) / scale
    private fun toContentY(y: Float): Float = (y - panY) / scale

    // The visible card is the interaction boundary even when zoom exposes only
    // part of the underlying map. Dropping outside must not select a hidden cell.
    private fun coordinateAtTouch(x: Float, y: Float): GridCoordinate? =
        if (x < 0f || y < 0f || x >= width || y >= height) null
        else geometry.coordinateAt(toContentX(x), toContentY(y))

    private fun pointAtTouch(x: Float, y: Float): RobotFootprint.Point? {
        if (coordinateAtTouch(x, y) == null) return null
        return RobotFootprint.Point(
            ((toContentX(x) - geometry.arenaLeft) / geometry.cellWidth).toDouble(),
            (state.config.rows - (toContentY(y) - geometry.arenaTop) / geometry.cellHeight).toDouble(),
        )
    }

    private fun robotDropAt(x: Float, y: Float): GridCoordinate? = pointAtTouch(x, y)?.let {
        GridCoordinate(kotlin.math.round(it.x - robotGrabOffsetX).toInt(),
            kotlin.math.round(it.y - robotGrabOffsetY).toInt())
    }

    private fun clampPan() {
        val minPanX = (width - width * scale).coerceAtMost(0f)
        val minPanY = (height - height * scale).coerceAtMost(0f)
        panX = panX.coerceIn(minPanX, 0f)
        panY = panY.coerceIn(minPanY, 0f)
    }

    /** Restore the full-map overview for hands-off run/result presentation. */
    fun fitToView() {
        resetToFit()
        invalidate()
    }

    fun zoomIn() = setScale(scale + ZOOM_STEP)

    fun zoomOut() = setScale(scale - ZOOM_STEP)

    private fun setScale(newScale: Float) {
        val centerX = width / 2f
        val centerY = height / 2f
        val contentX = toContentX(centerX)
        val contentY = toContentY(centerY)
        scale = newScale.coerceIn(MIN_SCALE, MAX_SCALE)
        if (scale == MIN_SCALE) {
            resetToFit()
            invalidate()
            return
        }
        panX = centerX - contentX * scale
        panY = centerY - contentY * scale
        clampPan()
        invalidate()
    }

    private fun resetToFit() {
        scale = MIN_SCALE
        panX = 0f
        panY = 0f
    }

    private fun buildContentDescription(state: ArenaState): String {
        val robot = state.robot?.let {
            "Robot at ${it.x}, ${it.y}, facing ${it.readableHeading()}. " +
                when (it.source) {
                    PoseSource.REPORTED -> "RPi reported."
                    PoseSource.OPERATOR -> "Operator placed."
                    PoseSource.ESTIMATED -> "Estimated movement."
                    PoseSource.SIMULATED -> "Path demo simulation."
                }
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
