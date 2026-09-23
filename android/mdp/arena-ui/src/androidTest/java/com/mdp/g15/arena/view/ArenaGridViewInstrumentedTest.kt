package com.mdp.g15.arena.view

import android.graphics.Bitmap
import android.graphics.Paint
import android.graphics.Canvas
import android.os.SystemClock
import android.view.MotionEvent
import android.view.ViewConfiguration
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.mdp.g15.arena.domain.ArenaState
import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.GridCoordinate
import com.mdp.g15.arena.domain.Obstacle
import com.mdp.g15.arena.domain.RobotPose
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import java.util.concurrent.atomic.AtomicReference

@RunWith(AndroidJUnit4::class)
class ArenaGridViewInstrumentedTest {
    @Test
    fun rendersChecklistStateWithoutThrowing() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val state = ArenaState(
            robot = RobotPose(GridCoordinate(7, 2), Direction.WEST),
            obstacles = mapOf(
                1 to Obstacle(1, GridCoordinate(10, 6), Direction.NORTH, "11"),
            ),
            selectedObstacleId = 1,
        )
        view.render(state, placementMode = false)
        view.measure(
            android.view.View.MeasureSpec.makeMeasureSpec(800, android.view.View.MeasureSpec.EXACTLY),
            android.view.View.MeasureSpec.makeMeasureSpec(800, android.view.View.MeasureSpec.EXACTLY),
        )
        view.layout(0, 0, 800, 800)
        val bitmap = Bitmap.createBitmap(800, 800, Bitmap.Config.ARGB_8888)

        view.draw(Canvas(bitmap))

        assertTrue(bitmap.getPixel(400, 400) != 0)
    }

    @Test
    fun placementTapReportsBottomLeftGridCoordinate() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val added = AtomicReference<GridCoordinate?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = added.set(position)
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) = Unit
            override fun onRemoveObstacle(obstacleId: Int) = Unit
            override fun onMoveRobot(destination: GridCoordinate) = Unit
        }

        onMain {
            prepare(view, ArenaState(), placementMode = true)
            val (x, y) = cellCenter(context, GridCoordinate(0, 0))
            dispatchTap(view, x, y)
        }

        assertTrue(added.get() == GridCoordinate(0, 0))
    }

    @Test
    fun longPressDragReportsFinalSnappedCoordinate() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val moved = AtomicReference<Pair<Int, GridCoordinate>?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = Unit
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) {
                moved.set(obstacleId to destination)
            }
            override fun onRemoveObstacle(obstacleId: Int) = Unit
            override fun onMoveRobot(destination: GridCoordinate) = Unit
        }
        val state = ArenaState(
            obstacles = mapOf(1 to Obstacle(1, GridCoordinate(1, 1))),
        )
        val start = cellCenter(context, GridCoordinate(1, 1))
        val destination = cellCenter(context, GridCoordinate(5, 6))
        val downTime = SystemClock.uptimeMillis()

        onMain {
            prepare(view, state, placementMode = false)
            dispatch(view, downTime, downTime, MotionEvent.ACTION_DOWN, start.first, start.second)
        }
        Thread.sleep(ViewConfiguration.getLongPressTimeout().toLong() + 100L)
        InstrumentationRegistry.getInstrumentation().waitForIdleSync()
        onMain {
            val eventTime = SystemClock.uptimeMillis()
            dispatch(view, downTime, eventTime, MotionEvent.ACTION_MOVE, destination.first, destination.second)
            assertTrue("Drag previews must not commit a move", moved.get() == null)
            dispatch(view, downTime, eventTime + 1L, MotionEvent.ACTION_UP, destination.first, destination.second)
        }

        assertTrue(moved.get() == (1 to GridCoordinate(5, 6)))
    }

    @Test
    fun robotDraggedOutsideOrCancelledDoesNotMoveOrDelete() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val state = ArenaState(robot = RobotPose(GridCoordinate(5, 5), Direction.NORTH))
        for (endAction in listOf(MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL)) {
            val view = createView(context)
            val events = mutableListOf<String>()
            view.interactionListener = object : ArenaInteractionListener {
                override fun onAddObstacle(position: GridCoordinate) { events.add("add") }
                override fun onSelectObstacle(obstacleId: Int?) { events.add("select") }
                override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) { events.add("move obstacle") }
                override fun onRemoveObstacle(obstacleId: Int) { events.add("remove") }
                override fun onMoveRobot(destination: GridCoordinate) { events.add("move robot") }
            }
            val start = cellCenter(context, GridCoordinate(5, 5))
            val time = SystemClock.uptimeMillis()
            onMain {
                prepare(view, state, false)
                dispatch(view, time, time, MotionEvent.ACTION_DOWN, start.first, start.second)
            }
            Thread.sleep(ViewConfiguration.getLongPressTimeout().toLong() + 100L)
            onMain {
                dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_MOVE, -100f, -100f)
                dispatch(view, time, SystemClock.uptimeMillis(), endAction, -100f, -100f)
                assertTrue(view.contentDescription.contains("Robot at 5, 5"))
            }
            assertTrue(events.isEmpty())
        }
    }

    @Test
    fun obstacleDragOutsideRemovesOnlyAfterRelease() = checkObstacleRemoval(zoomed = false)

    @Test
    fun zoomedObstacleDragOutsideViewportRemovesInsteadOfMovingToHiddenCell() = checkObstacleRemoval(zoomed = true)

    private fun checkObstacleRemoval(zoomed: Boolean) {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val removed = AtomicReference<Int?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = Unit
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) = Unit
            override fun onRemoveObstacle(obstacleId: Int) = removed.set(obstacleId)
            override fun onMoveRobot(destination: GridCoordinate) = Unit
        }
        val startCell = GridCoordinate(10, 10)
        val center = cellCenter(context, startCell)
        val zoom = if (zoomed) 1.5f else 1f
        val start = (center.first - VIEW_SIZE / 2f) * zoom + VIEW_SIZE / 2f to
            (center.second - VIEW_SIZE / 2f) * zoom + VIEW_SIZE / 2f
        val time = SystemClock.uptimeMillis()
        onMain {
            prepare(view, ArenaState(obstacles = mapOf(1 to Obstacle(1, startCell))), false)
            if (zoomed) view.zoomIn()
            dispatch(view, time, time, MotionEvent.ACTION_DOWN, start.first, start.second)
        }
        Thread.sleep(ViewConfiguration.getLongPressTimeout().toLong() + 100L)
        onMain {
            dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_MOVE, -50f, -50f)
            assertTrue(removed.get() == null)
            dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_UP, -50f, -50f)
        }
        assertTrue(removed.get() == 1)
    }

    @Test
    fun zoomedCanvasDoesNotDrawOutsideViewport() {
        val view = createView(ApplicationProvider.getApplicationContext())
        onMain {
            prepare(view, ArenaState(), false)
            repeat(6) { view.zoomIn() }
            val bitmap = Bitmap.createBitmap(1000, 1000, Bitmap.Config.ARGB_8888)
            bitmap.eraseColor(android.graphics.Color.MAGENTA)
            val canvas = Canvas(bitmap)
            canvas.translate(100f, 100f)
            view.draw(canvas)
            assertTrue(bitmap.getPixel(50, 400) == android.graphics.Color.MAGENTA)
            assertTrue(bitmap.getPixel(950, 400) == android.graphics.Color.MAGENTA)
            assertTrue(bitmap.getPixel(400, 50) == android.graphics.Color.MAGENTA)
            assertTrue(bitmap.getPixel(400, 950) == android.graphics.Color.MAGENTA)
            assertTrue(bitmap.getPixel(400, 400) != android.graphics.Color.MAGENTA)
        }
    }

    @Test
    fun viewportResizeReturnsToCenteredFit() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val added = AtomicReference<GridCoordinate?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = added.set(position)
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) = Unit
            override fun onRemoveObstacle(obstacleId: Int) = Unit
            override fun onMoveRobot(destination: GridCoordinate) = Unit
        }

        onMain {
            prepare(view, ArenaState(), placementMode = true)
            view.zoomIn()
            view.measure(
                android.view.View.MeasureSpec.makeMeasureSpec(1000, android.view.View.MeasureSpec.EXACTLY),
                android.view.View.MeasureSpec.makeMeasureSpec(800, android.view.View.MeasureSpec.EXACTLY),
            )
            view.layout(0, 0, 1000, 800)
            dispatchTap(view, 500f, 400f)
        }

        assertTrue(added.get() == GridCoordinate(10, 9))
    }

    @Test
    fun draggedObstacleKeepsItsTargetFaceEdge() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = createView(context)
        val state = ArenaState(obstacles = mapOf(1 to Obstacle(1, GridCoordinate(1, 1), Direction.NORTH, "11")))
        val start = cellCenter(context, GridCoordinate(1, 1))
        val destination = cellCenter(context, GridCoordinate(5, 6))
        val time = SystemClock.uptimeMillis()
        onMain {
            prepare(view, state, false)
            dispatch(view, time, time, MotionEvent.ACTION_DOWN, start.first, start.second)
        }
        Thread.sleep(ViewConfiguration.getLongPressTimeout().toLong() + 100L)
        onMain {
            dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_MOVE, destination.first, destination.second)
            val bitmap = Bitmap.createBitmap(VIEW_SIZE, VIEW_SIZE, Bitmap.Config.ARGB_8888)
            view.draw(Canvas(bitmap))
            val density = context.resources.displayMetrics.density
            val cellSize = (VIEW_SIZE - 38f * density) / 20f
            val faceY = (destination.second - cellSize * 0.42f).toInt()
            val faceColor = androidx.core.content.ContextCompat.getColor(context, com.mdp.g15.arena.R.color.arena_target_face)
            assertTrue(bitmap.getPixel(destination.first.toInt(), faceY) == faceColor)
            dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_CANCEL, destination.first, destination.second)
        }
    }

    @Test
    fun gridAndDragLabelsDistinguishUnknownTargetsFromSelectedDirections() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        // Include unset/all directions, alphabetic targets and a target identical to its ID.
        for (face in listOf(null) + Direction.entries) {
            for (target in listOf(null, "A", "11")) {
                val view = createView(context)
                val obstacle = Obstacle(11, GridCoordinate(1, 1), face, target)
                val state = ArenaState(obstacles = mapOf(11 to obstacle))
                val start = cellCenter(context, obstacle.position)
                val destination = cellCenter(context, GridCoordinate(5, 6))
                val time = SystemClock.uptimeMillis()
                val bitmap = Bitmap.createBitmap(VIEW_SIZE, VIEW_SIZE, Bitmap.Config.ARGB_8888)
                val labels = mutableListOf<String>()
                val canvas = object : Canvas(bitmap) {
                    override fun drawText(text: String, x: Float, y: Float, paint: Paint) {
                        labels.add(text)
                        super.drawText(text, x, y, paint)
                    }
                }
                onMain {
                    prepare(view, state, false)
                    view.draw(canvas)
                    assertEquals("Grid label for face=$face target=$target", target ?: "11", labels.last())
                    dispatch(view, time, time, MotionEvent.ACTION_DOWN, start.first, start.second)
                }
                Thread.sleep(ViewConfiguration.getLongPressTimeout().toLong() + 100L)
                onMain {
                    dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_MOVE, destination.first, destination.second)
                    labels.clear()
                    view.draw(canvas)
                    assertEquals("Drag label for face=$face target=$target", target ?: "?", labels.last())
                    dispatch(view, time, SystemClock.uptimeMillis(), MotionEvent.ACTION_CANCEL, destination.first, destination.second)
                    labels.clear()
                    view.draw(canvas)
                    assertEquals("Cancel restores grid label", target ?: "11", labels.last())
                }
                bitmap.recycle()
            }
        }
    }

    private fun createView(context: android.content.Context): ArenaGridView {
        val result = AtomicReference<ArenaGridView>()
        onMain { result.set(ArenaGridView(context)) }
        return result.get()
    }

    private fun prepare(view: ArenaGridView, state: ArenaState, placementMode: Boolean) {
        view.render(state, placementMode)
        view.measure(
            android.view.View.MeasureSpec.makeMeasureSpec(VIEW_SIZE, android.view.View.MeasureSpec.EXACTLY),
            android.view.View.MeasureSpec.makeMeasureSpec(VIEW_SIZE, android.view.View.MeasureSpec.EXACTLY),
        )
        view.layout(0, 0, VIEW_SIZE, VIEW_SIZE)
    }

    private fun dispatchTap(view: ArenaGridView, x: Float, y: Float) {
        val downTime = SystemClock.uptimeMillis()
        dispatch(view, downTime, downTime, MotionEvent.ACTION_DOWN, x, y)
        dispatch(view, downTime, downTime + 1L, MotionEvent.ACTION_UP, x, y)
    }

    private fun dispatch(
        view: ArenaGridView,
        downTime: Long,
        eventTime: Long,
        action: Int,
        x: Float,
        y: Float,
    ) {
        MotionEvent.obtain(downTime, eventTime, action, x, y, 0).also {
            view.dispatchTouchEvent(it)
            it.recycle()
        }
    }

    private fun cellCenter(
        context: android.content.Context,
        coordinate: GridCoordinate,
    ): Pair<Float, Float> {
        val density = context.resources.displayMetrics.density
        val axisPadding = 30f * density
        val cellSize = (VIEW_SIZE - axisPadding * 2f) / 20f
        val x = axisPadding + (coordinate.x + 0.5f) * cellSize
        val displayRow = 19 - coordinate.y
        val y = axisPadding + (displayRow + 0.5f) * cellSize
        return x to y
    }

    private fun onMain(block: () -> Unit) {
        InstrumentationRegistry.getInstrumentation().runOnMainSync(block)
    }

    private companion object {
        const val VIEW_SIZE = 800
    }
}
