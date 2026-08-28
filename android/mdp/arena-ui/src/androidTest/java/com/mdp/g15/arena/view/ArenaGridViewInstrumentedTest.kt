package com.mdp.g15.arena.view

import android.graphics.Bitmap
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
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import java.util.concurrent.atomic.AtomicReference

@RunWith(AndroidJUnit4::class)
class ArenaGridViewInstrumentedTest {
    @Test
    fun rendersChecklistStateWithoutThrowing() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val view = ArenaGridView(context)
        val state = ArenaState(
            robot = RobotPose(GridCoordinate(7, 2), Direction.WEST),
            obstacles = mapOf(
                1 to Obstacle(1, GridCoordinate(10, 6), Direction.NORTH, "11"),
            ),
            selectedObstacleId = 1,
            nextObstacleId = 2,
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
        val view = ArenaGridView(context)
        val added = AtomicReference<GridCoordinate?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = added.set(position)
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) = Unit
            override fun onRemoveObstacle(obstacleId: Int) = Unit
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
        val view = ArenaGridView(context)
        val moved = AtomicReference<Pair<Int, GridCoordinate>?>()
        view.interactionListener = object : ArenaInteractionListener {
            override fun onAddObstacle(position: GridCoordinate) = Unit
            override fun onSelectObstacle(obstacleId: Int?) = Unit
            override fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate) {
                moved.set(obstacleId to destination)
            }
            override fun onRemoveObstacle(obstacleId: Int) = Unit
        }
        val state = ArenaState(
            obstacles = mapOf(1 to Obstacle(1, GridCoordinate(1, 1))),
            nextObstacleId = 2,
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
            dispatch(view, downTime, eventTime + 1L, MotionEvent.ACTION_UP, destination.first, destination.second)
        }

        assertTrue(moved.get() == (1 to GridCoordinate(5, 6)))
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
        val outerPadding = 8f * density
        val cellSize = (VIEW_SIZE - axisPadding - outerPadding) / 20f
        val x = axisPadding + (coordinate.x + 0.5f) * cellSize
        val displayRow = 19 - coordinate.y
        val y = outerPadding + (displayRow + 0.5f) * cellSize
        return x to y
    }

    private fun onMain(block: () -> Unit) {
        InstrumentationRegistry.getInstrumentation().runOnMainSync(block)
    }

    private companion object {
        const val VIEW_SIZE = 800
    }
}
