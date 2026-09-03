package com.mdp.g15.arena.view

import com.mdp.g15.arena.domain.GridCoordinate

interface ArenaInteractionListener {
    fun onAddObstacle(position: GridCoordinate)
    fun onSelectObstacle(obstacleId: Int?)
    fun onMoveObstacle(obstacleId: Int, destination: GridCoordinate)
    fun onRemoveObstacle(obstacleId: Int)
    fun onMoveRobot(destination: GridCoordinate)
}
