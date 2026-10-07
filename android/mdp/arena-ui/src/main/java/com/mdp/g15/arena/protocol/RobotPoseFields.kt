package com.mdp.g15.arena.protocol

import com.mdp.g15.arena.domain.Direction
import com.mdp.g15.arena.domain.RobotPose

/** Shared ROBOT and STATUS,START pose fields. Coordinates are bottom-left cell units. */
object RobotPoseFields {
    fun parse(x: String, y: String, bearing: String): RobotPose? {
        // Still permit diagnostic off-map reports, while keeping Canvas/Float conversions finite.
        val px = x.toDoubleOrNull()?.takeIf { it.isFinite() && kotlin.math.abs(it) <= 1000.0 } ?: return null
        val py = y.toDoubleOrNull()?.takeIf { it.isFinite() && kotlin.math.abs(it) <= 1000.0 } ?: return null
        val angle = Direction.fromWire(bearing)?.ordinal?.times(90.0)
            ?: bearing.toDoubleOrNull()?.takeIf(Double::isFinite) ?: return null
        return RobotPose(px, py, ((angle % 360.0) + 360.0) % 360.0)
    }
}
