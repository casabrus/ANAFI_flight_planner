package com.fotogrammetria.anafiplanner.planner

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

data class Point2D(
    val x: Double,
    val y: Double,
) {
    fun distanceTo(other: Point2D): Double {
        return hypot(other.x - x, other.y - y)
    }

    fun rotate(angleRad: Double): Point2D {
        val cosAngle = cos(angleRad)
        val sinAngle = sin(angleRad)
        return Point2D(
            x = (x * cosAngle) - (y * sinAngle),
            y = (x * sinAngle) + (y * cosAngle),
        )
    }
}
