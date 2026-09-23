package com.mdp.g15.arena.domain

/** The same target label rule applies to wire messages, domain actions and restored state. */
object TargetId {
    private val valid = Regex("[A-Z0-9]{1,2}")

    fun error(value: String): String? = when {
        value.isBlank() -> "TARGET ID cannot be blank."
        !valid.matches(value) -> "TARGET ID must be 1-2 alphanumeric, uppercase-only characters."
        else -> null
    }
}
