package uk.co.tfd.autopilot.watch.ble

enum class PilotMode(val value: Int) {
    STANDBY(0), COMPASS(1), WIND_AWA(2), WIND_TWA(3), VMG_UP(4), VMG_DOWN(5);

    companion object {
        fun fromValue(v: Int) = entries.firstOrNull { it.value == v } ?: STANDBY
    }

    val displayName: String
        get() = when (this) {
            STANDBY -> "STANDBY"
            COMPASS -> "COMPASS"
            WIND_AWA -> "WIND AWA"
            WIND_TWA -> "WIND TWA"
            VMG_UP -> "VMG UP"
            VMG_DOWN -> "VMG DN"
        }
}

enum class PilotType(val value: Int) {
    PD(0), PID(1), SMOOTH(2), ADAPTIVE(3);

    companion object {
        fun fromValue(v: Int) = entries.firstOrNull { it.value == v } ?: PD
    }

    val displayName: String
        get() = when (this) {
            PD -> "PD"
            PID -> "PID"
            SMOOTH -> "Smooth"
            ADAPTIVE -> "Adaptive"
        }

    fun next(): PilotType = entries[(ordinal + 1) % entries.size]
}

data class AutopilotState(
    val pilotMode: PilotMode = PilotMode.STANDBY,
    val pilotType: PilotType = PilotType.PD,
    val targetValue: Float = 0f,
    val faultCode: Int = 0,
    val clutchEngaged: Boolean = false,
) {
    companion object {
        fun fromBytes(data: ByteArray): AutopilotState? {
            if (data.size < 6) return null
            val mode = PilotMode.fromValue(data[0].toInt() and 0xFF)
            val type = PilotType.fromValue(data[1].toInt() and 0xFF)
            val targetRaw = (data[2].toInt() and 0xFF) or ((data[3].toInt() and 0xFF) shl 8)
            val targetSigned = if (targetRaw > 32767) targetRaw - 65536 else targetRaw
            val target = targetSigned / 10f
            val fault = data[4].toInt() and 0xFF
            val flags = data[5].toInt() and 0xFF
            return AutopilotState(
                pilotMode = mode,
                pilotType = type,
                targetValue = target,
                faultCode = fault,
                clutchEngaged = (flags and 0x01) != 0,
            )
        }
    }
}

enum class ConnectionState {
    DISCONNECTED, SCANNING, CONNECTING, CONNECTED
}
