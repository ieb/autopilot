package uk.co.tfd.autopilot.watch.ble

object BleCommand {
    private const val CMD_SET_MODE: Byte = 0x01
    private const val CMD_ADJUST_TARGET: Byte = 0x02
    private const val CMD_SET_PILOT_TYPE: Byte = 0x03
    private const val CMD_STANDBY: Byte = 0x04

    fun setMode(mode: PilotMode, target: Float): ByteArray {
        val targetX10 = (target * 10).toInt().toShort()
        return byteArrayOf(
            CMD_SET_MODE,
            mode.value.toByte(),
            (targetX10.toInt() and 0xFF).toByte(),
            ((targetX10.toInt() shr 8) and 0xFF).toByte(),
        )
    }

    fun adjustTarget(delta: Float): ByteArray {
        val deltaX10 = (delta * 10).toInt().toShort()
        return byteArrayOf(
            CMD_ADJUST_TARGET,
            (deltaX10.toInt() and 0xFF).toByte(),
            ((deltaX10.toInt() shr 8) and 0xFF).toByte(),
            0x00,
        )
    }

    fun setPilotType(type: PilotType): ByteArray {
        return byteArrayOf(
            CMD_SET_PILOT_TYPE,
            type.value.toByte(),
            0x00,
            0x00,
        )
    }

    fun standby(): ByteArray {
        return byteArrayOf(CMD_STANDBY, 0x00, 0x00, 0x00)
    }
}
