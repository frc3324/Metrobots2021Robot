package frc.team3324.robot.util

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer

enum class Color(val RGB: Array<Int>) {
    RED(arrayOf(255, 0, 0)), ORANGE(arrayOf(255, 165, 0)), YELLOW(arrayOf(0, 255, 255)), GREEN(arrayOf(0, 255, 0)), BLUE(arrayOf(0, 0, 255)), PURPLE(arrayOf(255, 0, 255))
}

class LED {
    private val led = AddressableLED(9)
    private val ledBuffer = AddressableLEDBuffer(60)

    init {
        led.setLength(ledBuffer.length)
        led.setData(ledBuffer)
        led.start()
    }

    fun setColor(color: Color) {
        val RGB = when(color) {
            Color.RED -> Color.RED.RGB
            Color.ORANGE -> Color.ORANGE.RGB
            Color.YELLOW -> Color.YELLOW.RGB
            Color.GREEN -> Color.GREEN.RGB
            Color.BLUE -> Color.BLUE.RGB
            Color.PURPLE -> Color.PURPLE.RGB
        }

        for(i in 0 until ledBuffer.length) {
            ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2])
        }
        led.setData(ledBuffer)
    }
}