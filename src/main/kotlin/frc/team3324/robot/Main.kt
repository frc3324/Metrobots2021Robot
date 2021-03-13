package frc.team3324.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.team3324.robot.util.RearCamera

/**
 * Main initialization function. Do not perform any initialization here.
 *
 * <p>If you change your main robot class, change the parameter type.
 */
    fun main() {
       // RobotBase.startRobot(::Robot)
        val rearCamera = RearCamera()
        rearCamera.rearCamera()
    }

