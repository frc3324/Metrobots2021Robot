package frc.team3324.robot.intake

import edu.wpi.first.wpilibj.DigitalInput
import frc.team3324.library.motorcontrollers.MetroSparkMAX
import frc.team3324.library.subsystems.MotorSubsystem
import frc.team3324.robot.util.Consts
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

class Pivot: MotorSubsystem(listOf(Consts.Pivot.MOTOR)), Loggable {
    private val pivotEncoder = (getMotor(0) as MetroSparkMAX).getEncoder()
    val upperLimitSwitch = Consts.Pivot.UPPER_LIMIT_SWITCH
    val lowerLimitSwitch = Consts.Pivot.LOWER_LIMIT_SWITCH

    val encoderPosition: Double
        @Log
        get() = pivotEncoder.position

    val encoderVelocity: Double
        @Log
        get() = pivotEncoder.velocity

    override fun periodic() {
    }
}