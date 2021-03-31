package frc.team3324.robot.shooter.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team3324.robot.Robot
import frc.team3324.robot.shooter.Shooter
import frc.team3324.robot.util.FrontCamera
import kotlin.math.pow

class RunShooter(val shooter: Shooter, private val frontCamera: FrontCamera, val trench: Boolean) : CommandBase() {

    init {
        Robot.light.set(true)
        addRequirements(shooter)
    }

    override fun execute() {
        val area = frontCamera.contourArea()
        var rpm = 4845 + -1.4*area + 0.000558*area.pow(2) - 100
        val dashRPM = SmartDashboard.getNumber("Input RPM", 0.0)

        shooter.RPM = rpm

        SmartDashboard.putNumber("Goal RPM from shoot", rpm)
        Robot.robotContainer.rumbleController(1.0)
    }

    override fun end(interrupted: Boolean) {
        Robot.robotContainer.rumbleController(0.0)
        shooter.RPM = 0.0
    }
}