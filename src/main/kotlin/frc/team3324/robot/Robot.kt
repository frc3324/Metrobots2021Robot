package frc.team3324.robot

import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team3324.robot.util.Trajectories
import io.github.oblarg.oblog.Logger

class Robot: TimedRobot() {

    private val compressor = Compressor()
    private val ultrasonic = AnalogInput(1)
    private val depRangeInches: Double
        get() = ultrasonic.value.toDouble() * 0.125

    companion object {
        val light = DigitalOutput(1)
        val robotContainer = RobotContainer()
        val pdp = PowerDistributionPanel()
    }

    override fun robotInit() {
        SmartDashboard.putNumber("Volt", 0.0)
        //CameraServer.getInstance().startAutomaticCapture()
        LiveWindow.disableAllTelemetry()
        compressor.start()

    }

    fun enabledInit() {
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        Logger.updateEntries()
        Logger.updateEntries()
    }

    override fun teleopInit() {
        enabledInit()
        CommandScheduler.getInstance().cancelAll()

        // reset things for auto testing
        robotContainer.driveTrain.resetEncoders()
        robotContainer.driveTrain.resetOdometry(Trajectories.TestLine.trajectory.initialPose)
    }


    override fun autonomousInit() {
        val trajectory = robotContainer.selectedTrajectory()

        robotContainer.driveTrain.resetEncoders()
        robotContainer.driveTrain.resetOdometry(trajectory.initialPose)
        robotContainer.getRamseteCommand(trajectory).schedule()
    }

    override fun teleopPeriodic() {
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
