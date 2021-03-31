package frc.team3324.robot

import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

    override fun autonomousInit() {
        //CommandScheduler.getInstance().schedule(robotContainer.getAutoCommand())
        enabledInit()
    }

    override fun teleopInit() {
        enabledInit()
    }

    override fun teleopPeriodic() {
    }
}
