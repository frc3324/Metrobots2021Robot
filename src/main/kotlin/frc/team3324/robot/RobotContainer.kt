package frc.team3324.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team3324.robot.drivetrain.DriveTrain
import frc.team3324.robot.drivetrain.commands.teleop.Drive
import io.github.oblarg.oblog.Logger
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.spline.PoseWithCurvature
import frc.team3324.robot.util.Consts
import java.nio.file.Path
import java.util.function.BiConsumer
import java.util.function.Supplier

import frc.team3324.robot.util.Trajectories


class RobotContainer {
    val driveTrain = DriveTrain()

    private val table = NetworkTableInstance.getDefault()

    private val navChooser = SendableChooser<Trajectory>()

    private val primaryController = XboxController(0)
    private val secondaryController = XboxController(1)

    private val primaryRightX: Double
        get() = primaryController.getX(GenericHID.Hand.kLeft)
    private val primaryLeftY: Double
        get() = primaryController.getY(GenericHID.Hand.kRight)

    private val primaryTriggerRight: Double
        get() = primaryController.getTriggerAxis(GenericHID.Hand.kRight)
    private val primaryTriggerLeft: Double
        get() = primaryController.getTriggerAxis(GenericHID.Hand.kLeft)

    private val secondaryRightX: Double
        get() = secondaryController.getX(GenericHID.Hand.kLeft)
    private val secondRightY: Double
        get() = secondaryController.getY(GenericHID.Hand.kRight)
    private val secondLeftY: Double
        get() = secondaryController.getY(GenericHID.Hand.kLeft)

    private val secondTriggerRight: Double
        get() = secondaryController.getTriggerAxis(GenericHID.Hand.kRight)
    private val secondTriggerLeft: Double
        get() = secondaryController.getTriggerAxis(GenericHID.Hand.kLeft)


   init {
       Robot.light.set(true)
       Logger.configureLoggingAndConfig(this, true)
       driveTrain.defaultCommand = Drive(driveTrain, {primaryController.getY(GenericHID.Hand.kLeft)}, {primaryController.getX(GenericHID.Hand.kRight)})
       navChooser.setDefaultOption("Test Line", Trajectories.TestLine.trajectory)
       navChooser.addOption("Galactic A_R", Trajectories.GalacticAR.trajectory)
       navChooser.addOption("Galactic A_B", Trajectories.GalacticAB.trajectory)
       navChooser.addOption("Galactic B_R", Trajectories.GalacticBR.trajectory)
       navChooser.addOption("Galactic B_B", Trajectories.GalacticBB.trajectory)

       SmartDashboard.putData(navChooser)

       configureButtonBindings()
   }

    private fun configureButtonBindings() {
        JoystickButton(primaryController, Button.kB.value).whenPressed(getRamseteCommand(navChooser.selected))

        JoystickButton(primaryController, Button.kStart.value).whenPressed(Runnable{driveTrain.resetOdometry(navChooser.selected.initialPose)}, driveTrain)
        /*JoystickButton(primaryController, Button.kY.value).whileHeld(GyroTurn(
                driveTrain,
                1.0/70.0,
                (Consts.DriveTrain.ksVolts + 0.3)/12,
                {cameraTable.getEntry("targetYaw").getDouble(0.0)},
                   {input -> driveTrain.curvatureDrive(0.0, input, true)}
        ))*/

    }


    fun rumbleController(rumbleLevel: Double) {
        secondaryController.setRumble(GenericHID.RumbleType.kRightRumble, rumbleLevel)
    }

    fun getRamseteCommand(trajectory: Trajectory): Command {

        val disabledRamsete = object : RamseteController() {
            override fun calculate(currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMeters: Double, angularVelocityRefRadiansPerSecond: Double): ChassisSpeeds {
                return ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond)
            }
        }

        val leftController = PIDController(Consts.DriveTrain.kP, 0.0, 0.0)
        val rightController = PIDController(Consts.DriveTrain.kP, 0.0, 0.0)

        val ramseteCommand = RamseteCommand(
                trajectory,
                driveTrain::pose,
                RamseteController(Consts.DriveTrain.kRamseteB, Consts.DriveTrain.kRamseteZeta),
                SimpleMotorFeedforward(Consts.DriveTrain.ksVolts,
                                        Consts.DriveTrain.LOW_GEAR_KV,
                                        Consts.DriveTrain.LOW_GEAR_KA),
                Consts.DriveTrain.kDriveKinematics,
                driveTrain::wheelSpeeds,
                leftController,
                rightController,
                {leftVolts: Double, rightVolts: Double ->
                    driveTrain.tankDriveVolts(leftVolts, rightVolts)

                    SmartDashboard.putNumber("Left Measurement", driveTrain.autoWheelSpeeds.leftMetersPerSecond)
                    SmartDashboard.putNumber("Left Reference", leftController.setpoint)

                    SmartDashboard.putNumber("Right Measurement", driveTrain.autoWheelSpeeds.rightMetersPerSecond)
                    SmartDashboard.putNumber("Right Reference", rightController.setpoint)
                },
                arrayOf(driveTrain)
        )

        return ramseteCommand.andThen({driveTrain.tankDriveVolts(0.0, 0.0)}, arrayOf(driveTrain))
    }

    fun selectedTrajectory():Trajectory {
        return navChooser.selected
    }

    fun importTrajectory(navPath: String): Trajectory {
        var path = "paths/" + navPath
        val trajectoryPath: Path = Filesystem.getDeployDirectory().toPath().resolve(path)
        return TrajectoryUtil.fromPathweaverJson(trajectoryPath)
    }
}