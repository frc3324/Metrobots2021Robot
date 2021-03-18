package frc.team3324.robot.util

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj.util.Units

object Trajectories {

    // trajectory configuration
    private val autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforward(Consts.DriveTrain.ksVolts,
                    Consts.DriveTrain.LOW_GEAR_KV,
                    Consts.DriveTrain.LOW_GEAR_KA),
            Consts.DriveTrain.kDriveKinematics, 10.0) // maxVoltage of 10 allows for head room

    private val config = TrajectoryConfig(1.0, Consts.DriveTrain.LOW_GEAR_MAX_ACCELERATION)
            .setKinematics(Consts.DriveTrain.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)


    // trajectories
    object GalacticAR {
        private val startPose = Pose2d(0.0, Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?

        private val interiorPoses = listOf(
                Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(15.0-7.5)),  // ball 1
                Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(15.0-5.0)), // ball 2
                Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(15.0-12.5)), // ball 3
                Translation2d(Units.feetToMeters(24.5), Units.feetToMeters(15.0-7.5)) // line up for finish
        )

        private val endPose = Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(15.0-12.5), Rotation2d.fromDegrees(0.0)) // end at 25 to not slam into wall

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object GalacticAB {
        // private val startPose = Pose2d(0.0, Units.feetToMeters(15.0-2.5), Rotation2d.fromDegrees(0.0))
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?


        private val interiorPoses = listOf(
                Translation2d(Units.feetToMeters(6.0), Units.feetToMeters(15.0-7.5)), // move out of start
                Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(15.0-2.5)),  // ball 1
                Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(15.0-10.0)), // ball 2
                Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(15.0-7.5)) // ball 3
        )

        private val endPose = Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(15.0-7.5), Rotation2d.fromDegrees(0.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object GalacticBR {
        // private val startPose = Pose2d(0.0, Units.feetToMeters(15.0-10.0), Rotation2d.fromDegrees(0.0))
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?

        private val interiorPoses = listOf(
                Translation2d(Units.feetToMeters(6.0), Units.feetToMeters(15.0-7.5)), // move out of start
                Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(15.0-10.0)),  // ball 1
                Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(15.0-5.0)), // ball 2
                Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(15.0-10.5)), // ball 3
                Translation2d(Units.feetToMeters(24.5), Units.feetToMeters(15.0-7.5)) // line up for finish
        )

        private val endPose = Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(15.0-10.0), Rotation2d.fromDegrees(0.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object GalacticBB {
        //private val startPose = Pose2d(0.0, Units.feetToMeters(15.0-5.0), Rotation2d.fromDegrees(0.0))
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?


        private val interiorPoses = listOf(
                Translation2d(Units.feetToMeters(6.0), Units.feetToMeters(15.0-7.5)), // move out of start
                Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(15.0-5.0)),  // ball 1
                Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(15.0-10.0)), // ball 2
                Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(15.0-5.0)), // ball 3
                Translation2d(Units.feetToMeters(24.5), Units.feetToMeters(15.0-7.5)) // line up for finish
        )

        private val endPose = Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(15.0-5.0), Rotation2d.fromDegrees(0.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object BarrelRacingPath {
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?


        private val interiorPoses = listOf(
            Translation2d(Units.feetToMeters(14.5), Units.feetToMeters(7.5)),  // D5 loop start
            Translation2d(Units.feetToMeters(14.5), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(10.5), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(10.5), Units.feetToMeters(7.5)), // D5 loop end
            Translation2d(Units.feetToMeters(22.0), Units.feetToMeters(7.5)), // B8 loop start
            Translation2d(Units.feetToMeters(22.0), Units.feetToMeters(3.0)),
            Translation2d(Units.feetToMeters(18.0), Units.feetToMeters(3.0)),
            Translation2d(Units.feetToMeters(18.0), Units.feetToMeters(7.5)), // B8 loop end
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(11.5)), // D10 loop start
            Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(11.5)),
            Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(7.5)), //D10 loop end
        )

        private val endPose = Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object BouncePath {
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?


        private val interiorPoses = listOf(
            Translation2d(Units.feetToMeters(6.0), Units.feetToMeters(7.5)), // move out of start
            Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(2.5)),  // A3
            Translation2d(Units.feetToMeters(8.5), Units.feetToMeters(7.5)),
            Translation2d(Units.feetToMeters(11.0), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(2.5)), // A6
            Translation2d(Units.feetToMeters(16.0), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(21.5), Units.feetToMeters(12.0)),
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(2.5)), // A9
            Translation2d(Units.feetToMeters(24.5), Units.feetToMeters(7.5)) // line up for finish
        )

        private val endPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }

    object SlalomPath {
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(12.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?


        private val interiorPoses = listOf(
            Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5)), // move out of start
            Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(7.5)),  // D4-8 loop start
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(7.5)), // D10 loop start
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(12.5)),
            Translation2d(Units.feetToMeters(27.5), Units.feetToMeters(12.5)),
            Translation2d(Units.feetToMeters(27.5), Units.feetToMeters(7.5)),
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(7.5)),
            Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(12.5)), // D10 loop end
            Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5)),
            Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(7.5)) // D4-8 loop end
        )

        private val endPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180.0))

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }


    object TestLine {
        private val startPose = Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // should x be half of robot length?

        private val interiorPoses = listOf(Translation2d(Units.feetToMeters(5.0), Units.feetToMeters(7.5)))

        private val endPose = Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.0)) // end at 25 to not slam into wall

        val trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorPoses, endPose, config)
    }
}