package frc.team3324.robot.autocommands

import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team3324.library.commands.MotorCommand
import frc.team3324.library.subsystems.MotorSubsystem
import frc.team3324.robot.Robot.Companion.robotContainer
import frc.team3324.robot.intake.Pivot

class GalacticAutoGroup(pivot: Pivot, intake: MotorSubsystem, trajectory: Trajectory): SequentialCommandGroup() {
    init {
        // TODO: update command group to use trajectories after drivetrain_only merge
        //addCommands(MotorCommand(pivot, 0.4,finishedCondition = {!pivot.lowerLimitSwitch.get()}), robotContainer.getAutoCommand().alongWith(MotorCommand(intake, 1.0)))
    }
}