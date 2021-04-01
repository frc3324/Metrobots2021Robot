package frc.team3324.robot.autocommands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team3324.robot.shooter.Shooter
import frc.team3324.robot.shooter.commands.RunShooter
import frc.team3324.robot.storage.Storage
import frc.team3324.robot.util.FrontCamera

class ShooterAndStorageFiveBalls(shooter: Shooter, area: Double, storage: Storage, waitTime: Double, frontCamera: FrontCamera) : ParallelCommandGroup() {
    init {
        addCommands(RunShooter(shooter, frontCamera, false), ShootFiveBalls(shooter, storage, {area}, waitTime))
    }
}