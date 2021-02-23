package frc.team3324.library.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import frc.team3324.library.subsystems.MotorSubsystem
import java.util.function.BooleanSupplier

class MotorCommand(val subsystem: MotorSubsystem, val speed: Double, val index: Int? = null, val required: Boolean = true, val finishedCondition: () -> Boolean = {false}): CommandBase() {
    init {
        if (required) {
            addRequirements(subsystem)
        }
    }

    override fun execute() {
        if (index != null) {
            subsystem.setSpeed(speed, index)
        } else {
            subsystem.setSpeed(speed)
        }
    }

    override fun end(interrupted: Boolean) {
        subsystem.setSpeed(0.0)
    }

    override fun isFinished(): Boolean {
        return finishedCondition()
    }
}
