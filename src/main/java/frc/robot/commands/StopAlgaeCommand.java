package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class StopAlgaeCommand extends Command {

    private final AlgaeSubsystem algaeSubsystem;

    public StopAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        addRequirements(algaeSubsystem);  // Make sure the subsystem is required by this command
    }

    @Override
    public void initialize() {
        algaeSubsystem.stop();  // Call stop method from AlgaeSubsystem to stop the intake and wrist
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();  // Ensure the subsystem stops when the command ends
    }

    @Override
    public boolean isFinished() {
        return true;  // The command finishes immediately after being initialized
    }
}