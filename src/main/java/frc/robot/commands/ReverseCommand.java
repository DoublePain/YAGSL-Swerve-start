package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class ReverseCommand extends Command {
    private final CoralSubsystem CoralSubsystem;

    public ReverseCommand(CoralSubsystem CoralSubsystem) {
        this.CoralSubsystem = CoralSubsystem;
        addRequirements(CoralSubsystem);  // Ensure CoralSubsystem is required by the command
    }

    @Override
    public void initialize() {
        CoralSubsystem.reverse();  // Call the reverse method on the CoralSubsystem
    }

    @Override
    public boolean isFinished() {
        return false;  // The command will run until explicitly interrupted
    }

    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.stopCoral();  // Stop the coral subsystem when the command ends
    }
}


