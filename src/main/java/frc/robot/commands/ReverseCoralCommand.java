package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class ReverseCoralCommand extends Command {
    private final CoralSubsystem CoralSubsystem;

    public ReverseCoralCommand(CoralSubsystem CoralSubsystem) {
        this.CoralSubsystem = CoralSubsystem;
        addRequirements(CoralSubsystem);  // Ensure CoralSubsystem is required by the command
    }

    @Override
    public void initialize() {}

    @Override
  public void execute() {
    CoralSubsystem.reverse();
  }

    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.stopCoral();  // Stop the coral subsystem when the command ends
    }
    
    @Override
    public boolean isFinished() {
        return false;  // The command will run until explicitly interrupted
    }
}


