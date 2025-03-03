package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class GrabAlgaeCommand extends Command {

    private final AlgaeSubsystem algaeSubsystem;

    public GrabAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        addRequirements(algaeSubsystem);  // Make sure the subsystem is required by this command
    }

    @Override
    public void initialize() {}

    @Override
  public void execute() {
    algaeSubsystem.grabAlgae();
  }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();  // Ensure the subsystem stops when the command ends
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}