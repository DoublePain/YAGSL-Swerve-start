package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class ScoreCoralCommand extends Command {
    private final CoralSubsystem CoralSubsystem;
    private final boolean level24;  // Boolean to determine if it's Level 1 or Level 24 scoring

    public ScoreCoralCommand(CoralSubsystem CoralSubsystem, boolean level24) {
        this.CoralSubsystem = CoralSubsystem;
        this.level24 = level24;
        addRequirements(CoralSubsystem);  // Ensure CoralSubsystem is required by the command
    }

    @Override
    public void initialize() {
        if (level24) {
            CoralSubsystem.scoreL24();  // Call the method for Level 24 scoring
        } else {
            CoralSubsystem.scoreL1();  // Call the method for Level 1 scoring
        }
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