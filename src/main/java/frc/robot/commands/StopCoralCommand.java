package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class StopCoralCommand extends Command{

    private final CoralSubsystem CoralSubsystem;

     public StopCoralCommand(CoralSubsystem CoralSubsystem) {
            this.CoralSubsystem = CoralSubsystem;
            addRequirements(CoralSubsystem);
        }

        @Override
        public void initialize() {
            CoralSubsystem.stopCoral();
        }

        @Override
        public void end(boolean interrupted) {
            CoralSubsystem.stopCoral();
        }

        @Override
        public boolean isFinished() {
            return true;  // The command finishes immediately
        }
    }
    

