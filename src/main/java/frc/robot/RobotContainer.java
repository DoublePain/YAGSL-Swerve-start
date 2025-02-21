// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.StopCoralCommand;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.commands.StopAlgaeCommand;
import frc.robot.commands.StowCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem      drivebase          =  new SwerveSubsystem();
  private final ElevatorSubsystem    elevator           =  new ElevatorSubsystem();
  private final CoralSubsystem       m_coralSubsystem   =  new CoralSubsystem();
  private final AlgaeSubsystem       algaeSubsystem     =  new AlgaeSubsystem();
  
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

      private final CommandXboxController m_driverController2 = new CommandXboxController(
        OperatorConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY()  ,
                                                                () -> m_driverController.getLeftX()  )
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.deadband)
                                                            .scaleTranslation(0.5)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2)*0.5)
                                                                    .deadband(OperatorConstants.deadband)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));


    elevator.setDefaultCommand(elevator.setElevatorHeight(1));
   SmartDashboard.putData("Side View", Constants.sideView);
    configureBindings();
    
    m_driverController.button(1).whileTrue(new GrabAlgaeCommand(algaeSubsystem));  // Button 1 for GrabAlgaeCommand
    m_driverController.button(1).whileFalse(new StopAlgaeCommand(algaeSubsystem));  // Button 1 for StopAlgaeCommand

    m_driverController.button(2).whileTrue(new ScoreCoralCommand(m_coralSubsystem,true)); // Button 6 for ScoreCoralCommand
    m_driverController.button(3).whileTrue(new ScoreCoralCommand(m_coralSubsystem,false)); 
    
    m_driverController.button(2).whileFalse(new StopCoralCommand(m_coralSubsystem)).and
  (m_driverController.button(3).whileFalse(new StopCoralCommand(m_coralSubsystem)));  // Button 7 for StopCoralCommand
    

    //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
  

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

m_driverController.button(1).whileTrue(elevator.setElevatorHeight(4));
 
  // Example: Mapping button 1 to the `ScoreCoralCommand`
   m_driverController2.button(11).whileTrue(elevator.setGoal(3));
   m_driverController2.button(12).whileTrue(elevator.setGoal(6));
   m_driverController2.button(13).whileTrue(elevator.setGoal(9));
   m_driverController.button(1).whileTrue(driveFieldOrientedDirectAngleKeyboard);
   
}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Mid two coral");
  }
}
