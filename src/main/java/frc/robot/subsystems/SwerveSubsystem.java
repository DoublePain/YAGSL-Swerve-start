// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.io.File;
import java.util.function.DoubleSupplier;

/* 
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
*/
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
/* 
import limelight.Limelight;
import limelight.structures.LimelightSettings;
import limelight.estimator.LimelightPoseEstimator;
import limelight.estimator.PoseEstimate;
import limelight.structures.AngularVelocity3d;
import limelight.structures.LimelightResults;
import limelight.structures.LimelightSettings.LEDMode;
import limelight.structures.Orientation3d;
import limelight.structures.target.pipeline.NeuralClassifier;
*/
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.math.geometry.Pose2d;

// Swerve Subsystem Code yippee
public class SwerveSubsystem extends SubsystemBase {

  // Imports stuff from the JSON Files
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;
 
 /*  Limelight                      limelight;
  LimelightPoseEstimator         poseEstimator;
   Pose3d                         cameraOffset        = new Pose3d(Inches.of(5).in(Meters),
                                                                  Inches.of(5).in(Meters),
                                                                  Inches.of(5).in(Meters),
                                                                  Rotation3d.kZero);
*/
  // Creates a New SwerveSubsystem
  public SwerveSubsystem() {

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // TURN OFF DURING COMPETITION BECAUSE IT * WILL *  SLOW YOUR ROBOT
    // (It's for displaying info in Shuffleboard)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    // Initializes robot using the JSON Files with all the constants so you don't have to. Hooray!
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.k_maxSpeed);
    } 
      catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    setupPathPlanner();
    // Cosine Compensator makes your robot slower on some wheels. Set it to false if it drives funky
    swerveDrive.setCosineCompensator(false);

   /*limelight = new Limelight("limelight");
    limelight.getSettings()
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(cameraOffset)
             .save();
    poseEstimator = limelight.getPoseEstimator(true);
    */
  }

  // Command to drive the robot using translative values and heading as angular velocity.
  // translationX - Translation in the X direction. Cubed for smoother controls.
  // translationY - Translation in the Y direction. Cubed for smoother controls.
  // angularRotationX - Angular velocity of the robot to set. Cubed for smoother controls.
  // Returns Drive command.

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
    });
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

   public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }
  
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {}

  @Override
  // This method will be called once per scheduler run during simulation
  public void simulationPeriodic() {}

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

   public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }


}


