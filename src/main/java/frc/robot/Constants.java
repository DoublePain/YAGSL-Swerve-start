// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  // Constants for controller IDs
  public static final class OperatorConstants {
    public static final int k_driverController = 0;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    public static final double k_maxSpeed = Units.feetToMeters(18.9);
    public static final double k_wheelDiameter = Units.inchesToMeters(2);
    public static final double k_trackwidth = Units.inchesToMeters(28);
    public static final double k_Fvelocity = (1.8);
    public static final double k_Svelocity = (1.8);
    public static final double k_AngularVelocity = (.18);
    public static final double k_FLXOffset = Units.inchesToMeters(13.5);
    public static final double k_FLYOffset = Units.inchesToMeters(13.5);
    public static final double k_FRXOffset = Units.inchesToMeters(13.5);
    public static final double k_FRYOffset = Units.inchesToMeters(-13.5);
    public static final double k_RRXOffset = Units.inchesToMeters(-13.5);
    public static final double k_RRYOffset = Units.inchesToMeters(13.5);
    public static final double k_RLXOffset = Units.inchesToMeters(13.5);
    public static final double k_RLYOffset = Units.inchesToMeters(-13.5);
    public static final double k_WFLOffset = 150; 
    public static final double k_WFROffset = 0;
    public static final double k_WRLOffset = 170;
    public static final double k_WRROffset = 0;

  } 

  // Constants for controller input!
  public static final class DriveConstants {
    public static final double k_driveDeadBand = 0.05;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;
    public static final int k_frontLeftAngleMotorID = 7; //7
    public static final int k_frontRightAngleMotorID = 5; //5
    public static final int k_rearLeftAngleMotorID = 3; //2
    public static final int k_rearRightAngleMotorID = 2; //3
  }
}
