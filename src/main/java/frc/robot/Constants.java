// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Constants {

  // Constants for controller IDs
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 =1;
    public static final double deadband = 0.05;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    public static final double MAX_SPEED = 10;
  } 

  // Constants for controller input!
  public static final class DriveConstants {
    public static final double k_driveDeadBand = 0.05;
    public static final double k_driveSpeed = 10;
  }
 
  public static class AlgaeArmConstants{ //?
    public static final double kAlgaeArmLength = Inches.of(31).in(Meters); //? need changes too
    public static final Angle kAlgaeArmStartingAngle = Degree.of(90); //? need changes too
  }

  //Need to change all the constants later
  public static class ElevatorConstants{
    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;
    public static final double kMaxVelocity = Meters.of(0.5).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(0.3).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12;
    public static final double kElevatorSproketTeeth = 22;
    public static final double kElevatorPitch = Units.inchesToMeters(0.25);
    public static final double kElevatorCarriageMass = Units.lbsToKilograms(16);
    public static final double kElevatorDrumRadius = (kElevatorSproketTeeth * kElevatorPitch) / (2 *Math.PI);
    public static final double kElevatorMinHeightMeters = Units.inchesToMeters(0);
    public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(30);
    public static final Distance kMinElevatorHeight      = Meters.of(kElevatorMinHeightMeters);
    public static final Distance kMaxElevatorHeight      = Meters.of(kElevatorMaxHeightMeters);
    public static final double   kElevatorAllowableError = Units.inchesToMeters(0.005); 
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);
    public static final double   kElevatorUnextendedHeight    = Units.inchesToMeters(41.5);
    public static double kLowerToScoreHeight = Units.inchesToMeters(6);
  }

  public static final Mechanism2d sideView = new Mechanism2d(AlgaeArmConstants.kAlgaeArmLength *2, 
                                                             ElevatorConstants.kMaxElevatorHeight.in(Meters) +
                                                            AlgaeArmConstants.kAlgaeArmLength +
                                                            ElevatorConstants.kElevatorUnextendedHeight);
  public static final MechanismRoot2d kElevatorCarriage;
  public static final MechanismRoot2d kElevatorJoint;
  public static final MechanismLigament2d kElevatorTower;
  //public static final MechanismLigament2d kElevatorFixed;
  public static final double              maxSpeed      = 7;

  static{
    kElevatorCarriage = Constants.sideView.getRoot("ElevatorCarriage",
                                                        AlgaeArmConstants.kAlgaeArmLength,
                                                     ElevatorConstants.kElevatorStartingHeightSim.in(Meters) +
                                                        ElevatorConstants.kElevatorUnextendedHeight);
    kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
                                        "Elevator",
                                              ElevatorConstants.kElevatorStartingHeightSim.in(Meters),
                                              -90,
                                       6,
                                             new Color8Bit(Color.kRed)));
                                                  
    kElevatorJoint = Constants.sideView.getRoot("ElevatorJoint",
                                                      AlgaeArmConstants.kAlgaeArmLength,
                                                      ElevatorConstants.kElevatorUnextendedHeight);
                                                  
    /*kElevatorFixed = kElevatorJoint.append(new MechanismLigament2d("ElevatorFixed",
                                          ElevatorConstants.kElevatorUnextendedHeight,
                                          -90,
                                          6,
                                          new Color8Bit(Color.kPaleVioletRed)));*/
  }

  public static class IDConstants {
    public static final int Outtake_Left_ID = 11;
    public static final int Outtake_Right_ID = 12;
    public static final int Elevator_Left_ID = 13;
    public static final int Elevator_Right_ID = 14;
    public static final int Algae_Pivot_ID = 15;
    public static final int Algae_Intake_ID = 16;
    public static final int kLaserId =17;
  }

  public static class Coral_Algae_Constants {
    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.3;
    public static final double kL1Speed = 0.4;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDiffernce = kL1Speed * 0.5;
    public static final double kWristEncoderId = 20;
    public static final int mWristEncoderID1 = 0;
    public static final int mWristEncoderID2 = 1;
    public static final boolean mWristEncoderInvert = true;
    public static final int kMaxWristCurrent = 10;

    
    public static final double kWristP = 0.01;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.0;
    public static final double kWristKG = 0.0;
    public static final double kWristKV = 0.100;
    public static final double kWristKA = 0.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    
    public static final double kStowAngle = 233.0;
    public static final double kDeAlgaeAngle = 215.0;
    public static final double kGroundIntakeAngle = 162.0;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kAlgaeIntakeSpeed = 0.6;
    public static final double kEjectSpeed = -0.3;
    public static final double kGroundIntakeSpeed = -0.3;
    
    public static final double kWristOffset = 141.0;
  }
}
