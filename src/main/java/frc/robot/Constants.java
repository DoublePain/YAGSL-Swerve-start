// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort  = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final double deadband = 0.05;
  
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = 10;
  }
  public static class DriveConstants {
    public static final double k_driveSpeed = 10;
    public static final double k_driveDeadBand = 0.05;
  }


  public static class ElevatorConstants {
    public static final double Kelevatorkp = 5;//5
    public static final double Kelevatorki = 0;
    public static final double Kelevatorkd = 0;//
    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double KMaxAccel = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorS = 0.02;
    public static final double kElevatorG = 0.9;
    public static final double kElevatorV = 3.8;
    public static final double kElevatorA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12.0;
    public static final double kElevatorCarriageMass = 4.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorMinHeight = 0.0;
    public static final double kElevatorMaxHeight = 10.25;
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset          = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);
    public static final      int      kElevatorCurrentLimit   = 40;
    public static final double   kElevatorAllowableError = Units.inchesToMeters(0.005);
    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);;
  }

  //public static final Mechanism2d sideview = new Mechanism2d(Inches.of(31).in(Meters) * 2, ElevatorConstants.kElevatorMaxHeight);
  public static final Mechanism2d sideView = new Mechanism2d(Inches.of(31).in(Meters) * 2, ElevatorConstants.kElevatorMaxHeight);
public static final MechanismRoot2d elevatorCarriage;
public static final MechanismLigament2d elevatorMech;



  static{
    elevatorCarriage = Constants.sideView.getRoot("Elevator Carriage",
                                                         ElevatorConstants.kElevatorStartingHeightSim.in(Meters),
                                                        ElevatorConstants.kElevatorStartingHeightSim.in(Meters));
  
    elevatorMech = elevatorCarriage.append(new MechanismLigament2d("Elevator",
                                                        ElevatorConstants.kElevatorLength,
                                                        ElevatorConstants.kElevatorStartingAngle.in(Degrees),
                                                        6,
                                                        new Color8Bit(Color.kRed)));
  }

  public static class IDConstants {
    public static final int Outtake_Left_ID = 11;
    public static final int Outtake_Right_ID = 12;
    public static final int Elevator_Left_ID = 13;
    public static final int Elevator_Right_ID = 14;
    public static final int Algae_Pivot_ID = 15;
    public static final int Algae_Intake_ID = 16;
    public static final int kLaserId = 17;
    public static final int EkLaserID = 18;
    
  }

  public static class Coral_Algae_Constants {
    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.3;
    public static final double kL1Speed = 0.4;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDifference = kL1Speed * 0.5;
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
