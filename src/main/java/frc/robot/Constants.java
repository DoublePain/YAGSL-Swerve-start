// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
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
    public static final int kDriverControllerPort = 0;
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
    public static final double Kelevatorkp = 5;
    public static final double Kelevatorki = 0;
    public static final double Kelevatorkd = 0;
    public static final double kMaxVelocity =  Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double KMaxAccel = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorS = 0.02;
    public static final double kElevatorG = 0.9;
    public static final double kElevatorV = 3.8;
    public static final double kElevatorA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12.8;
    public static final double kElevatorCarriageMass = 4.8;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2);
    public static final double kElevatorMaxHeight = 0;
    public static final double kElevatorMinHeight = 10.25;
  }

}
