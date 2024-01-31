// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConstants;
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
  }

  public static class NeoMotorConstants{
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class DriveConstants{
    public static final double kTrackWidth = Units.inchesToMeters(24.245);
    public static final double kWheelBase = Units.inchesToMeters(24.245);

    public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth,2) + Math.pow(kWheelBase, 2)) /2;

    public static final double kRealMaxSpeedMPS = 3.81;
    public static final double kMaxAngularSpeed = 4 * Math.PI / 3;
 

    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // correction
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 Tooth Bevel Gear, 15 Tooth Bevel Gear Driving, 19 Tooth Second Stage Out, 
    // 25 Tooth Second Stage In, 14 Tooth Pinion, 50 Tooth First Stage
    public static final double kDrivingMotorReduction = (45.0 * 19 * 50) / (kDrivingMotorPinionTeeth * 15 * 25);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                    * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                    / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                    / kDrivingMotorReduction) / 60.0; // meters per second

    public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // LF
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), // LR
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // RR
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0) // RF
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      swerveModuleLocations[0],
      swerveModuleLocations[1],
      swerveModuleLocations[2],
      swerveModuleLocations[3]);


      public static final boolean kUseRateLimit = true;
      public static final double kDirectionSlewRate = 4.5;
      public static final double kMagnitudeSlewRate = 4.5;
      public static final double kRotationalSlewRate = 2.0;


      public static final double kHeadingCorrectionP = 0.05;
      public static final double kHeadingCorrectionTolerance = 2.0;
  }

  // Pulled from Team 341 2023-Public Code (Thank you!)
  public static class Swerve{
    public static final class Mod0{
      public static final int TURNING_SPARK_ID = 11;
      public static final int DRIVING_SPARK_ID = 12;
      public static final Rotation2d angleOffset = new Rotation2d(5.74); //2.63

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }

    public static final class Mod1{
      //Left Rear
      public static final int TURNING_SPARK_ID = 21;
      public static final int DRIVING_SPARK_ID = 22;
      public static final Rotation2d angleOffset = new Rotation2d(2.8); //1.05

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }

    public static final class Mod2{
      public static final int TURNING_SPARK_ID = 32;
      public static final int DRIVING_SPARK_ID = 31;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.97);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }

    public static final class Mod3{
      public static final int TURNING_SPARK_ID = 41;
      public static final int DRIVING_SPARK_ID = 42;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(6.27);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }
}
}

