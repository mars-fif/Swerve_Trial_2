// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static class IntakeConstants{
    //Change these later! 
    public static final int intake_leftMotorID = 52; 
    public static final int intake_rightMotorID = 51; 
    
    //regular NEOs are 80
    public static final int kIntakeCurrentLimit = 20;

  }

  public static class ShooterConstants{
  
    public static final int shooter_leftMotorID = 54; 
    public static final int shooter_rightMotorID = 53; 

    //Change this later! 
    public static final double shooterVelocityP = 0.005; 
    public static final double shooterVelocityI = 0.0;
    public static final double shooterVelocityD = 0.0;
    
    public static final int kShootCurrentLimit = 80;


  }

  public static class ArmConstants{
    //right: 55
    //left: 56 
    public static final int arm_leftMotorID = 56; 
    public static final int arm_rightMotorID = 55; 

    //Change these to actual values later 
    public static final int topLimitSwitchChannel = 0; 
    public static final int bottomLimitSwitchChannel = 1; 

    public static final int leftEncoderDIO = 0; 
    public static final int rightEncoderDIO = 1;

    //CHANGE THESE LATER! 
    //Encoder values
    //When the arm is positioned at a 90 degree angle (max), the values outputed by absolute encoders
    public static final double leftEncPos = 0.4582; 
    public static final double leftEncoderOffset = 0.7017; //Encoder values when the arm is in its home position
    public static final double leftActualMaxPos = leftEncoderOffset - leftEncPos; 
    public static final double rightEncPos = 0.4751; 
    public static final double rightEncoderOffset = 0.2293; //Encoder values when the arm is in its home position
    public static final double rightActualMaxPos = rightEncPos - rightEncoderOffset;
    public static final double leftEncMaxDegrees = (leftActualMaxPos*360);
    public static final double rightEncMaxDegrees = (rightActualMaxPos*360); 

    public static final int kArmCurrentLimit = 80; 

    //When the arm is going up, it should be using a lower P value, when it's going down it should use a high p value
    public static final double armPosPHigh = 0.05;
    public static final double armPosPLow = 0.005; 
    public static final double armPosI = 0.0; 
    public static final double armPosD = 0.0;
    // D: 0.0002 

    //Velocity PID
    public static final double armVP = 0.05; 
    public static final double armVI = 0.0; 
    public static final double armVD = 0.0; 
  }

  public static class SmolArmConstants{
    public static final int smolArm_MotorID = 57; 
    //Motor gear ratio is at 1:125, neo encoder count is 42
    public static final double sEncoderCountToDegrees = (360/125)*42;

    public static final int smolArmCurrentLimit = 80;

    public static final double armSP = 0.0; 
    public static final double armSI = 0.0; 
    public static final double armSD = 0.0; 

    //Raw - 20?

  }


  public static class AutoConstants{
    public static final double kMaxSpeedMetersPerSec = DriveConstants.kRealMaxSpeedMPS/8; 
    public static final double kMaxAcceleration = .3;
    
    public static final double xControllerP = 7.0;
    //public static final double yControllerP = 0;
    public static final double thetaControllerP = 1;

    //sorry lol these naming schemes succcc
    //Adjust values later

    public static final double kmaxAngularSpeedRadiansPerSec = DriveConstants.kMaxAngularSpeed / 10; 
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; 
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(kmaxAngularSpeedRadiansPerSec, kMaxAngularAccelerationRadiansPerSecondSquared);

  }

  public static class DriveConstants{
    public static final double kTrackWidth = Units.inchesToMeters(24.245);
    public static final double kWheelBase = Units.inchesToMeters(24.245);

    public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth,2) + Math.pow(kWheelBase, 2)) /2;

    public static final double kRealMaxSpeedMPS = 3.81;
    public static final double kMaxAngularSpeed = 4 * Math.PI / 3;
 
    public static final double kNormalModeTranslationSpeedScale = 1.0;
    public static final double kNormalModeRotationSpeedScale = 1.0;
    public static final double kSlowModeTranslationSpeedScale = 0.2;
    public static final double kSlowModeRotationSpeedScale = 0.2;

    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // correction
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 Tooth Bevel Gear, 15 Tooth Bevel Gear Driving, 19 Tooth Second Stage Out, 
    // 25 Tooth Second Stage In, 14 Tooth Pinion, 50 Tooth First Stage
    public static final double kDrivingMotorReduction = (45.0 * 19 * 50) / (kDrivingMotorPinionTeeth * 15 * 25);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                    * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
                    / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
                    / kDrivingMotorReduction) / 60.0; // meters per second

    public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // FL
      new Translation2d(kWheelBase / 2.0, -kTrackWidth/2.0), // FR
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // BL
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // BR
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


      public static final double kHeadingCorrectionP = 0.04;
      public static final double kHeadingCorrectionTolerance = 2.0;
  }

  // Pulled from Team 341 2023-Public Code (Thank you!)
  public static class Swerve{
    public static final class Mod0{
      //Left Front
      public static final int TURNING_SPARK_ID = 11;
      public static final int DRIVING_SPARK_ID = 12;
      public static final Rotation2d angleOffset = new Rotation2d(2.62); //2.62 <--Original
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset, inverted);
  }

    public static final class Mod1{
      //Left Rear
      public static final int TURNING_SPARK_ID = 21;
      public static final int DRIVING_SPARK_ID = 22;
      public static final Rotation2d angleOffset = new Rotation2d(6.04); //1.05
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset, inverted);
  }

    public static final class Mod2{
      //Right Rear
      public static final int TURNING_SPARK_ID = 32;
      public static final int DRIVING_SPARK_ID = 31;
      public static final Rotation2d angleOffset = new Rotation2d(0.88);
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset, inverted);
  }

    public static final class Mod3{
      // Front Right
      public static final int TURNING_SPARK_ID = 41;
      public static final int DRIVING_SPARK_ID = 42;
      public static final Rotation2d angleOffset = new Rotation2d(6.26);
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset, inverted);
  }
}
}

