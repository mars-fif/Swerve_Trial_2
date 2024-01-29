// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
      public static final Rotation2d angleOffset = new Rotation2d(1.05); //1.05

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }

    public static final class Mod2{
      public static final int TURNING_SPARK_ID = 32;
      public static final int DRIVING_SPARK_ID = 31;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.0);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }

    public static final class Mod3{
      public static final int TURNING_SPARK_ID = 41;
      public static final int DRIVING_SPARK_ID = 42;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.0);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVING_SPARK_ID, TURNING_SPARK_ID,
        angleOffset);
  }
}
}

