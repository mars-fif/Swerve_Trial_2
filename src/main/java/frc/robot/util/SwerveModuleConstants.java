package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;


// Pulled/modified from Team 341's Code Repo (FRC2023-Public)
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final Rotation2d angleOffset;
    public final boolean inverted;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param angleOffset
     * @param inverted
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, Rotation2d angleOffset, boolean inverted) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleOffset = angleOffset;
        this.inverted = inverted;
    }
}