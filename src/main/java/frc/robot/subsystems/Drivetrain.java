package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Swerve;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain drivetrain;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;

    private final Mk4TTBSwerve[] swerveModules;
    private final Mk4TTBSwerve frontLeftSwerveModule, backRightSwerveModule, backLeftSwerveModule, frontRightSwerveModule;

    private double lastestChassisSpeed;

    private final ADIS16470_IMU gyro;
    private double heading;
    private boolean isFlipped;

    private boolean useHeadingCorrection;
    private Rotation2d correctHeadingTargetHeading;
    private Timer correctHeadingTimer;
    private double correctHeadingPreviousTime;
    private double correctHeadingOffTime;

    private final SwerveDrivePoseEstimator odometry;



    public Drivetrain(){
        frontLeftSwerveModule = new Mk4TTBSwerve(2, Swerve.Mod2.constants);
        frontRightSwerveModule = new Mk4TTBSwerve(1, Swerve.Mod1.constants);
        backLeftSwerveModule = new Mk4TTBSwerve(3 , Swerve.Mod3.constants);
        backRightSwerveModule = new Mk4TTBSwerve(0 , Swerve.Mod0.constants);

        
        

        swerveModules = new Mk4TTBSwerve[] {frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule};

        swerveModulePositions = new SwerveModulePosition[] {
            frontLeftSwerveModule.getPosition(), 
            frontRightSwerveModule.getPosition(), 
            backLeftSwerveModule.getPosition(), 
            backRightSwerveModule.getPosition()};

        gyro = new ADIS16470_IMU();
        isFlipped = true;

        lastestChassisSpeed = 0.0;

        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, getHeadingAsRotation2d(), 
            new SwerveModulePosition[]
            {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                backLeftSwerveModule.getPosition(),
                backRightSwerveModule.getPosition()}, 
                new Pose2d(), 
                VecBuilder.fill(0.1, 0.1, 0.1), 
                VecBuilder.fill(0.9,0.9,0.9));

        useHeadingCorrection = true;
        correctHeadingTimer = new Timer();
        correctHeadingTimer.start();
        correctHeadingPreviousTime = 0.0;
        correctHeadingOffTime = 0.0;
        correctHeadingTargetHeading = getHeadingAsRotation2d();
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }

        return drivetrain;
    }

    public double getSpeed(){
        return lastestChassisSpeed;
    }

    public void setUseHeadingCorrection(boolean enable){
        useHeadingCorrection = enable;
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kRealMaxSpeedMPS); //12.5 per SDS for L1
    
        for(int i=0; i < swerveModules.length; i++){
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }

    }

    public void stopSwerveModules(){
        for(Mk4TTBSwerve module : swerveModules){
            module.stop();
        }
    }

    public ChassisSpeeds getRobotChassisSpeeds(){
        return DriveConstants.kinematics.toChassisSpeeds(frontLeftSwerveModule.getState(), 
                                                        frontRightSwerveModule.getState(), 
                                                        backLeftSwerveModule.getState(),
                                                        backRightSwerveModule.getState());
    }

    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){
        double correctHeadingCurrentTime = correctHeadingTimer.get();
        double dt = correctHeadingCurrentTime - correctHeadingPreviousTime;

        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.sqrt(Math.pow(desiredSpeed.vxMetersPerSecond, 2) + Math.pow(desiredSpeed.vyMetersPerSecond, 2));

        if(vr > 0.01 || vr < -0.01){
            correctHeadingOffTime = correctHeadingCurrentTime;
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if(correctHeadingCurrentTime - correctHeadingOffTime < 0.5){
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if (v < 0.05){
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }

        correctHeadingTargetHeading = correctHeadingTargetHeading.plus(new Rotation2d(vr * dt));
        Rotation2d currentHeading = getHeadingAsRotation2d();

        Rotation2d deltaHeading = correctHeadingTargetHeading.minus(currentHeading);

        if(Math.abs(deltaHeading.getDegrees()) < DriveConstants.kHeadingCorrectionTolerance){
            return desiredSpeed;
        }

        double correctedVr = deltaHeading.getRadians() / dt * DriveConstants.kHeadingCorrectionP;

        correctHeadingPreviousTime = correctHeadingCurrentTime;
        return new ChassisSpeeds(
            desiredSpeed.vxMetersPerSecond,
            desiredSpeed.vyMetersPerSecond,
            correctedVr);
    }

    public void straighten(){
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
        backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0.0)));
        backRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0.0)));
        frontRightSwerveModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRoation){
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

        if(useHeadingCorrection){
            fieldRelativeSpeeds = correctHeading(fieldRelativeSpeeds);
        }

        if(fieldOriented){
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, 
                                Rotation2d.fromDegrees((isFlipped ? 180 : 0) + getHeading()));
        }else{
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }


        lastestChassisSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2) + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRoation);

        setSwerveModuleStates(swerveModuleStates);
    }

    public double getHeading(){
        heading = gyro.getAngle(gyro.getYawAxis());
        return Math.IEEEremainder(heading, 360);
    }

    public Rotation2d getHeadingAsRotation2d(){
        return Rotation2d.fromDegrees(heading);
    }

    public void resetGyro(){
        gyro.reset();
        isFlipped = true;
    }

    public Pose2d getPose(){
        return odometry.getEstimatedPosition();
    }

    public void updateOdometry(){
        odometry.updateWithTime(Timer.getFPGATimestamp(), getHeadingAsRotation2d(), swerveModulePositions);
    }

    public void setFlipped(){
        isFlipped = Math.abs(getPose().getRotation().getDegrees()) < 90;
    }

    public void setFlipped(boolean bool){
        isFlipped = bool;
    }

    public boolean getFlipped(){
        return isFlipped;
    }

    @Override
    public void periodic(){
        for (Mk4TTBSwerve module : swerveModules){
            module.putSmartDashboard();
        }

        for(int i = 0; i < 4; i++){
            swerveModulePositions[i] = swerveModules[i].getPosition();
        }

        SmartDashboard.putNumber("Gyro Raw Heading", getHeading());
        
        updateOdometry();
    }
}
