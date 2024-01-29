package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain drivetrain;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;

    private final Mk4TTBSwerve[] swerveModules;
    private final Mk4TTBSwerve frontLeftSwerveModule, backLeftSwerveModule; 
//ew u think in binary ;-; wat a nerd
    public Drivetrain(){
        frontLeftSwerveModule = new Mk4TTBSwerve(0, Swerve.Mod0.constants);
        backLeftSwerveModule = new Mk4TTBSwerve(1 , Swerve.Mod1.constants);

        //frontRightSwerveModule = new Mk4TTBSwerve(3, Swerve.Mod3.constants);
        //backRightSwerveModule = new Mk4TTBSwerve(2 , Swerve.Mod2.constants);
        

        swerveModules = new Mk4TTBSwerve[] {frontLeftSwerveModule, backLeftSwerveModule};

        swerveModulePositions = new SwerveModulePosition[] {frontLeftSwerveModule.getPosition(), backLeftSwerveModule.getPosition()};

    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }

        return drivetrain;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation){
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

    }

    public void stopSwerveModules(){
        for(Mk4TTBSwerve module : swerveModules){
            module.stop();
        }
    }

    public void turnTest(){
        backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI/2)));
    }

    public void straighten(){
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
        backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0.0)));
    }

    @Override
    public void periodic(){
        for (Mk4TTBSwerve module : swerveModules){
            module.putSmartDashboard();
        }
    }
}
