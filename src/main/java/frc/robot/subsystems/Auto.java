package frc.robot.subsystems;

import javax.swing.text.Position;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DriverOI;
import java.util.function.Supplier;
import java.util.function.Consumer;

public class Auto extends SubsystemBase{
    private static Auto auto;

    public Auto(Drivetrain mDrivetrain, Supplier<Pose2d> getPose, Consumer<Pose2d> resetPose){
//ppppp

        AutoBuilder.configureHolonomic(
            getPose, // change to get cur |
            resetPose,
            mDrivetrain::getRobotChassisSpeeds,
            mDrivetrain::autoDrive,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              //Change later - get select from driver station
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        



    }

    
}
