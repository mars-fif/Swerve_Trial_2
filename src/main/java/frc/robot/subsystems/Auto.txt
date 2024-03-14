package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class Auto extends SubsystemBase{
  private static Drivetrain driveTrain; 
  private static Auto auto; 

  private final SendableChooser<Command> autoChooser;
  private Field2d field = new Field2d();

  public Auto(){
      AutoBuilder.configureHolonomic(
          driveTrain::getPose, // Robot pose supplier
          driveTrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          driveTrain::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          driveTrain::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                  new PIDConstants(AutoConstants.xControllerP, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(AutoConstants.thetaControllerP, 0.0, 0.0), // Rotation PID constants
                  DriveConstants.kRealMaxSpeedMPS, // Max module speed, in m/s
                  DriveConstants.kBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                  new ReplanningConfig() // Default path replanning config. See the API for the options here
          ),
          () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          driveTrain // Reference to this subsystem to set requirements
      );

    autoChooser = AutoBuilder.buildAutoChooser();

  }

  public static Auto getInstance(){
    if(auto == null){
        auto = new Auto();
    }

    return auto;
  }

  @Override
  public void periodic(){
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }
}
