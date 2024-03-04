package frc.robot.subsystems;

<<<<<<< Updated upstream
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
=======
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
>>>>>>> Stashed changes

public class Auto extends SubsystemBase{
    private static Auto auto;
    private final Drivetrain drivetrain;

    private SendableChooser<Command> autoRoutineChooser;
    private Hashtable<String,Command> autoRoutines;

    private final TrajectoryConfig trajectoryConfig;

    private final PIDController xController; 
    private final PIDController yController; 
    private final ProfiledPIDController thetaController; 

    private Trajectory MoveOut, NoMove;

<<<<<<< Updated upstream
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

        
=======
    public Auto(){
        autoRoutines = new Hashtable<String,Command>();
        autoRoutineChooser = new SendableChooser<Command>();

        drivetrain = Drivetrain.getInstance();

        xController = new PIDController(AutoConstants.xControllerP, 0, 0);
        yController = new PIDController(AutoConstants.yControllerP, 0, 0);
        thetaController = new ProfiledPIDController(AutoConstants.thetaControllerP, 0, 0, AutoConstants.thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSec, 
        AutoConstants.kMaxAcceleration).setKinematics(DriveConstants.kinematics);

        defineAutoPaths();
        setupAutoRoutines();
        setupAutoChooser();

      
    }

    public SwerveControllerCommand configAutoWithTrajectory(Trajectory trajectory){
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        drivetrain::getPose,
        DriveConstants.kinematics,
        xController,
        yController,
        thetaController,
        drivetrain::setSwerveModuleStates,
        drivetrain);
        SmartDashboard.putString("Auto Status", "womp3");
        SmartDashboard.putNumber("Current Pose", drivetrain.getPose().getX());

        return swerveControllerCommand;
    }

    public void defineAutoPaths(){
        //REMEMBER THAT IT USUALLY MOVES DOUBLE ITS DISTANCE! 

        //Doesn't move at all 
        NoMove = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        ),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        

        // Moves straight forward by 2 meters
        System.out.println("Move forward");
        MoveOut = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        new Translation2d(0, 0)
        ),
        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        
    }

    public void setupAutoChooser(){
        Enumeration<String> e = autoRoutines.keys();

        while(e.hasMoreElements()){
            String autoRoutineName = e.nextElement();
            autoRoutineChooser.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        };

        SmartDashboard.putData("Auto Routines", autoRoutineChooser);
    }

    public void setupAutoRoutines(){
        autoRoutines.put("Nothing lol", configAutoWithTrajectory(NoMove));
        autoRoutines.put("Move Out", configAutoWithTrajectory(MoveOut));
    }

    public Command getAutonomousCommand(){
        //return autoRoutineChooser.getSelected();
        return new SequentialCommandGroup(
            
            autoRoutineChooser.getSelected(),
        new InstantCommand(() -> drivetrain.stopSwerveModules())
    );
    }
>>>>>>> Stashed changes


<<<<<<< Updated upstream

    }

    
=======
    public static Auto getInstance(){
        if(auto == null){
            auto = new Auto();
        }
        return auto;
    }

>>>>>>> Stashed changes
}
