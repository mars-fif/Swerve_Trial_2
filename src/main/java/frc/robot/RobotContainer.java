// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ArmCmds.SetArmHome;
//import frc.robot.commands.IntakeCmds.nomNom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.util.DriverOI;
import frc.robot.util.OperatorOI;
import frc.robot.subsystems.Auto;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.ArmCmds.SetArmHome;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Shooter shooter;
  private final Arm arm;
  private final DriverOI driverOI;
  private final OperatorOI operatorOI;
  //private final Auto auto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    drivetrain.setDefaultCommand(new SwerveDriveCommand());

    arm = Arm.getInstance();
    arm.setDefaultCommand(new SetArmHome());
    // arm.register();

    intake = Intake.getInstance();
    intake.register();

    shooter = Shooter.getInstance();
    shooter.register();

    

    //Xbox controllers: Driver(0), Operator(1)
    driverOI = DriverOI.getInstance(); 
    operatorOI = OperatorOI.getInstance();
    //auto = Auto.getInstance();

  }

  public Command getAutonomousCommand(){
    /* 
    System.out.println("AutoConfig Starting");
    System.out.println("AutoConfig Starting");
    System.out.println("AutoConfig Starting");
    System.out.println("AutoConfig Starting");

    SmartDashboard.putString("Auto Status", "womp2");
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSec, 
      AutoConstants.kMaxAcceleration).setKinematics(DriveConstants.kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, 0)
        //new Translation2d(1, 0)
      ),
      new Pose2d(1, 0, Rotation2d.fromDegrees(180)),
      trajectoryConfig
    );

    PIDController xController = new PIDController(AutoConstants.xControllerP, 0, 0);
    PIDController yController = new PIDController(AutoConstants.yControllerP, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.thetaControllerP, 0, 0, AutoConstants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

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


    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetPose(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> drivetrain.stopSwerveModules())
    );
    */

    // drivetrain.resetGyro();
    drivetrain.resetGyro();
    return null;

  }

  

  


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  //private void configureBindings() {
//
  //}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    
  // }

  //--SmartDashOut for auto
  // public void periodic(){
  //   SmartDashboard.getBoolean("Auto running", getAutonomousCommand().isScheduled());
  // }
}


