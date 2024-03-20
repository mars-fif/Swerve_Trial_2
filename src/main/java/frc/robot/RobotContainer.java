// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ArmCmds.SetArmHome;
import frc.robot.commands.AutoCmds.A_RunIntakeIn;
import frc.robot.commands.AutoCmds.A_RunIntakeShot;
import frc.robot.commands.AutoCmds.A_SetArmHome;
import frc.robot.commands.AutoCmds.A_SetArmSpeaker;
import frc.robot.commands.AutoCmds.A_Shoot;
//import frc.robot.commands.IntakeCmds.nomNom;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.DriverOI;
import frc.robot.util.OperatorOI;
//import frc.robot.subsystems.Auto;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.ArmCmds.SetArmHome;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import edu.wpi.first.util.sendable.Sendable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.commands.AutoCmds.A_RunIntakeIn;
import frc.robot.commands.AutoCmds.A_Shoot;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final DriverOI driverOI;
  private final OperatorOI operatorOI;
  private final Arm arm;
  private final Intake intake; 
  private final Shooter shooter;
  //private final Auto auto;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    drivetrain.setDefaultCommand(new SwerveDriveCommand());


    arm = Arm.getInstance();
    arm.setDefaultCommand(new SetArmHome());
    //arm.register();

    intake = Intake.getInstance();
    //intake.register();

    shooter = Shooter.getInstance();
    //shooter.register();


    //Xbox controllers: Driver(0), Operator(1)
    driverOI = DriverOI.getInstance(); 
    operatorOI = OperatorOI.getInstance();

    //auto = Auto.getInstance();
    //Setting named commands
    NamedCommands.registerCommand("Intake Shwat", new A_RunIntakeShot());
    NamedCommands.registerCommand("Set Arm Home", new A_SetArmHome());
    NamedCommands.registerCommand("Intake In", new A_RunIntakeIn());
    NamedCommands.registerCommand("Shooter", new A_Shoot());
    NamedCommands.registerCommand("Set Arm Speaker", new A_SetArmSpeaker());
    

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand(){
    drivetrain.resetGyro();
    return autoChooser.getSelected();
    //return null;
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
}
