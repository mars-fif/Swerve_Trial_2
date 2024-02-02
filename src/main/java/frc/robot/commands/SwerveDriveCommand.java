package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DriverOI;

public class SwerveDriveCommand extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DriverOI driverOI = DriverOI.getInstance();

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d position;
    position = driverOI.getSwerveTranslation();
    SmartDashboard.putNumber("field relative input forward axis", position.getX());
    SmartDashboard.putNumber("field relative input strafe axis", position.getY());
    double rotation = driverOI.getRotation();
    Translation2d centerOfRotation = driverOI.getCenterOfRotation();

    drivetrain.drive(position, rotation, false, centerOfRotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}