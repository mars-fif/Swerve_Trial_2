package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class StraightenDrivetrain extends Command{
    private Drivetrain drivetrain;
    private double initialTime;

    public StraightenDrivetrain(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.straighten();
        initialTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished(){
        //return Timer.getFPGATimestamp()-initialTime > 0.5;
        return false;
    }
}
