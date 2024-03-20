package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class A_Shoot extends Command{
    private Shooter shooter;
    private Timer timer; 
    private static double startTime; 
    //private static ParallelDeadlineGroup deadlineGroup; 

    public A_Shoot(){
        shooter = Shooter.getInstance();
        addRequirements(shooter);
        
    }

    @Override 
    public void initialize(){
        startTime = timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        shooter.setSpeed(-.7);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (timer.getFPGATimestamp() - startTime > 4){
            return true;
        }
        return false;
    }
}
