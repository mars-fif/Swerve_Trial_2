package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class A_Shoot extends Command{
    private Shooter shooter;
    private double time;

    public A_Shoot(){
        shooter = Shooter.getInstance();
        addRequirements(shooter);
        
    }

    @Override 
    public void initialize(){
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        shooter.setSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (Timer.getFPGATimestamp() - time >= 2){
            return true;
        }
        return false;
    }
}
