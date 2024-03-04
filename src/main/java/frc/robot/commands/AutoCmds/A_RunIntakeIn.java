package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class A_RunIntakeIn extends Command{
    private Intake intake;
    //private static Timer timer;
    private static double time; 

    public A_RunIntakeIn(){
        intake = Intake.getInstance();
    }

    @Override 
    public void initialize(){
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        intake.setSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(.5);
    }

    @Override
    public boolean isFinished(){
        if (time == 1){
            return true;
        }
        return false;
    }
}
