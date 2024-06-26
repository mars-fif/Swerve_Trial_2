package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;

public class A_RunIntakeIn extends Command{
    private Intake intake;
    //private static Timer timer;
    //private static double time; 

    public A_RunIntakeIn(){
        intake = Intake.getInstance();
        //addRequirements(intake);
    }

    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        intake.setSpeed(1);
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (intake.noteReady()){
            return true; 
        }else{
            return false;
        }
    }
}
