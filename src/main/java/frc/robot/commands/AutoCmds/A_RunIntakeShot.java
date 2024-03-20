package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class A_RunIntakeShot extends Command{
    private Intake intake;
    private static Timer timer;
    private static double startTime; 

    //private static ParallelDeadlineGroup deadlineGroup;

    public A_RunIntakeShot(){
        intake = Intake.getInstance();
        //addRequirements(intake);
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        //deadlineGroup.setDeadline(this);
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
        if (Timer.getFPGATimestamp() - startTime > 1.5){
            return true; 
        }
        return false;
    }
}
