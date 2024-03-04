package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.io.Console;

public class A_SetArmSpeaker extends Command{
    private Arm arm; 

    public A_SetArmSpeaker(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        arm.setArmToPos(108);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        if (arm.armInPos()){
            return true;
        }
        return false;   
    }
}
