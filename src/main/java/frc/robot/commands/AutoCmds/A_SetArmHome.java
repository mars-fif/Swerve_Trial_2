package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.io.Console;

public class A_SetArmHome extends Command{
    private Arm arm; 

    public A_SetArmHome(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        arm.setSetpoint(86);
        arm.setArmToPos();
    }

    @Override
    public void end(boolean interrupted){
        arm.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (arm.withinRange(arm.getEncoderAngle(), 86)){
            return true;
        }
        return false;    
    }
}
