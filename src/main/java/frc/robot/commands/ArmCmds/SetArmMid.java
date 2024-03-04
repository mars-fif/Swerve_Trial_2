package frc.robot.commands.ArmCmds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;

public class SetArmMid extends Command{
    //Prepares the arm position for the speaker 
    private final Arm arm;
    
    public SetArmMid(){
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
        return false;
    }
}
