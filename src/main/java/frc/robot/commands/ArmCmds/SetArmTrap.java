package frc.robot.commands.ArmCmds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;

public class SetArmTrap extends Command{
    //Prepares the arm position for the amp
    private final Arm arm;
    
    public SetArmTrap(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        arm.setArmToPos(94);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
