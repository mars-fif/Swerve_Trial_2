package frc.robot.commands.ArmCmds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;

public class SetArmHigh extends Command{
    
    private final Arm arm;
    
    public SetArmHigh(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        arm.setSetpoint(140);
        arm.setArmToPos();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
