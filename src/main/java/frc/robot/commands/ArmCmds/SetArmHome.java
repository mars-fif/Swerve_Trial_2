package frc.robot.commands.ArmCmds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;

public class SetArmHome extends Command{
    
    private final Arm arm;
    
    public SetArmHome(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        //arm.setArmToPos(84);
        arm.setSetpoint(86);
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
