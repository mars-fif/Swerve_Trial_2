package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnTest extends Command{
    private Drivetrain drivetrain;

    public TurnTest(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.turnTest();
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
