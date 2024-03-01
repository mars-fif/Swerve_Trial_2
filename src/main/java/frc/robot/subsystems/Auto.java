package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase{
    private static Auto auto;

    private final Drivetrain mDrivetrain;
    public Auto(){
      mDrivetrain = Drivetrain.getInstance();



    }

    public void configAuto(){
        
    }

    @Override
    public void periodic(){
        
    }

    public void resetAutoBuilderAndPaths(){

    }

    public static Auto getInstance(){
        if(auto == null){
            auto = new Auto();
        }
        return auto;
    }

}
