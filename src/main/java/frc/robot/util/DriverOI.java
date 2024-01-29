package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurnTest;
import frc.robot.commands.StopTest;
import frc.robot.commands.StraightenDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class DriverOI {
    public static DriverOI instance;

    private final Drivetrain drivetrain;

    private final XboxController controller = new XboxController(0);

    public enum DPadDirection{
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };

    public DriverOI(){
        drivetrain = Drivetrain.getInstance();

        configureController();
    }

    public void configureController(){
        Trigger xTrigger = new JoystickButton(controller, XboxController.Button.kX.value);
        xTrigger.onTrue(new SequentialCommandGroup(new StraightenDrivetrain()));

        Trigger aTrigger = new JoystickButton(controller, XboxController.Button.kA.value);
        aTrigger.onTrue(new SequentialCommandGroup(new TurnTest()));

        Trigger bTrigger = new JoystickButton(controller, XboxController.Button.kB.value);
        bTrigger.onTrue(new SequentialCommandGroup(new StopTest()));
    }

    public static DriverOI getInstance(){
        if(instance == null){
            instance = new DriverOI();
        }

        return instance;
    }
    
}
