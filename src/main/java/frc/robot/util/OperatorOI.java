package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import frc.robot.commands.ArmCmds.ArmClimbPos;
import frc.robot.commands.ArmCmds.SetArmHigh;
import frc.robot.commands.ArmCmds.SetArmHome;
import frc.robot.commands.ArmCmds.SetArmMid;
import frc.robot.commands.ArmCmds.SetArmTrap;

public class OperatorOI {
    public static OperatorOI instance; 

    private final Shooter shooter;
    private final Arm arm;
    private final XboxController controller = new XboxController(1);

    public OperatorOI(){
        shooter = Shooter.getInstance();
        arm = Arm.getInstance();

        configureController();
    }

    public void configureController(){
        
        Trigger shootOutSlow = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        shootOutSlow.onTrue(new InstantCommand(()->shooter.setSpeed(-.30)))
        .onFalse(new InstantCommand(()->shooter.setSpeed(0)));
        
        //This is for shooting the game piece:
        Trigger shootOut = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        shootOut.onTrue(new InstantCommand(()->shooter.setSpeed(-.54)))
        .onFalse(new InstantCommand(()->shooter.setSpeed(0))); 

        //For bringing arm up and down 
        /*
        Trigger rightTrigger = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        rightTrigger.onTrue(new InstantCommand(()->arm.setSpeed(.25)))
        .onFalse(new InstantCommand(()-> arm.stop()));

        Trigger leftTrigger = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        leftTrigger.onTrue(new InstantCommand(()->arm.setSpeed(-.25)))
        .onFalse(new InstantCommand(()-> arm.stop()));
        */

        //Commands to set arm positions
        
         
        Trigger setArmHome = new JoystickButton(controller, XboxController.Button.kA.value);
        setArmHome.onTrue(new SetArmHome());
        
        Trigger setArmHigh = new JoystickButton(controller, XboxController.Button.kY.value);
        setArmHigh.onTrue(new SetArmHigh());

        Trigger setArmClimbPos = new JoystickButton(controller, XboxController.Button.kX.value);
        setArmClimbPos.onTrue(new SetArmMid());

        Trigger setArmTrapPose = new JoystickButton(controller, XboxController.Button.kB.value);
        setArmTrapPose.onTrue(new SetArmTrap());
    }

    public static OperatorOI getInstance(){
        if(instance == null){
            instance = new OperatorOI();
        }

        return instance;
    }

}
