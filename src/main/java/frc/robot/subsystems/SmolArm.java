package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SmolArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmolArm extends SubsystemBase{
    private static SmolArm s_Arm;
    private final CANSparkMax s_Motor;
    private final RelativeEncoder s_Encoder; 

    private final Drivetrain mDrivetrain;
    private final PIDController s_PIDController;

    public SmolArm(){
      mDrivetrain = Drivetrain.getInstance();
      
      s_Motor = new CANSparkMax(SmolArmConstants.smolArm_MotorID, MotorType.kBrushless);
      s_Encoder = s_Motor.getEncoder();
      s_PIDController = new PIDController(SmolArmConstants.armSP, SmolArmConstants.armSI, SmolArmConstants.armSD);

      configSmallArm();
    }

    public void configSmallArm(){
        s_Motor.setSmartCurrentLimit(SmolArmConstants.smolArmCurrentLimit);
        s_Motor.setIdleMode(IdleMode.kBrake);
    }
    
    public double getEncoderRaw(){
        return s_Encoder.getPosition();
    }

    public double getEncoderAngle(){
        return getEncoderRaw()*SmolArmConstants.sEncoderCountToDegrees;
    }

    public void setSpeed(double speed){
        s_Motor.set(speed);
    }

    public void stop(){
        s_Motor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Smol Arm Encoder Raw", getEncoderRaw());
        
    }

    public static SmolArm getInstance(){
        if(s_Arm == null){
            s_Arm = new SmolArm();
        }
        return s_Arm;
    }

}