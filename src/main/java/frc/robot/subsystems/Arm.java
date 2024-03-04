package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

public class Arm extends SubsystemBase{
    //NEED VELOCITY PID TO SLOW THE ARM DOWN WHEN IT'S GOING DOWN! 
    private static Arm arm; 
    private final CANSparkMax m_leftMotor; 
    private final CANSparkMax m_rightMotor; 
    
    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;

    //private final PIDController armPosPID = new PIDController(ArmConstants.armPosPLow, ArmConstants.armPosI, ArmConstants.armPosD);
    private final PIDController armPosPID;
    //private final PIDController armVelocityPID = new PIDController(ArmConstants.armVP, ArmConstants.armVI, ArmConstants.armVD);

    public Arm(){
        m_leftMotor = new CANSparkMax(ArmConstants.arm_leftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ArmConstants.arm_rightMotorID, MotorType.kBrushless);

        //right: 1, left: 0
        leftEncoder = new DutyCycleEncoder(ArmConstants.leftEncoderDIO); //Left encoder is being used as a fallback
        rightEncoder = new DutyCycleEncoder(ArmConstants.rightEncoderDIO);
        armPosPID = new PIDController(0.0, ArmConstants.armPosI, 0.0);

        config();
    }

    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Left Arm Encoder Value", getEncoderAbsPos(leftEncoder));
        SmartDashboard.putNumber("Right Arm Encoder Value", getEncoderAbsPos(rightEncoder));

        SmartDashboard.putNumber("Get Right Encoder Angle", getEncoderAngle());
        SmartDashboard.putNumber("ArmPID P Value", armPosPID.getP());

        SmartDashboard.putBoolean("Arm in position", armInPos());
    }

    public static Arm getInstance(){
        if(arm == null){
            arm = new Arm();
        }
        return arm;
    }

    public void config(){
        m_leftMotor.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setInverted(true);

        m_rightMotor.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(false);
    }

    public void setSpeed(double speed){
        m_leftMotor.set(speed);
        m_rightMotor.set(speed);
    }

    public double getEncoderAbsPos(DutyCycleEncoder encoder){
        return encoder.getAbsolutePosition();
    }

    //... These functions aren't being used rn 
    public double getLeftEncoderAngle(){
        return getEncoderAbsPos(leftEncoder)/ArmConstants.leftEncMaxDegrees;
    }

    public double getRightEncoderAngle(){
        return getEncoderAbsPos(rightEncoder)/ArmConstants.rightEncMaxDegrees;
        
    }
    //...

    public double getEncoderAngle(){
        /*
        double leftEncoderAngle = getEncoderAbsPos(leftEncoder)/ArmConstants.leftEncToDegrees;
        double rightEncoderAngle = getEncoderAbsPos(rightEncoder)/ArmConstants.rightEncToDegrees;
        double tolerance = 0.1; //CHANGE THIS VALUE!
        if (Math.abs(leftEncoderAngle - rightEncoderAngle) < tolerance){
            return leftEncoderAngle;
        }else{
            return rightEncoderAngle;
        }
        */
        return getEncoderAbsPos(rightEncoder)*360;
    }

    public void setArmToPos(double setpoint){
        if (getEncoderAngle() < setpoint){
            armPosPID.setP(ArmConstants.armPosPHigh);
            armPosPID.setD(ArmConstants.armPosD);
        }else if (getEncoderAngle() > setpoint){
            armPosPID.setP(ArmConstants.armPosPLow);
        }
        setSpeed(MathUtil.clamp(armPosPID.calculate(getEncoderAngle(),setpoint), -.15, 0.25));
    }

    public boolean armInPos(){
        if (getEncoderAngle() > 108 - 3){
            return true;
        }
        return false;
    }

    public void stop(){
        m_leftMotor.set(0);
        m_rightMotor.set(0);
    }
    
}
