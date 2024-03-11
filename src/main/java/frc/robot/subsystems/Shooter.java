package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;

public class Shooter extends SubsystemBase{
    private static Shooter shooter; 
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor; 
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    private final SparkPIDController shooterPIDLeft;
    private final SparkPIDController shooterPIDRight;
    
    public Shooter(){
        m_leftMotor = new CANSparkMax(ShooterConstants.shooter_leftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ShooterConstants.shooter_rightMotorID, MotorType.kBrushless); 
        m_rightEncoder = m_rightMotor.getEncoder();
        m_leftEncoder = m_leftMotor.getEncoder();

        shooterPIDLeft = m_leftMotor.getPIDController();
        shooterPIDRight =  m_rightMotor.getPIDController();
        shooterPIDLeft.setFeedbackDevice(m_leftEncoder);
        shooterPIDRight.setFeedbackDevice(m_rightEncoder);

        shooterPIDLeft.setP(0.04, 0);
        shooterPIDLeft.setI(0,0);
        shooterPIDLeft.setD(0,0);
        shooterPIDLeft.setFF(1, 0);
        shooterPIDLeft.setOutputRange(-1, 1, 0);

        shooterPIDRight.setP(0.04, 0);
        shooterPIDRight.setI(0,0);
        shooterPIDRight.setD(0,0);
        shooterPIDRight.setFF(1, 0);
        shooterPIDRight.setOutputRange(-1, 1, 0);

        config();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Right encoder", m_rightEncoder.getVelocity());
    }

    public static Shooter getInstance(){
        if(shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }

    public void config(){
        //left is negative, right positive
        m_leftMotor.setSmartCurrentLimit(ShooterConstants.kShootCurrentLimit);
        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_leftMotor.setInverted(true);

        m_rightMotor.setSmartCurrentLimit(ShooterConstants.kShootCurrentLimit);
        m_rightMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setInverted(false);

        m_rightMotor.burnFlash();
        m_leftMotor.burnFlash();
    }

    public void setSpeed(double speed){
        m_rightMotor.set(speed);
        m_leftMotor.set(speed);

        //setSpeed(MathUtil.clamp(armPosPID.calculate(getEncoderAngle(),setpoint), -.15, 0.25));
    }

    //Uses PID to set the shooter at a constant speed
    public void setSpeedConst(double desiredSpeed){
        //setSpeed(shooterPIDLeft.setReference(desiredSpeed, ControlType.kDutyCycle, 0, desiredSpeed, null));
    }


    public void stop(){
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

}
