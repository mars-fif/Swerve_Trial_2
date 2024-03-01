package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private static Shooter shooter; 
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor; 
    
    public Shooter(){
        m_leftMotor = new CANSparkMax(ShooterConstants.shooter_leftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ShooterConstants.shooter_rightMotorID, MotorType.kBrushless); 
        
        config();
    }

    @Override
    public void periodic(){

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
    }

    public void setSpeed(double speed){
        m_rightMotor.set(speed);
        m_leftMotor.set(speed);
    }

    public void stop(){
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

}
