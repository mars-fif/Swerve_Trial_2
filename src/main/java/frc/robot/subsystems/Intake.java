package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake intake; 
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor; 

    public Intake(){
        m_leftMotor = new CANSparkMax(IntakeConstants.intake_leftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(IntakeConstants.intake_rightMotorID, MotorType.kBrushless); 

        config();
    }

    @Override
    public void periodic(){

    }

    public static Intake getInstance(){
        if(intake == null){
            intake = new Intake();
        }
        return intake;
    }

    public void config(){
        //left is negative, right positive
        m_leftMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_leftMotor.setInverted(true);

        m_rightMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
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
