package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.Anomath;

public class Mk4TTBSwerve{
    private final CANSparkMax m_turningSparkMax;
    private final SparkAnalogSensor m_turningEncoder;
    private final SparkPIDController m_turningPIDController;

    private int moduleNum;

    //private final AnalogEncoder m_analogEncoder;
    

    // private final PIDController m_turningPIDController;

    private SwerveModuleState state;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double m_angleOffset = 0.0;

    public Mk4TTBSwerve(int moduleNum, SwerveModuleConstants constants){
        this.moduleNum = moduleNum;

        m_turningSparkMax = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
        m_angleOffset = constants.angleOffset.getRadians();
        m_turningEncoder = m_turningSparkMax.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        m_turningPIDController = m_turningSparkMax.getPIDController();
        configTurningSpark();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = 0.0;
        correctedDesiredState.angle = desiredState.angle.plus(new Rotation2d(m_angleOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        
        m_desiredState = desiredState;
    }



    public SwerveModuleState getDesiredState(){
        return state;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(0.0, new Rotation2d(m_turningEncoder.getPosition()-m_angleOffset));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(0, new Rotation2d(m_turningEncoder.getPosition()-m_angleOffset));
    }

    public void stop(){
        m_turningSparkMax.set(0.0);
    }

    // public double getAngleDeg(){
    //     return getAngleRad()*(360.0/2*Math.PI);
    // }

    // public double getAngleRad(){
    //     return m_analogEncoder.getAbsolutePosition()*2*Math.PI;
    // }

    public int getModuleNumber(){
        return this.moduleNum;
    }

    public void configTurningSpark(){
        m_turningSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.setIdleMode(IdleMode.kBrake);
        m_turningSparkMax.setSmartCurrentLimit(40);
        m_turningEncoder.setInverted(true);
        m_turningEncoder.setPositionConversionFactor((2*Math.PI)/3.3);
        m_turningEncoder.setVelocityConversionFactor(((2*Math.PI)/3.3)/60);
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(0);
        m_turningPIDController.setPositionPIDWrappingMaxInput(2*Math.PI);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);
        m_turningPIDController.setP(1.0);
        m_turningPIDController.setI(0);
        m_turningPIDController.setD(0);
        m_turningPIDController.setFF(0);
        m_turningPIDController.setOutputRange(-1, 1);
        m_turningSparkMax.setClosedLoopRampRate(0.05);

        m_turningSparkMax.burnFlash();
    }

    public void putSmartDashboard(){
        SmartDashboard.putNumber(this.moduleNum + " Actual Angle", m_turningEncoder.getPosition());
        SmartDashboard.putNumber(this.moduleNum + " Mod. Offset", m_angleOffset);
        SmartDashboard.putNumber(this.moduleNum + " M Angle", m_turningEncoder.getPosition()-m_angleOffset);
        SmartDashboard.putNumber(this.moduleNum + " Set Point", m_desiredState.angle.getRadians());
    }
}
