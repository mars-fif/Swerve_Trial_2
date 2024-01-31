package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.StopTest;
import frc.robot.commands.StraightenDrivetrain;
//import frc.robot.subsystems.Drivetrain;

public class DriverOI {
    public static DriverOI instance;

    //private final Drivetrain drivetrain;
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private final XboxController controller = new XboxController(0);


    public enum DPadDirection{
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };

    public DriverOI(){
        //drivetrain = Drivetrain.getInstance();

        configureController();
    }

    public void configureController(){
        Trigger xTrigger = new JoystickButton(controller, XboxController.Button.kX.value);
        xTrigger.onTrue(new SequentialCommandGroup(new StraightenDrivetrain()));

        //Trigger aTrigger = new JoystickButton(controller, XboxController.Button.kA.value);

        Trigger bTrigger = new JoystickButton(controller, XboxController.Button.kB.value);
        bTrigger.onTrue(new SequentialCommandGroup(new StopTest()));
    }

    public static DriverOI getInstance(){
        if(instance == null){
            instance = new DriverOI();
        }

        return instance;
    }

    public double getForward(){
        double input = controller.getRawAxis(XboxController.Axis.kLeftY.value);

        if(Math.abs(input) < 0.9){
            return input *=0.7777; // Why 0.7777?
        }else{
            return input = Math.pow(input, 3);
        }
    }

        public double getStrafe(){
        double input = controller.getRawAxis(XboxController.Axis.kLeftX.value);

        if(Math.abs(input) < 0.9){
            return input *=0.7777; // Why 0.7777?
        }else{
            return input = Math.pow(input, 3);
        }
    }

    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation;
        if (DriveConstants.kUseRateLimit) {
            combinedRotation = m_rotLimiter.calculate((rightRotation - leftRotation) / 2.0);
        } else {
            combinedRotation = (rightRotation - leftRotation) / 2.0;
        }

        return combinedRotation * 1.0 * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = controller.getRawAxis(XboxController.Axis.kRightX.value) * DriveConstants.kWheelBase;
        double rotY = controller.getRawAxis(XboxController.Axis.kRightY.value) * DriveConstants.kTrackWidth;

        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude){
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public Translation2d getSwerveTranslation(){
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded, ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        if(DriveConstants.kUseRateLimit){
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed,2)+Math.pow(ySpeed,2));

            double directionSlewRate;
            if(m_currentTranslationMag != 0.0){
                //directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            }
        }


        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        //double norm = next_translation.getNorm();
        
        Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
        Translation2d deadband_vector = fromPolar(deadband_direction, 0.1);

        double new_translation_x = next_translation.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
        double new_translation_y = next_translation.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());

        next_translation = new Translation2d(new_translation_x * 1.0 * DriveConstants.kRealMaxSpeedMPS,
                                             new_translation_y * 1.0 * DriveConstants.kRealMaxSpeedMPS);
                                             
        return next_translation;

    }
    
}
