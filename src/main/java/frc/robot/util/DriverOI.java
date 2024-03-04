package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.StopTest;
import frc.robot.commands.StraightenDrivetrain;
<<<<<<< Updated upstream
//import frc.robot.subsystems.Drivetrain;
=======
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SmolArm;
>>>>>>> Stashed changes

public class DriverOI {
    public static DriverOI instance;

<<<<<<< Updated upstream
    //private final Drivetrain drivetrain;
=======
    private final Drivetrain drivetrain;
    private final Intake intake; 

    private final SmolArm smolArm;

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        //drivetrain = Drivetrain.getInstance();
=======
        drivetrain = Drivetrain.getInstance();
        intake = Intake.getInstance();
        smolArm = SmolArm.getInstance();
>>>>>>> Stashed changes

        configureController();
    }

    public void configureController(){
        Trigger xTrigger = new JoystickButton(controller, XboxController.Button.kX.value);
        xTrigger.whileTrue(new SequentialCommandGroup(new StraightenDrivetrain()));

        /*
        Trigger bTrigger = new JoystickButton(controller, XboxController.Button.kB.value);
        bTrigger.whileTrue(new SequentialCommandGroup(new StopTest()));
<<<<<<< Updated upstream
=======
        */ 
        Trigger bTrigger = new JoystickButton(controller, XboxController.Button.kB.value);
        bTrigger.onTrue(new InstantCommand(()->drivetrain.resetGyro()));

        //Small arm 
        Trigger yTrigger = new JoystickButton(controller, XboxController.Button.kY.value);
        yTrigger.onTrue(new InstantCommand(()->smolArm.setSpeed(-0.5)))
        .onFalse(new InstantCommand(()->smolArm.stop()));

        Trigger aTrigger = new JoystickButton(controller, XboxController.Button.kA.value);
        aTrigger.onTrue(new InstantCommand(()->smolArm.setSpeed(0.5)))
        .onFalse(new InstantCommand(()->smolArm.stop()));

        Trigger leftTrigger = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        leftTrigger.onTrue(new InstantCommand(()->intake.setSpeed(.5)))
        .onFalse(new InstantCommand(()->intake.setSpeed(0)));

        Trigger rightTrigger = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        rightTrigger.onTrue(new InstantCommand(()->intake.setSpeed(-.5)))
        .onFalse(new InstantCommand(()->intake.setSpeed(0)));


        
>>>>>>> Stashed changes
    }

    public static DriverOI getInstance(){
        if(instance == null){
            instance = new DriverOI();
        }

        return instance;
    }

    public double getForward(){
        double input = -controller.getRawAxis(XboxController.Axis.kLeftY.value);

        if(Math.abs(input) < 0.9){
            return input *= 0.7777; // Why 0.7777?
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
        double leftRotation = controller.getRawAxis(XboxController.Axis.kLeftTrigger.value);
        double rightRotation = controller.getRawAxis(XboxController.Axis.kRightTrigger.value);

        double combinedRotation;
        if (DriveConstants.kUseRateLimit) {
            combinedRotation = m_rotLimiter.calculate((rightRotation - leftRotation) / 2.0);
        } else {
            combinedRotation = (rightRotation - leftRotation) / 2.0;
        }

        return combinedRotation * 1.0 * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = -controller.getRawAxis(4) * DriveConstants.kWheelBase;
        double rotY = controller.getRawAxis(5) * DriveConstants.kTrackWidth;

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
    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        }
        else if (_target < _current) {
            return _current - _stepsize;
        }
        else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) {
            return _target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
                return _target;
            }
            else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        }
        else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        }
        else {
            return _angle;
        }
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
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            }else{
                directionSlewRate = 500.0;
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir =StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
        }

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if(norm < 0.15){
            return new Translation2d();
        }else{
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, 0.15);

            double new_translation_x = next_translation.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(new_translation_x * 1.0 * DriveConstants.kRealMaxSpeedMPS,
                                                 new_translation_y * 1.0 * DriveConstants.kRealMaxSpeedMPS);
                                             
            return next_translation;
        }
    }
    
}
