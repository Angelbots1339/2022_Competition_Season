package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimbingSubsystem extends SubsystemBase {
    private WPI_TalonFX extenderLeft = new WPI_TalonFX(EXTENDER_LEFT_PORT);
    private WPI_TalonFX extenderRight = new WPI_TalonFX(EXTENDER_RIGHT_PORT);
    private WPI_TalonFX rotatorLeft = new WPI_TalonFX(ROTATOR_LEFT_PORT);
    private WPI_TalonFX rotatorRight = new WPI_TalonFX(ROTATOR_RIGHT_PORT);


    private ShuffleboardTab tab = Shuffleboard.getTab("Climber Subsystem");

    private DigitalInput rotatorLeftLimit = new DigitalInput(ROTATOR_LEFT_LIMIT_PORT);
    private DigitalInput rotatorRightLimit = new DigitalInput(ROTATOR_RIGHT_LIMIT_PORT);
    private Debouncer debouncerLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
   
    private SimpleMotorFeedforward extenderLeftFeedforward;
    private SimpleMotorFeedforward extenderRightFeedforward;


    private PIDController leftExtenderPID = new PIDController(.1, 0, 0);
    private PIDController leftRotatorPID = new PIDController(.001, 0, 0);
    private PIDController rightExtenderPID = new PIDController(.1, 0, 0);
    private PIDController rightRotatorPID = new PIDController(.001, 0, 0);

    public ClimbingSubsystem() {
        // extenderLeft.configFactoryDefault();
        // extenderRight.configFactoryDefault();
        // rotatorLeft.configFactoryDefault();
        // rotatorRight.configFactoryDefault();
        extenderLeft.setInverted(EXTENDER_LEFT_INVERTED);
        extenderRight.setInverted(EXTENDER_RIGHT_INVERTED);
        rotatorLeft.setInverted(ROTATOR_LEFT_INVERTED);
        rotatorRight.setInverted(ROTATOR_RIGHT_INVERTED);

        extenderRight.setNeutralMode(NeutralMode.Brake);
        extenderLeft.setNeutralMode(NeutralMode.Brake);
        rotatorLeft.setNeutralMode(NeutralMode.Brake);
        rotatorRight.setNeutralMode(NeutralMode.Brake);

        tab.addBoolean("left", () -> getLeftRotatorLimit());
        tab.addBoolean("right", () -> getRightRotatorLimit());
        tab.addNumber("left Angle", () -> getLeftAngle());
        tab.addNumber("right Angle", () -> getRightAngle());

        tab.addNumber("right Length", () -> getRightLength());
        tab.addNumber("left Length", () -> getLeftLength());
        tab.add(this);
        tab.add(leftExtenderPID);
        
        leftRotatorPID.enableContinuousInput(-180, 180);
        rightRotatorPID.enableContinuousInput(-180, 180);

        extenderLeftFeedforward = new SimpleMotorFeedforward(EXTENDER_LEFT_KS, EXTENDER_LEFT_KV);
        extenderRightFeedforward = new SimpleMotorFeedforward(EXTENDER_RIGHT_KS, EXTENDER_RIGHT_KV);

    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("left angle", getLeftAngle());
        SmartDashboard.putNumber("right angle", getRightAngle());
    }

    // Getters
    
    
    public double getRightLength() {
        return extenderRight.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getLeftLength() {
        return extenderLeft.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getRightAngle() {
        return GET_DEGREES_FROM_CLICKS(rotatorRight.getSelectedSensorPosition());
    }

    public double getLeftAngle() {
        return GET_DEGREES_FROM_CLICKS(rotatorLeft.getSelectedSensorPosition());
    }
    public boolean getLeftRotatorLimit() {
        return !debouncerLeft.calculate(rotatorLeftLimit.get());
    }
    
    public boolean getRightRotatorLimit() {
        return !debouncerRight.calculate(rotatorRightLimit.get());
    }

    // Setters

    /**
     * This is used for PID controlling the climber arm extension
     * 
     * @param left in meters per second
     * @param right in meters per second
     */
    public void setExtensionSpeed(double left, double right) {
        setExtensionSpeedRight(right); 
        setExtensionSpeedLeft(left);

    }
    /**
     * 
     * @param right in meters per second
     */
    public void setExtensionSpeedRight(double right) {
        extenderRight.set(checkBoundsExtensions(right, extenderRight.getSelectedSensorPosition() * LENGTH_PER_CLICK) ? right * ROTATOR_PERCENT_MAX : 0);
        
    }
    /**
     * 
     * @param left in meters per second
     */
    public void setExtensionSpeedLeft(double left) {
        extenderLeft.set(checkBoundsExtensions(left, extenderLeft.getSelectedSensorPosition() * LENGTH_PER_CLICK) ? left * ROTATOR_PERCENT_MAX : 0);
    }
    


    /**
     * This is used for PID of the climbing arm rotation
     * 
     * 
     * @param left
     * @param right
     */

    public void setRotationSpeed(double left, double right) {
        rotatorRight.set(right);
        rotatorLeft.set(left);
    }

    public void setLeftRotationSpeed(double left) {
        rotatorLeft.set(checkLeftBoundsRotations(left) ? left * ROTATOR_PERCENT_MAX : 0);
    }

    public void setRightRotationSpeed(double right) {
        rotatorRight.set(checkRightBoundsRotations(right) ? right * ROTATOR_PERCENT_MAX : 0);
    }


    /**
     * Checks if the extender is a
     * 
     * t max positions, and which direction is is trying to move
     * 
     * @param speed
     * @param currentPos
     * @return Returns false if extender is not allowed to move, returns true if extender is allowed to move
     */
    private boolean checkBoundsExtensions(double speed, double currentPos){
        if(LENGTH_PER_CLICK * currentPos >= SLACK_LENGTH_METERS && speed > 0){
            return false;

        } else if(currentPos <= 0 && speed < 0){
            return false;
    
        } 

        return true;
    } 


    /**
     * Checks if the left rotator is at max positions, and which direction it is trying to move
     * 
     * @param speed
     * @return Returns false if the left rotator is not allowed to move, returns true if it is allowed to move
     */

    private boolean checkLeftBoundsRotations(double speed){
    
        if(getLeftRotatorLimit() && speed < 0){
            return false;

        } else if(GET_DEGREES_FROM_CLICKS(rotatorLeft.getSelectedSensorPosition()) <= ROTATOR_BACK_LIMIT_DEG && speed > 0){
            return false;

        } else {
            return true;
        }
    }

    /**
     * Checks if the right rotator is at max positions, and which direction it is trying to move
     * 
     * @param speed
     * @return Returns false if the right rotator is not allowed to move, returns true if it is allowed to move
     */

    private boolean checkRightBoundsRotations(double speed) {

        if (getRightRotatorLimit() && speed < 0) {
            return false;

        } else if (GET_DEGREES_FROM_CLICKS(rotatorRight.getSelectedSensorPosition()) <= ROTATOR_BACK_LIMIT_DEG
                && speed > 0) {
            return false;

        } else {
            return true;
        }
    }

    public boolean checkInThreshold(DoubleSupplier input) {
        return Math.abs(input.getAsDouble()) < JoystickConstants.JOYSTICK_THRESHOLD;
    }

    private double leftExtenderMemory;
    private double rightExtenderMemory;
    private double leftRotatorMemory;
    private double rightRotatorMemory;

    private boolean leftExtenderFlag = false;
    private boolean leftRotatorFlag = false;
    private boolean rightExtenderFlag = false;
    private boolean rightRotatorFlag = false;

    

    /**
     * Used for setting all of the speeds for the robot climber
     * 
     * @param leftExtender Speed of the left extender
     * @param rightExtender Speed of the right extender
     * @param leftRot Speed of the left rotator
     * @param rightRot Speed of the right rotator
     */

    public void setClimberSpeeds(DoubleSupplier leftExtender, DoubleSupplier rightExtender, DoubleSupplier leftRot, DoubleSupplier rightRot){

        if(checkInThreshold(leftExtender)) {
            if(!leftExtenderFlag) {
                leftExtenderMemory = getLeftLength();
                leftExtenderFlag = true;
            }
            extenderLeft.set(leftExtenderPID.calculate(leftExtenderMemory));
        } else {

            leftExtenderFlag = false;
            extenderLeft.set(leftExtender.getAsDouble() * EXTENDER_PERCENT_MAX);

        }
        // if(checkInThreshold(leftRot)) {
            
        //     if(!leftRotatorFlag) {
        //         leftRotatorMemory = getLeftAngle();
        //         leftRotatorFlag = true;
        //     }
        //     rotatorLeft.set(leftRotatorPID.calculate(-leftRotatorMemory));
        //     SmartDashboard.putNumber("PID", leftRotatorPID.calculate(leftRotatorMemory));
        // } else {

        //     leftRotatorFlag = false;
        //     rotatorLeft.set(leftRot.getAsDouble() * ROTATOR_PERCENT_MAX);

        // }
        // SmartDashboard.putBoolean("Check in treshhold", checkInThreshold(leftExtender));
        if(checkInThreshold(rightRot)) {
            
            if(!rightRotatorFlag) {
                rightRotatorMemory = getLeftAngle();
                rightRotatorFlag = true;
            }
            //rotatorLeft.set(leftRotatorPID.calculate(-leftRotatorMemory));
            SmartDashboard.putNumber("LeftRotPID", rightRotatorPID.calculate(-rightRotatorMemory));
        } else {

            rightRotatorFlag = false;
            SmartDashboard.putNumber("LeftRotNoPID", rightRot.getAsDouble() * ROTATOR_PERCENT_MAX);
            //rotatorLeft.set(leftRot.getAsDouble() * ROTATOR_PERCENT_MAX);

        }
        SmartDashboard.putBoolean("Check in treshhold", checkInThreshold(leftExtender));


        //setRightRotationSpeed(rightRot.getAsDouble());
        //setLeftRotationSpeed(leftRot.getAsDouble());
        //rotatorLeft.set(leftRot.getAsDouble() * ROTATOR_PERCENT_MAX);
        rotatorRight.set(rightRot.getAsDouble() * ROTATOR_PERCENT_MAX);
        //setExtensionSpeedRight(rightExtender.getAsDouble());
        //setExtensionSpeedLeft(leftExtender.getAsDouble());
        extenderRight.set(rightExtender.getAsDouble() * EXTENDER_PERCENT_MAX);
        
    }

   
   

   
}
