package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimbingSubsystem extends SubsystemBase {
    private WPI_TalonFX extenderLeft = new WPI_TalonFX(EXTENDER_LEFT_PORT);
    private WPI_TalonFX extenderRight = new WPI_TalonFX(EXTENDER_RIGHT_PORT);
    private WPI_TalonFX rotatorLeft = new WPI_TalonFX(ROTATOR_LEFT_PORT);
    private WPI_TalonFX rotatorRight = new WPI_TalonFX(ROTATOR_RIGHT_PORT);

    private DigitalInput rotatorLeftLimit = new DigitalInput(ROTATOR_LEFT_LIMIT_PORT);
    private DigitalInput rotatorRightLimit = new DigitalInput(ROTATOR_RIGHT_LIMIT_PORT);
    private Debouncer debouncerLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    

    public ClimbingSubsystem() {
        extenderLeft.configFactoryDefault();
        extenderRight.configFactoryDefault();
        rotatorLeft.configFactoryDefault();
        rotatorRight.configFactoryDefault();
        extenderLeft.setInverted(EXTENDER_LEFT_INVERTED);
        extenderRight.setInverted(EXTENDER_RIGHT_INVERTED);
        rotatorLeft.setInverted(ROTATOR_LEFT_INVERTED);
        rotatorRight.setInverted(ROTATOR_RIGHT_INVERTED);
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
        return debouncerLeft.calculate(rotatorLeftLimit.get());
    }
    
    public boolean getRightRotatorLimit() {
        return debouncerRight.calculate(rotatorRightLimit.get());
    }

    // Setters

    /**
     * This is used for PID controlling the climber arm extension
     * 
     * @param left
     * @param right
     */
    public void setExtensionSpeed(double left, double right) {
        extenderRight.set(right);
        extenderLeft.set(left);
    }
    
    /**
     * This is used for setting the climbing arm extension speed
     * 
     * @param speed
     */
    public void setExtensionSpeed(DoubleSupplier speed) {
        extenderRight.set(speed.getAsDouble());
        extenderLeft.set(speed.getAsDouble());
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

    /**
     * This sets the speed of the motors rotating the climbing arm
     * 
     * @param left
     * @param right
     */
    public void setRotationSpeed(DoubleSupplier speed) {
        rotatorRight.set(speed.getAsDouble());
        rotatorLeft.set(speed.getAsDouble());
    }
   

   
}
