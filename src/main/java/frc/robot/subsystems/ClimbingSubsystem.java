package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimbingSubsystem extends SubsystemBase {

    // Motors
    private WPI_TalonFX extenderLeftMotor = new WPI_TalonFX(EXTENDER_LEFT_PORT, Constants.CANIVORE_NAME);
    private WPI_TalonFX extenderRightMotor = new WPI_TalonFX(EXTENDER_RIGHT_PORT, Constants.CANIVORE_NAME);
    private WPI_TalonFX rotatorLeftMotor = new WPI_TalonFX(ROTATOR_LEFT_PORT, Constants.CANIVORE_NAME);
    private WPI_TalonFX rotatorRightMotor = new WPI_TalonFX(ROTATOR_RIGHT_PORT, Constants.CANIVORE_NAME);

    private ShuffleboardTab tab = Shuffleboard.getTab("Climber Subsystem");

    // Limit Switches
    private DigitalInput rotatorLeftLimit = new DigitalInput(ROTATOR_LEFT_LIMIT_PORT);
    private DigitalInput rotatorRightLimit = new DigitalInput(ROTATOR_RIGHT_LIMIT_PORT);
    private Debouncer debouncerLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    
    private boolean autoClimbStarted = false;

    public ClimbingSubsystem() {
        extenderLeftMotor.setInverted(EXTENDER_LEFT_INVERTED);
        extenderRightMotor.setInverted(EXTENDER_RIGHT_INVERTED);
        rotatorLeftMotor.setInverted(ROTATOR_LEFT_INVERTED);
        rotatorRightMotor.setInverted(ROTATOR_RIGHT_INVERTED);

        extenderRightMotor.setNeutralMode(NeutralMode.Brake);
        extenderLeftMotor.setNeutralMode(NeutralMode.Brake);
        rotatorLeftMotor.setNeutralMode(NeutralMode.Brake);
        rotatorRightMotor.setNeutralMode(NeutralMode.Brake);
        log();

        extenderLeftMotor.clearStickyFaults();
        extenderRightMotor.clearStickyFaults();
        rotatorLeftMotor.clearStickyFaults();
        rotatorRightMotor.clearStickyFaults();

    }

    @Override
    public void periodic() {
    }

    public void log() {
        tab.addBoolean("left Limit Switch", () -> isLeftAtLimit());
        tab.addBoolean("right Limit Switch", () -> isRightAtLimit());

        tab.addNumber("left Angle", () -> getLeftAngle());
        tab.addNumber("right Angle", () -> getRightAngle());

        tab.addNumber("right Length", () -> getRightLength());
        tab.addNumber("left Length", () -> getLeftLength());

     

        tab.add(this);
    }

    

    // Setters
    
    /**
     * Sets both extension to the same voltage.
     * @param volts Input voltage (will be clamped)
     */
    public void setExtensionVolts(double volts){
        setLeftExtensionVolts(volts);
        setRightExtensionVolts(volts);
    }

    /**
     * Sets both rotation to the same voltage.
     * @param volts Input voltage (will be clamped)
     */
    public void setRotationVolts(double volts) {
        setLeftRotationVolts(volts);
        setRightRotationVolts(volts);
    }

    /**
     * @param volts Input voltage (will be clamped)
     */
    public void setLeftRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorLeftMotor.setVoltage(checkBoundsRotations(volts, getLeftAngle(), isLeftAtLimit()));
    }

    /**
     * @param volts Input voltage (will be clamped)
     */
    public void setRightRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorRightMotor.setVoltage(checkBoundsRotations(volts, getRightAngle(), isRightAtLimit()));
    }

    /**
     * @param volts Input voltage (will be clamped)
     */
    public void setRightExtensionVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_EXTENDER_VOLTS, MAX_EXTENDER_VOLTS);
        extenderRightMotor.setVoltage(checkBoundsExtensions(volts, getRightLength()));
    }

    /** 
     * @param volts Input voltage (will be clamped)
     */
    public void setLeftExtensionVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_EXTENDER_VOLTS, MAX_EXTENDER_VOLTS);
        extenderLeftMotor.setVoltage(checkBoundsExtensions(volts, getLeftLength()));
    }

    /**
     * Checks if the extender is at max positions, and which direction is is trying
     * to move
     * 
     * @param voltage
     * @param currentPos
     * @return limited voltage output to not hit end stops
     */
    private double checkBoundsExtensions(double voltage, double currentPos) {
        // Positive voltage is extend out, negative voltage is reel in
        
        if ((currentPos <= EXTENDER_TOP_LIMIT && voltage > 0) || // Current position below top 
                (currentPos >= EXTENDER_BOTTOM_LIMIT && voltage < 0)) {
            SmartDashboard.putBoolean("ElevatorBoundsTripped", true);
            return voltage;

        }
        SmartDashboard.putBoolean("ElevatorBoundsTripped", false);
        return 0;
    }
    
    // Getters

    /**
     * Checks if the left rotator is at max positions, and which direction it is
     * trying to move
     * 
     * @param voltage
     * @param angle   angle of target motor
     * @param limit   limit switch of given motor
     * @return limited voltage output to not hit end stops
     */

    private double checkBoundsRotations(double voltage, double angle, boolean limit) {
        // Negative voltage is rotate towards hard stop, positive voltage is rotate towards intake
        if ( // check rotate forward (not at limit & forward) or (below front limit & forward)
            ((!limit && voltage > 0) && (angle <= ROTATOR_FRONT_LIMIT_DEG && voltage > 0)) ||
            // check rotate back (above back limit & forward)
            (angle >= ROTATOR_BACK_LIMIT_DEG && voltage < 0)
            ) {
            return voltage;
        }
        return 0;
    }

    public boolean isAutoClimbStarted(){
        // TODO: This is a getter, should not be editing variables.
        // autoClimberStarted = !autoClimberStarted;
        return autoClimbStarted;

    }

    public double getRightLength() {
        return extenderRightMotor.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getLeftLength() {
        return extenderLeftMotor.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getRightAngle() {
        return GET_DEGREES_FROM_CLICKS(rotatorRightMotor.getSelectedSensorPosition());
    }

    public double getLeftAngle() {
        return GET_DEGREES_FROM_CLICKS(rotatorLeftMotor.getSelectedSensorPosition());
    }

    public boolean isLeftAtLimit() {
        return !debouncerLeft.calculate(rotatorLeftLimit.get());
    }

    public boolean isRightAtLimit() {
        return !debouncerRight.calculate(rotatorRightLimit.get());
    }
}
