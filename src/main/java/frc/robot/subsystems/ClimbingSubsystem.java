package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Logging;

import static frc.robot.Constants.*;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimbingSubsystem extends SubsystemBase {

    // Motors
    private WPI_TalonFX extenderLeftMotor = new WPI_TalonFX(EXTENDER_LEFT_PORT, CANIVORE_NAME);
    private WPI_TalonFX extenderRightMotor = new WPI_TalonFX(EXTENDER_RIGHT_PORT, CANIVORE_NAME);
    private WPI_TalonFX rotatorLeftMotor = new WPI_TalonFX(ROTATOR_LEFT_PORT, CANIVORE_NAME);
    private WPI_TalonFX rotatorRightMotor = new WPI_TalonFX(ROTATOR_RIGHT_PORT, CANIVORE_NAME);
    private ShuffleboardTab tab = Shuffleboard.getTab("Climber Subsystem");

    // Limit Switches
    private DigitalInput rotatorLeftFrontLimit = new DigitalInput(ROTATOR_LEFT_FRONT_LIMIT_PORT);
    private DigitalInput rotatorRightFrontLimit = new DigitalInput(ROTATOR_RIGHT_FRONT_LIMIT_PORT);
    private DigitalInput rotatorRightBackLimit = new DigitalInput(ROTATOR_LEFT_BACK_LIMIT_PORT);
    private DigitalInput rotatorLeftBackLimit = new DigitalInput(ROTATOR_RIGHT_BACK_LIMIT_PORT);

    private Debouncer debouncerFrontLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerBackLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerFrontRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debounceBackRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    
    //Through bore encoders
    private DutyCycleEncoder leftEncoder = new DutyCycleEncoder(LEFT_ENCODER_PORT);
    private DutyCycleEncoder rightEncoder = new DutyCycleEncoder(RIGHT_ENCODER_PORT);

    private double leftRotLast = 0;
    private double rightRotLast = 0;


    public ClimbingSubsystem() {

        
        extenderLeftMotor.setInverted(EXTENDER_LEFT_INVERTED);
        extenderRightMotor.setInverted(EXTENDER_RIGHT_INVERTED);
        extenderLeftMotor.setNeutralMode(NeutralMode.Brake);
        extenderRightMotor.setNeutralMode(NeutralMode.Brake);
        rotatorLeftMotor.setNeutralMode(NeutralMode.Coast);
        rotatorRightMotor.setNeutralMode(NeutralMode.Coast);
        rotatorLeftMotor.setInverted(ROTATOR_LEFT_INVERTED);
        rotatorRightMotor.setInverted(ROTATOR_RIGHT_INVERTED);

        // rotatorLeftMotor.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(true, 231, 231, .001)
        // );
        // rotatorRightMotor.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(false, 231, 231, .001)
        // );
        // rotatorLeftMotor.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(true, 40, 40, .001)
        // );
        // rotatorRightMotor.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(false, 40, 40, .001)
        // );

        clearStickies();
        
        reset(true);
        if(Logging.climb) {
            log();
            
        }
        tab.addBoolean("back left limit", () -> isLeftBackAtLimit());
        tab.addBoolean("back right limit", () -> isRightBackAtLimit());
        tab.addBoolean("front left limit", () -> isLeftFrontAtLimit());
        tab.addBoolean("front right limit", () -> isRightFrontAtLimit());
        tab.addNumber("Left RPM", () -> rotatorLeftMotor.getSelectedSensorVelocity() * 10 / 60);
        tab.addNumber("Right RPM", () -> rotatorRightMotor.getSelectedSensorVelocity() * 10 / 60);
        tab.addNumber("Left stator Current", () -> rotatorLeftMotor.getStatorCurrent() * ROTATIONS_PER_CLICK * 10 * 60);
        tab.addNumber("Right stator Current", () -> rotatorRightMotor.getStatorCurrent() * ROTATIONS_PER_CLICK * 10 * 60);
        tab.add(this);
    }

    @Override
    public void periodic() {
    }

    public void log() {

        tab.addNumber("left Angle", () -> getLeftAngle());
        tab.addNumber("right Angle", () -> getRightAngle());

        tab.addNumber("left Angle offset", () -> getLeftAngleOffset());
        tab.addNumber("right Angle offset", () -> getRightAngleOffset());

        tab.addNumber("right Length", () -> getRightLength());
        tab.addNumber("left Length", () -> getLeftLength());
        
        //tab.addBoolean("Arms stalling?", () -> false);
        tab.add(this);
    }
    public boolean areMotorsStalling(){
        return isMotorStalling(extenderLeftMotor) || isMotorStalling(extenderRightMotor) || isMotorStalling(rotatorLeftMotor) || isMotorStalling(rotatorRightMotor);
    }
    

    // Setters

    
    /**
     * Resets rotation/extension to 0
     */
    public void reset(boolean resetAngle) {
        extenderLeftMotor.setSelectedSensorPosition(0);
        extenderRightMotor.setSelectedSensorPosition(0);
        if(resetAngle) {
            leftEncoder.reset();;
            rightEncoder.reset();
        }
    }
    
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
        rotatorLeftMotor.setVoltage(checkBoundsRotations(volts, getLeftAngle(), isLeftFrontAtLimit(), isLeftBackAtLimit()));
    }

    /**
     * @param volts Input voltage (will be clamped)
     * @param brakemode motors in brakemode, will try to stall motors in opposite direction if in brakemode
     */
    public void setLeftRotationVolts(double volts, boolean brakemode) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        if(brakemode) {
            volts -= MathUtil.clamp(rotatorLeftMotor.getSelectedSensorVelocity() * BRAKE_KP, -8, 8);
            SmartDashboard.putNumber("Left brake output", volts);
        }
        setLeftRotationVolts(volts);
    }

    /**
     * @param volts Input voltage (will be clamped)
     * @param brakemode motors in brakemode, will try to stall motors in opposite direction if in brakemode
     */
    public void setRightRotationVolts(double volts, boolean brakemode) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        if(brakemode) {
            volts -= MathUtil.clamp(rotatorRightMotor.getSelectedSensorVelocity() * BRAKE_KP, -8, 8);
            SmartDashboard.putNumber("Right brake output", volts);
            SmartDashboard.putNumber("Right vel", rotatorRightMotor.getSelectedSensorVelocity());
        }
        setRightRotationVolts(volts);

    }

    /**
     * @param volts Input voltage (will be clamped)
     */
    public void setRightRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorRightMotor.setVoltage(checkBoundsRotations(volts, getRightAngle(), isRightFrontAtLimit(), isRightBackAtLimit()));
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

    public void setTestExtenderPercent(double left, double right){
        extenderRightMotor.set(right);
        extenderLeftMotor.set(left);
    }

    public void setTestRotatorPercent(double left, double right) {
        rotatorLeftMotor.set(left);
        rotatorRightMotor.set(right);
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
            return voltage;

        }
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

    private double checkBoundsRotations(double voltage, double angle, boolean frontlimit, boolean backlimit) {
        // Negative voltage is rotate towards hard stop, positive voltage is rotate towards intake

        if ( // check rotate forward (not at limit & forward) or (below front limit & forward)
            ((!frontlimit && voltage > 0) && (angle <= ROTATOR_FRONT_LIMIT_DEG && voltage > 0)) ||
            ((angle >= ROTATOR_BACK_LIMIT_DEG && voltage < 0) && (!backlimit && voltage < 0))
            ) {
            return voltage;
        }
        return 0;
    }

    public double getRightRotateVelocity() {
        
        return rotatorRightMotor.getSelectedSensorVelocity() * DriveConstants.CLICKS_PER_ROT;
    }

    public double getLeftRotateVelocity() {
        return rotatorLeftMotor.getSelectedSensorVelocity() * DriveConstants.CLICKS_PER_ROT;
    }

    public double getRightLength() {
        return extenderRightMotor.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getLeftLength() {
        return extenderLeftMotor.getSelectedSensorPosition() * LENGTH_PER_CLICK;
    }

    public double getRightAngle() {
        return Math.IEEEremainder(rightEncoder.get() * 360, 180);
    }

    public double getLeftAngle() {
        return Math.IEEEremainder(-leftEncoder.get() * 360, 180);
    }
    public double getRightAngleOffset() {
        return rightEncoder.getPositionOffset() * 360;

    }

    public double getLeftAngleOffset() {
        return -leftEncoder.getPositionOffset() * 360;
    }

    public boolean isLeftFrontAtLimit() {
        return !debouncerFrontLeft.calculate(rotatorLeftFrontLimit.get());
    }

    public boolean isRightFrontAtLimit() {
        return !debouncerFrontRight.calculate(rotatorRightFrontLimit.get());
    }
    public boolean isRightBackAtLimit() {
        return !debounceBackRight.calculate(rotatorRightBackLimit.get());
    }
    public boolean isLeftBackAtLimit() {
        return !debouncerBackLeft.calculate(rotatorLeftBackLimit.get());
    }

    public void clearStickies() {
        extenderRightMotor.clearStickyFaults();
        extenderLeftMotor.clearStickyFaults();
        rotatorLeftMotor.clearStickyFaults();
        rotatorRightMotor.clearStickyFaults();

        extenderRightMotor.setNeutralMode(NeutralMode.Brake);
        extenderLeftMotor.setNeutralMode(NeutralMode.Brake);
        rotatorLeftMotor.setNeutralMode(NeutralMode.Coast);
        rotatorRightMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void disable() {
        setExtensionVolts(0);
        setRotationVolts(0);
    }
    
}
