package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimbingSubsystem extends SubsystemBase {

    // Motors
    private WPI_TalonFX extenderLeftMotor = new WPI_TalonFX(EXTENDER_LEFT_PORT);
    private WPI_TalonFX extenderRightMotor = new WPI_TalonFX(EXTENDER_RIGHT_PORT);
    private WPI_TalonFX rotatorLeftMotor = new WPI_TalonFX(ROTATOR_LEFT_PORT);
    private WPI_TalonFX rotatorRightMotor = new WPI_TalonFX(ROTATOR_RIGHT_PORT);

    private ShuffleboardTab tab = Shuffleboard.getTab("Climber Subsystem");

    // Limit Switches
    private DigitalInput rotatorLeftLimit = new DigitalInput(ROTATOR_LEFT_LIMIT_PORT);
    private DigitalInput rotatorRightLimit = new DigitalInput(ROTATOR_RIGHT_LIMIT_PORT);
    private Debouncer debouncerLeft = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private Debouncer debouncerRight = new Debouncer(LIMIT_SWITCH_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);

    // Encoders
    private Encoder rotatorLeftEncoder;
    private Encoder rotatorRightEncoder;

    // Control
    private SimpleMotorFeedforward extenderLeftFeedforward = new SimpleMotorFeedforward(EXTENDER_LEFT_KS,
            EXTENDER_LEFT_KV);
    private SimpleMotorFeedforward extenderRightFeedforward = new SimpleMotorFeedforward(EXTENDER_RIGHT_KS,
            EXTENDER_RIGHT_KV);
    private SimpleMotorFeedforward rotatorRightFeedforward = new SimpleMotorFeedforward(EXTENDER_RIGHT_KS,
            EXTENDER_RIGHT_KV);
    private SimpleMotorFeedforward rotatorLeftFeedforward = new SimpleMotorFeedforward(EXTENDER_RIGHT_KS,
            EXTENDER_RIGHT_KV);

    private PIDController holdElevationLeftPid = new PIDController(.1, 0, 0);
    private PIDController holdElevationRightPid = new PIDController(.1, 0, 0);

    public ClimbingSubsystem() {
        extenderLeftMotor.setInverted(EXTENDER_LEFT_INVERTED);
        extenderRightMotor.setInverted(EXTENDER_RIGHT_INVERTED);
        rotatorLeftMotor.setInverted(ROTATOR_LEFT_INVERTED);
        rotatorRightMotor.setInverted(ROTATOR_RIGHT_INVERTED);

        extenderRightMotor.setNeutralMode(NeutralMode.Brake);
        extenderLeftMotor.setNeutralMode(NeutralMode.Brake);
        rotatorLeftMotor.setNeutralMode(NeutralMode.Brake);
        rotatorRightMotor.setNeutralMode(NeutralMode.Brake);

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

    // Getters
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

    // Setters
    /**
     * @param velocity
     */
    public void setExtensionSpeed(double velocity) {
        setRightExtensionVolts(extenderRightFeedforward.calculate(velocity));
        setLeftExtensionVolts(extenderLeftFeedforward.calculate(velocity));
    }

    /**
     * @param velocity
     */
    public void setRotationSpeed(double velocity) {
        setRightRotationVolts(rotatorRightFeedforward.calculate(velocity));
        setLeftRotationVolts(rotatorLeftFeedforward.calculate(velocity));
    }

    /**
     * Clamps voltage
     * @param volts
     */
    public void setLeftRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorLeftMotor.setVoltage(checkBoundsRotations(volts, getLeftAngle()));
    }

    /**
     * Clamps voltage
     * @param volts
     */
    public void setRightRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorLeftMotor.setVoltage(checkBoundsRotations(volts, getRightAngle()));
    }

    public void setRightExtensionVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_EXTENDER_VOLTS, MAX_EXTENDER_VOLTS);
        extenderRightMotor.setVoltage(checkBoundsExtensions(volts, getRightLength()));
    }

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

        // TODO check if negative / positive is flipped
        // Negative is out, positive is in
        if ((currentPos >= EXTENDER_TOP_LIMIT && voltage < 0) ||
                (currentPos <= EXTENDER_BOTTOM_LIMIT && voltage > 0)) {
            return 0;

        }
        return voltage;
    }

    /**
     * Checks if the left rotator is at max positions, and which direction it is
     * trying to move
     * 
     * @param voltage
     * @param angle   angle of target motor
     * @return limited voltage output to not hit end stops
     */

    private double checkBoundsRotations(double voltage, double angle) {
        // TODO check if negative / positive is flipped
        // Negative is back, positive is forwards
        if (isLeftAtLimit() && voltage > 0 ||
                angle <= ROTATOR_BACK_LIMIT_DEG && voltage < 0 ||
                angle >= ROTATOR_FRONT_LIMIT_DEG && voltage > 0) {
            return 0;
        }
        return voltage;
    }
}
