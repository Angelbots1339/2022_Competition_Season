package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
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
     * @param volts
     */
    
    public void setExtensionSpeedSimpleVolts(double volts){
        SmartDashboard.putNumber("SimpleVoltInput", volts);
        setLeftExtensionVolts(volts);
        setRightExtensionVolts(volts);
    }
    public void setExtensionSpeedSimpleVolts(DoubleSupplier volts) {
        setExtensionSpeedSimpleVolts(volts.getAsDouble());
    }

    /**
     * @param volts
     */
    public void setRotationVolts(double volts) {
        setLeftRotationVolts(volts);
        setRightRotationVolts(volts);
    }

    /**
     * Clamps voltage
     * @param volts
     */
    public void setLeftRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorLeftMotor.setVoltage(checkBoundsRotations(volts, getLeftAngle(), isLeftAtLimit()));
    }

    /**
     * Clamps voltage
     * @param volts
     */
    public void setRightRotationVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_ROTATOR_VOLTS, MAX_ROTATOR_VOLTS);
        rotatorRightMotor.setVoltage(checkBoundsRotations(volts, getRightAngle(), isRightAtLimit()));
    }

    public void setRightExtensionVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_EXTENDER_VOLTS, MAX_EXTENDER_VOLTS);
        extenderRightMotor.setVoltage(checkBoundsExtensions(volts, getRightLength()));
    }

    public void setLeftExtensionVolts(double volts) {
        volts = MathUtil.clamp(volts, -MAX_EXTENDER_VOLTS, MAX_EXTENDER_VOLTS);
        SmartDashboard.putNumber("left Clamped Volts", volts);
        extenderLeftMotor.setVoltage(checkBoundsExtensions(volts, getLeftLength()));
        SmartDashboard.putNumber("left Bounded Volts", checkBoundsExtensions(volts, getLeftLength()));
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
}
