// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ArmsToSetpoints extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private final double angleSetpoint;
  private final double lengthSetpoint;
  private boolean stopRoatator = false;
  private boolean stopExtender = false;

  /** Creates a new ExtendArms. */
  /**
   * 
   * @param climbingSubsystem
   * @param lengthSetpoint    // m
   * @param angleSetpoint     // degrees
   */
  public ArmsToSetpoints(ClimbingSubsystem climbingSubsystem, double lengthSetpoint, double angleSetpoint) {
    addRequirements(climbingSubsystem);

    this.climbingSubsystem = climbingSubsystem;
    this.angleSetpoint = angleSetpoint;
    this.lengthSetpoint = lengthSetpoint;
  }

  public ArmsToSetpoints(ClimbingSubsystem climbingSubsystem, double lengthSetpoint) {
    this(climbingSubsystem, lengthSetpoint, 0);
    stopRoatator = true;
  }

  public ArmsToSetpoints(double angleSetpoint, ClimbingSubsystem climbingSubsystem) {
    this(climbingSubsystem, 0, angleSetpoint);
    stopExtender = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stop individual movement once arms reach setpoints.
    // Restart movement if arms have moved out of setpoints.
    // Only call when changed to avoid high traffic.
    double leftExtendDesired = extenderDesiredOutput(lengthSetpoint, climbingSubsystem.getLeftLength());
    double rightExtendDesired = extenderDesiredOutput(lengthSetpoint, climbingSubsystem.getRightLength());
    double leftRotateDesired = rotatorDesiredOutput(angleSetpoint, climbingSubsystem.getLeftAngle());
    double rightRotateDesired = rotatorDesiredOutput(angleSetpoint, climbingSubsystem.getRightAngle());
    

    if (!stopRoatator) {
      climbingSubsystem.setLeftRotationVolts(leftRotateDesired);
      climbingSubsystem.setRightRotationVolts(rightRotateDesired);
    }
    if(!stopExtender) {
      climbingSubsystem.setLeftExtensionVolts(leftExtendDesired);
      climbingSubsystem.setRightExtensionVolts(rightExtendDesired);
    }
    SmartDashboard.putBoolean("left ROtator",
        isWithinThreshold(climbingSubsystem.getLeftAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD));
    SmartDashboard.putBoolean("right rotator",
        isWithinThreshold(climbingSubsystem.getRightAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD));
    SmartDashboard.putBoolean("left extender",
        isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD));
    SmartDashboard.putBoolean("right extender",
        isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD));
    SmartDashboard.putBoolean("StopExtender", stopExtender);
   
  }

  private double rotatorDesiredOutput(double setpoint, double position) {
    if (isWithinThreshold(setpoint, position, ROTATION_SETPOINT_THRESHOLD))
      return 0;
    return MAX_ROTATOR_VOLTS * Math.signum(setpoint - position);
  }

  private double extenderDesiredOutput(double setpoint, double position) {
    if (isWithinThreshold(setpoint, position, EXTENDER_SETPOINT_THRESHOLD))
      return 0;
    double maxVoltage;
    if (setpoint - position < 0) {
      maxVoltage = MAX_EXTENDER_VOLTS_RETRACT;
    } else {
      maxVoltage = MAX_EXTENDER_VOLTS;
    }
    return maxVoltage * Math.signum(setpoint - position);

    
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setRotationVolts(0);
    climbingSubsystem.setExtensionVolts(0);
  }

  private boolean isWithinThreshold(double setpoint, double position, double threshold) {
    return Math.abs(setpoint - position) < threshold;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (stopRoatator) {
      return (isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD)
          && isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD))
          //|| climbingSubsystem.areMotorsStalling();
          ;
    } else if(stopExtender) {
      return (isWithinThreshold(angleSetpoint, climbingSubsystem.getRightAngle(), ROTATION_SETPOINT_THRESHOLD)
          && isWithinThreshold(angleSetpoint, climbingSubsystem.getLeftAngle(), ROTATION_SETPOINT_THRESHOLD))
          //|| climbingSubsystem.areMotorsStalling();
          ;
    } else {
      return (isWithinThreshold(climbingSubsystem.getRightLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD)
          && isWithinThreshold(climbingSubsystem.getLeftLength(), lengthSetpoint, EXTENDER_SETPOINT_THRESHOLD)
          && isWithinThreshold(climbingSubsystem.getRightAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD)
          && isWithinThreshold(climbingSubsystem.getLeftAngle(), angleSetpoint, ROTATION_SETPOINT_THRESHOLD))
          //|| climbingSubsystem.areMotorsStalling();
          ;
    }

  }
}