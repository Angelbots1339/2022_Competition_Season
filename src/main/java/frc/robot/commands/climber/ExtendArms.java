// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

import static frc.robot.Constants.ClimberConstants.*;

public class ExtendArms extends CommandBase {
  private ClimbingSubsystem climbingSubsystem;
  private double velocity;
  private double setpoint;
  //private PIDController followerArmController;

  private double direction;
  /** Creates a new ExtendArms. */
  /**
   * 
   * @param climbingSubsystem
   * @param speed meters per second
   * @param setpoint meters, 0 is base
   */
  public ExtendArms(ClimbingSubsystem climbingSubsystem, double speed, double setpoint) {
    addRequirements(climbingSubsystem);
    this.climbingSubsystem = climbingSubsystem;
    // TODO check if negative / positive is flipped
    // Negative is out, positive is in
    if(climbingSubsystem.getRightLength() > setpoint) {
      this.velocity = -speed;
    } else {
      this.velocity = speed;
    }
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    climbingSubsystem.setExtensionSpeed(velocity, velocity);
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    regulateSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.setExtensionSpeed(0, 0);
  }

  /**
   * Checks if both arms have reached setpoints.
   * Will turn off arms that have reached setpoints.
   */
  private void regulateSpeeds() {
    climbingSubsystem.setExtensionSpeed(leftReachedSetpoint() ? 0 : velocity , rightReachedSetpoint() ? 0 : velocity);
  }


  private boolean leftReachedSetpoint() {
    // True if going out (false) and above setpoint (false)
    // True if going in (true) and below setpoint (true)
    return velocity > 0 == climbingSubsystem.getLeftLength() < setpoint;
  }

  private boolean rightReachedSetpoint() {
    return velocity > 0 == climbingSubsystem.getRightLength() < setpoint;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Going in? 
    return leftReachedSetpoint() && rightReachedSetpoint();
  }
}
