// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;


public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  DriveSubsystem driveSubsystem;
  private double angle;
  private PIDController turnController = new PIDController(0, 0, 0);
  

  public TurnToAngle(DriveSubsystem driveSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    this.angle = angle;
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(TURN_THRESHHOLD);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = MathUtil.clamp(turnController.calculate(driveSubsystem.getHeading().getDegrees(), angle), TURN_MAX_VOLTS, -TURN_MAX_VOLTS);
    driveSubsystem.tankDriveVolts(voltage, -voltage);


    // SmartDashboard.putNumber("voltage out", voltage);
    // SmartDashboard.putNumber("current angle", (driveSubsystem.getHeading().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turnController.atSetpoint() && turnController.getVelocityError() < TURN_VELOCITY_THRESHHOLD){ // Velocity less than 5 degrees per second
      driveSubsystem.disable();
      return true;
    }
    return false;
  }
}
