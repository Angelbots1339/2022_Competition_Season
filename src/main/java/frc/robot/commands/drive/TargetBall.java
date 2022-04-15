// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Targeting;

public class TargetBall extends CommandBase {
  /** Creates a new TargetBall. */
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier fwd;
  private DoubleSupplier rot;
  private PIDController pidController = new PIDController(TARGETING_KP, TARGETING_KI, TARGETING_KD);
  public TargetBall(DriveSubsystem driveSubsystem, DoubleSupplier fwd, DoubleSupplier rot) {
    this.driveSubsystem = driveSubsystem;
    this.fwd = fwd;
    this.rot = rot;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);//center pixel line
    pidController.setTolerance(TARGETING_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidRot = -MathUtil.clamp(pidController.calculate(Targeting.getTargetXOffset()), -.5, .5);
    driveSubsystem.arcadeDrive(fwd, () -> rot.getAsDouble() + pidRot);
    // SmartDashboard.putNumber("pidrot", pidRot);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
