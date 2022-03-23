// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.LimeLight;

/**
 * 
 * Toggles between the front and back cameras, and reverses the driving controls
 */
public class ToggleCamera extends CommandBase {
  BooleanConsumer isDriveReversed;

  /**
   * 
   * 
   * @param isDriveReversed use this to invert the drive when cameras are swapped
   *
   */
  public ToggleCamera(BooleanConsumer isDriveReversed) {
    this.isDriveReversed = isDriveReversed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimeLight.setStream(1);
    LimeLight.setPipeline(1);
    LimeLight.setCamMode(1);
    LimeLight.setLEDMode(3);
    isDriveReversed.accept(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimeLight.setStream(2);
    isDriveReversed.accept(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
