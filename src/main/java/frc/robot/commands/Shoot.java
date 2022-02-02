// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShooterProfiles;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  private ShooterSubsystem shooterSubsystem;
  private ShooterProfiles shooterProfile;
  private XboxController joystick;
  public Shoot(ShooterSubsystem shooterSubsystem, ShooterProfiles shooterProfile, XboxController joystick) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooterProfile = shooterProfile;
    this.joystick = joystick;
    addRequirements(shooterSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setPowerWheelRPM(shooterProfile.getPowerRPM());
    shooterSubsystem.setAimWheelRPM(shooterProfile.getPowerRPM());
    joystick.setRumble( RumbleType.kRightRumble, (shooterSubsystem.getAtSetpoint())? 1.0 : 0.0);
    joystick.setRumble( RumbleType.kLeftRumble, (shooterSubsystem.getAtSetpoint())? 1.0 : 0.0);
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
