// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Candle;
import frc.robot.utils.ShooterProfiles;
import frc.robot.utils.Candle.LEDState;

import static frc.robot.Constants.LoaderConstants.*;

public class Shoot extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private LoaderSubsystem loaderSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterProfiles shooterProfiles;
  private boolean reject;

  /**
   * Revs the flywheels, and when they are at setpoint, it will feed the balls and
   * shoot
   * 
   * @param loaderSubsystem  pass in the intake subsystem
   * @param shooterSubsystem pass in the shooter subsystem
   * @param shooterProfile   pass in a shooter profile
   * @param reject Turn on/off ball rejection
   */
  public Shoot(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem,
      ShooterProfiles shooterProfile, boolean reject) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loaderSubsystem = loaderSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterProfiles = shooterProfile;
    this.intakeSubsystem = intakeSubsystem;
    this.reject = reject;
    addRequirements(loaderSubsystem, shooterSubsystem);
  }
  public Shoot(IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem,
      ShooterProfiles shooterProfile){
    this.loaderSubsystem = loaderSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterProfiles = shooterProfile;
    this.intakeSubsystem = intakeSubsystem;
    this.reject = false;
    addRequirements(loaderSubsystem, shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Candle.getInstance().changeLedState(LEDState.Fire);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If so, surround color sensor calls in try/catch loop
    // Check ball color and team color chosen
    if (
     // intakeSubsystem.isBallLow() && 
      reject
      // If ball is blue and we are red
      && ((BLUE.colorMatch(IntakeSubsystem.getColorSensorRaw()) && RobotContainer.getTeamColor()) 
      // If ball is red and we are blue
      || (RED.colorMatch(IntakeSubsystem.getColorSensorRaw()) && !RobotContainer.getTeamColor()))) { 
        // Reject current ball
        // If behavior is unexpected, try updating flywheel setpoints for a set
        // timer, then reverting back to previous setpoints.
        // Make sure to revert to old setpoints when command ends/ is cancelled.
        this.cancel();
        Candle.getInstance().changeLedState(LEDState.ReverseIntake);
        CommandScheduler.getInstance().schedule(
          new ShootTimed(intakeSubsystem, loaderSubsystem, shooterSubsystem, ShooterConstants.SHOOTER_PROFILE_REJECT, AutoConstants.SHOOT_TIME_1B));
    }


    shooterSubsystem.setPowerWheelRPM(shooterProfiles.getPowerRPM());
    shooterSubsystem.setAimWheelRPM(shooterProfiles.getAimRPM());

    if (shooterSubsystem.isAtSetpoint()) {
      loaderSubsystem.runLoader(MAX_LOADER_SPEED);
      intakeSubsystem.runIndexerLow(IntakeConstants.MAX_INDEXER_PERCENT);
    } 
    else{
      loaderSubsystem.runLoader(0);
      intakeSubsystem.runIndexerLow(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loaderSubsystem.disable();
    shooterSubsystem.disable();
    intakeSubsystem.disable();
    Candle.getInstance().changeLedState(LEDState.Idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
