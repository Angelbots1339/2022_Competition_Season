// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter.ReverseShoot;
import frc.robot.commands.shooter.ShootTimed;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class RejectBall extends CommandBase {

  private final LoaderSubsystem loaderSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private Timer shootTimer = new Timer();
  private boolean shooting = false;
  private Timer retractTimer = new Timer();
  private boolean retracting = false;

  /**
   * Checks if a ball is at the color sensor, what color it is, and what team the
   * robot is on, then runs the loader to eject any enemy balls
   * 
   * @param isTeamRed False if the team is Blue, True if it is Red
   */
  public RejectBall(LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem, boolean rejectEnabled) {
    this.loaderSubsystem = loaderSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(loaderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check ball color
    if (IntakeSubsystem.isBallLow() && !shooting
        // If ball is blue and we are red
        && (BLUE.colorMatch(IntakeSubsystem.getColorSensorRaw()) && RobotContainer.getTeamColor()) 
        // If ball is red and we are blue
        || (RED.colorMatch(IntakeSubsystem.getColorSensorRaw()) && !RobotContainer.getTeamColor())) { 
        // If ball is opponent color, reject it
        // Start shoot timer
          shootTimer.start();
          shooting = true;
    } /*else if (shootCommand.isScheduled() // Trying to eject wrong ball
    // If ball is blue and we are blue
    && (BLUE.colorMatch(intakeSubsystem.getColorSensorRaw()) && !isTeamRed.getAsBoolean()) 
    // If ball is red and we are red
    || (RED.colorMatch(intakeSubsystem.getColorSensorRaw()) && isTeamRed.getAsBoolean())) { 
      shootCommand.cancel();
    }*/

    if(shootTimer.get() > REJECT_TIME) {
      shooting = false;
      retracting = true;
      retractTimer.start();
    }

    if(retractTimer.get() > REVERSE_TIME) {
      retracting = false;
    }

    if(shooting) {
      shooterSubsystem.setAimWheelRPM(SHOOTER_PROFILE_REJECT.getAimRPM());
      shooterSubsystem.setPowerWheelRPM(SHOOTER_PROFILE_REJECT.getPowerRPM());
      loaderSubsystem.runLoader(MAX_LOADER_SPEED);
    } else if (retracting) {
      shooterSubsystem.setAimVolts(-2);
      shooterSubsystem.setPowerVolts(-2);
      loaderSubsystem.runLoader(-MAX_LOADER_SPEED);
    } else {
      shooterSubsystem.disable();
      loaderSubsystem.disable();
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
