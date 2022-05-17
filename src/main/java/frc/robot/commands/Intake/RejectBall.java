// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;


import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class RejectBall extends CommandBase {

  private final LoaderSubsystem loaderSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private Timer shootTimer = new Timer();
  private boolean finishedShooting = false;
  private boolean retracting = false;
  private RawColor ballColor;
  private boolean reject = false;
  

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
    reject = false;
    retracting = false;
    finishedShooting = false;
    ballColor = IntakeSubsystem.getColorSensorRaw();
    shootTimer.start(); 
    if(IntakeSubsystem.isBallLow()) {
      // if we are red & ball is blue
      if((RobotContainer.getTeamColor() && BLUE.colorMatch(ballColor)) ||
      // if we are blue & ball is red
      (!RobotContainer.getTeamColor() && RED.colorMatch(ballColor))) {
        shootTimer = new Timer();
        shootTimer.start();
        reject = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isBallLow && !shooting
    //     // If ball is blue and we are red
    //     && (RobotContainer.getTeamColor() && BLUE.colorMatch(IntakeSubsystem.getColorSensorRaw())) 
    //     // If ball is red and we are blue
    //     || (!RobotContainer.getTeamColor() && RED.colorMatch(IntakeSubsystem.getColorSensorRaw()))) { 
    //     // If ball is opponent color, reject it
    //     // Start shoot timer
    //       shootTimer = new Timer();
    //       shootTimer.start();
    //       shooting = true;
    // }

    if(!finishedShooting && shootTimer.get() > REJECT_TIME_BLUE) {
      finishedShooting = true;
      retracting = true;
    }
    
    if(retracting && shootTimer.get() > REVERSE_TIME + REJECT_TIME_BLUE) {
      retracting = false;
    }

    if(!finishedShooting && reject) {
      shooterSubsystem.setAimWheelRPM(SHOOTER_PROFILE_REJECT.getAimRPM());
      shooterSubsystem.setPowerWheelRPM(SHOOTER_PROFILE_REJECT.getPowerRPM());
      loaderSubsystem.runLoader(MAX_LOADER_SPEED);
    } else {
      shooterSubsystem.setAimVolts(-2);
      shooterSubsystem.setPowerVolts(-2);
      loaderSubsystem.runLoader(-MAX_LOADER_SPEED);
    }

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loaderSubsystem.disable();
    shooterSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!reject) || (finishedShooting && !retracting);
  }
}

/**
 * // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter.ReverseShoot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Logging;

import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class RejectBall extends CommandBase {

  private final LoaderSubsystem loaderSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private Timer shootTimer = new Timer();
  private boolean shooting = false;
  private Timer retractTimer = new Timer();
  private boolean retracting = false;
  private boolean ballColorRead = false;
  private boolean isBallRed = false;
  private boolean isBallBlue = false;

  /**
   * Checks if a ball is at the color sensor, what color it is, and what team the
   * robot is on, then runs the loader to eject any enemy balls
   * 
   * @param isTeamRed False if the team is Blue, True if it is Red
   *
  public RejectBall(LoaderSubsystem loaderSubsystem, ShooterSubsystem shooterSubsystem, boolean rejectEnabled) {
    this.loaderSubsystem = loaderSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(loaderSubsystem);
    //this.shootCommand = new ShootTimed(intakeSubsystem, loaderSubsystem, shooterSubsystem, SHOOTER_PROFILE_LOW, REJECT_TIME);
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
        && ((isBallBlue && RobotContainer.getTeamColor()) 
        // If ball is red and we are blue
        || (isBallRed && !RobotContainer.getTeamColor()))) { 
        // If ball is opponent color, reject it
        // Start shoot timer
          shootTimer = new Timer();
          shootTimer.start();
          shooting = true;
    }

    if(shootTimer.get() > (RobotContainer.getTeamColor() ? REJECT_TIME_BLUE : REJECT_TIME_RED) && shooting) {
      shooting = false;
      retractTimer = new Timer();
      retracting = true;
      retractTimer.start();
    }

    if(retractTimer.get() > REVERSE_TIME && retracting) {
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

    if(Logging.loader) {
      SmartDashboard.putBoolean("Shooting", shooting);
      SmartDashboard.putBoolean("Retracting", retracting);
      SmartDashboard.putBoolean("Ball Color Read", ballColorRead);
      SmartDashboard.putBoolean("isBallBlue", isBallBlue);
      SmartDashboard.putBoolean("isBallRed", isBallRed);
      SmartDashboard.putNumber("retractTimer", retractTimer.get());
      SmartDashboard.putNumber("shootTimer", shootTimer.get());

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

 */