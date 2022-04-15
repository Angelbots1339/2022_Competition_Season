// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ClimberConstants.*;


import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.utils.ArmSpeeds;

public class PIDArmsToSetpoints extends CommandBase {
  private ProfiledPIDController leftRotator;
  private ProfiledPIDController rightRotator;
  private ProfiledPIDController leftExtender;
  private ProfiledPIDController rightExtender;
  private ArmSpeeds armSpeeds;
  /** Creates a new PIDArmsToSetpoints. */
  
  public PIDArmsToSetpoints(ClimbingSubsystem climbingSubsystem,  double extension, double rotation, ArmSpeeds armSpeeds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSpeeds = armSpeeds;
    rightRotator = new ProfiledPIDController(ROTATOR_KP, ROTATOR_KI, ROTATOR_KD, armSpeeds.getRotatorConstraints());
    leftRotator = new ProfiledPIDController(ROTATOR_KP, ROTATOR_KI, ROTATOR_KD, armSpeeds.getRotatorConstraints());
 
    rightRotator.setGoal(rotation);
    leftRotator.setGoal(rotation);
    rightRotator.reset(new TrapezoidProfile.State());
    leftRotator.reset(new TrapezoidProfile.State());
    
    
    rightExtender = new ProfiledPIDController(EXTENDER_KP, EXTENDER_KI, EXTENDER_KD, armSpeeds.getExtenderConstraints());
    leftExtender = new ProfiledPIDController(EXTENDER_KP, EXTENDER_KI, EXTENDER_KD, armSpeeds.getExtenderConstraints());
    rightExtender.setGoal(extension);
    leftExtender.setGoal(extension);
    leftExtender.reset(new TrapezoidProfile.State());
    rightExtender.reset(new TrapezoidProfile.State());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(!armSpeeds.isExtenderNeutral()){
      
      //SmartDashboard.putNumber("leftExtender", leftExtender.calculate(climbingSubsystem.getLeftLength()));
      //SmartDashboard.putNumber("leftExtenderLength",climbingSubsystem.getLeftLength());
    

      rightExtender.setConstraints(armSpeeds.getExtenderConstraints());
      leftExtender.setConstraints(armSpeeds.getExtenderConstraints());

      // SmartDashboard.putNumber("rightExtender", rightExtender.calculate(climbingSubsystem.getRightLength()));
      // SmartDashboard.putNumber("rightLength", climbingSubsystem.getRightLength());
      // SmartDashboard.putBoolean("at setPoint", rightExtender.atSetpoint());
      // SmartDashboard.putBoolean("at goal", rightExtender.atGoal());
      // SmartDashboard.putNumber("Goal", rightExtender.getGoal().position);
      // SmartDashboard.putNumber("Setpoint", rightExtender.getSetpoint().position);

      //SmartDashboard.putBoolean("Test", new TrapezoidProfile.Constraints(10, 10) == armSpeeds.getExtenderConstraints());
      


      //climbingSubsystem.setLeftExtensionVolts(MathUtil.clamp(leftExtender.calculate(climbingSubsystem.getLeftLength()), -MAX_RETRACTION_VOLTS, MAX_EXTENSION_VOLTS));
      //climbingSubsystem.setRightExtensionVolts(MathUtil.clamp(rightExtender.calculate(climbingSubsystem.getRightLength()), MAX_EXTENSION_VOLTS, -MAX_RETRACTION_VOLTS));
    }

    if(!armSpeeds.isRotatorNeutral()){
      rightRotator.setConstraints(armSpeeds.getRotatorConstraints());
      leftRotator.setConstraints(armSpeeds.getRotatorConstraints());
      // SmartDashboard.putNumber("rightRotator", rightRotator.calculate(climbingSubsystem.getRightAngle()));
      // SmartDashboard.putNumber("leftRotator", leftRotator.calculate(climbingSubsystem.getLeftAngle()));
      
      //climbingSubsystem.setLeftRotationVolts(MathUtil.clamp(MAX_ROTATOR_VOLTS, -MAX_ROTATOR_VOLTS, leftRotator.calculate(climbingSubsystem.getLeftAngle())));
      //climbingSubsystem.setRightRotationVolts(MathUtil.clamp(MAX_ROTATOR_VOLTS, -MAX_ROTATOR_VOLTS, rightRotator.calculate(climbingSubsystem.getRightAngle())));
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return (armSpeeds.isExtenderNeutral() || (leftRotator.atGoal() && rightRotator.atGoal())) && (armSpeeds.isRotatorNeutral() || (rightExtender.atGoal() && leftExtender.atGoal()));
  }
}
