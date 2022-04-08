// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ArmSpeeds {
    private double extenderMaxVelocity;
    private double extenderMaxAcceleration;
    private double rotatorMaxVelocity;
    private double rotatorMaxAcceleration;

    public ArmSpeeds(double rotatorMaxVelocity, double rotatorMaxAcceleration, double extenderMaxVelocity, double extenderMaxAcceleration){
        this.extenderMaxVelocity = extenderMaxVelocity;
        this.extenderMaxAcceleration = extenderMaxAcceleration;
        this.rotatorMaxVelocity = rotatorMaxVelocity;
        this.rotatorMaxAcceleration = rotatorMaxAcceleration;
    }
    
    public TrapezoidProfile.Constraints  getRotatorConstraints(){
        return new TrapezoidProfile.Constraints(rotatorMaxVelocity, rotatorMaxAcceleration);
    }
    public TrapezoidProfile.Constraints  getExtenderConstraints(){
        return new TrapezoidProfile.Constraints(extenderMaxVelocity, extenderMaxAcceleration);
    }
    public boolean isRotatorNeutral(){
        return rotatorMaxVelocity == 0;
    }
    public boolean isExtenderNeutral(){
        return extenderMaxVelocity == 0;
    }
}
