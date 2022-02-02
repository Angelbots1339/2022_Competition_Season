// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class ShooterProfiles {
    private double powerRPM;
    private double aimRPM;

    public ShooterProfiles(double powerRPM, double aimRPM) {
        this.powerRPM = powerRPM;
        this.aimRPM = aimRPM;
    }

    public void setPowerRPM(double RPM) {
        this.powerRPM = RPM;
    }

    public void setAimRPM(double RPM) {
        this.aimRPM = RPM;
    }

    public double getPowerRPM() {
        return this.powerRPM;
    }

    public double getAimRPM() {
        return this.aimRPM;
    }
    
}
