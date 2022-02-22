// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterProfiles {
    private DoubleSupplier powerRPM;
    private DoubleSupplier aimRPM;

    public ShooterProfiles(DoubleSupplier powerRPM, DoubleSupplier aimRPM) {
        this.powerRPM = powerRPM;
        this.aimRPM = aimRPM;
    }

    public double getPowerRPM() {
        return this.powerRPM.getAsDouble();
    }

    public double getAimRPM() {
        return this.aimRPM.getAsDouble();
    }    
}
