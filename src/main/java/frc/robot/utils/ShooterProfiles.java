// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterProfiles {
    private DoubleSupplier powerRPM;
    private DoubleSupplier aimRPM;
    private DoubleSupplier powerPercentage;
    private DoubleSupplier aimPercentage;

    public ShooterProfiles(DoubleSupplier powerRPM, DoubleSupplier aimRPM, DoubleSupplier powerPercentage, DoubleSupplier aimPercentage) {
        this.powerRPM = powerRPM;
        this.aimRPM = aimRPM;
        this.powerPercentage = powerPercentage;
        this.aimPercentage = aimPercentage;
    }


    public double getPowerRPM() {
        return this.powerRPM.getAsDouble();
    }

    public double getAimRPM() {
        return this.aimRPM.getAsDouble();
    }

    public double getPowerPercentage() {
        return this.powerPercentage.getAsDouble();
    }

    public double getAimPercentage() {
        return this.aimPercentage.getAsDouble();
    }
    
}
