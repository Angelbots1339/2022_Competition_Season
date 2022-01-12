package frc.robot.utils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.AbstractMap;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ShuffleboardLogger {
	HashMap<Boolean, SubsystemLogger> Subsystems = new HashMap<Boolean, SubsystemLogger>();


	public void log(){
		for (Map.Entry<Boolean, SubsystemLogger> entry  : Subsystems.entrySet()) 
			entry = new AbstractMap.SimpleEntry<Boolean,SubsystemLogger>(SmartDashboard.getBoolean(entry.getValue().subsystem, false), entry.getValue());
		
			for (Map.Entry<Boolean, SubsystemLogger> entry : Subsystems.entrySet()) {
				if(!entry.getKey()) break;
				ShuffleboardTab tab = Shuffleboard.getTab(entry.getValue().getSubsystem());
				entry.getValue().boolLog.forEach((String key, Boolean value) -> tab.addBoolean(key, () -> value));
				entry.getValue().numberLog.forEach((String key, Number value) -> tab.addNumber(key, () -> (double)value));
				
		}

	}
	public void	addSubsystem(SubsystemLogger logger, boolean defaultEnabled){
		Subsystems.put(defaultEnabled, logger);
	}
	public void addSubsystem(SubsystemLogger logger){
		Subsystems.put(false, logger);
	}




	public static class SubsystemLogger{
		private String subsystem;
		public HashMap<String, Number> numberLog = new HashMap<String, Number>();
		public HashMap<String, Boolean> boolLog = new HashMap<String, Boolean>();
		public void logNumber(String key, Number value){
			numberLog.putIfAbsent(key, value);
			numberLog.replace(key, value);
		}
		public void logBool(String key,Boolean value){
			boolLog.putIfAbsent(key, value);
			boolLog.replace(key, value);
		}
		public String getSubsystem(){
			return subsystem;
		}

		public SubsystemLogger(String subsystem) {
			this.subsystem = subsystem;
		}
	}

}

