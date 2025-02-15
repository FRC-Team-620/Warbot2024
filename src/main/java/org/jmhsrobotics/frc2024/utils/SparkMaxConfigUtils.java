package org.jmhsrobotics.frc2024.utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxConfigUtils {
	public static void applyConfigInFlight(SparkMax motor, SparkMaxConfig config) {
		motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}
}
