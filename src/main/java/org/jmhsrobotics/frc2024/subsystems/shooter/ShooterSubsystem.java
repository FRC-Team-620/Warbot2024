package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.utils.SparkMaxConfigUtils;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private SparkMax topFlywheel = new SparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private SparkMax bottomFlywheel = new SparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);
	private SparkMaxConfig topFlywheelConfig = new SparkMaxConfig();
	private SparkMaxConfig bottomFlywheelConfig = new SparkMaxConfig();

	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	private BangBangController bangBangController;
	private double reference;
	private ControlType controlType = ControlType.BANG_BANG;

	private PIDController upperPID;
	private PIDController lowerPID;
	private SimpleMotorFeedforward upperFeedforward = new SimpleMotorFeedforward(0.10948, 0.019707, 0.0018555);
	private SimpleMotorFeedforward lowerFeedforward = new SimpleMotorFeedforward(0.10948, 0.019707, 0.0018555);

	public enum ControlType {
		BANG_BANG, VOLTAGE, PID
	};

	// private double speed;

	public ShooterSubsystem() {
		// Initializes motor(s)
		this.bangBangController = new BangBangController();
		this.bangBangController.setTolerance(200);

		this.upperPID = new PIDController(0.001, 0, 0);
		this.lowerPID = new PIDController(0.001, 0, 0);
		this.upperPID.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(200)); // WARNING: this value is in Rad/s
		this.lowerPID.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(200)); // WARNING: this value is in Rad/s

		initializeMotors();
		if (RobotBase.isSimulation()) {
			initSim();
		}

		// SmartDashboard.putData("ShooterUpperPID", this.upperPID);
		// SmartDashboard.putData("ShooterLowerPID", this.lowerPID);
	}

	@Override
	public void periodic() {
		switch (this.controlType) {
			case BANG_BANG :
				double outPut = this.bangBangController.calculate(this.getRPM(), this.reference);
				this.topFlywheel.set(outPut);
				this.bottomFlywheel.set(outPut);
				break;

			case VOLTAGE :
				this.setVoltages(this.reference, this.reference);
				break;
			case PID :
				double upperOutput = MathUtil
						.clamp(this.upperPID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.getTopRRPM()),
								Units.rotationsPerMinuteToRadiansPerSecond(this.reference)), -12, 12);
				double lowerOutput = MathUtil
						.clamp(this.lowerPID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.getBottomRRPM()),
								Units.rotationsPerMinuteToRadiansPerSecond(this.reference)), -12, 12);

				upperOutput += upperFeedforward.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.reference));
				lowerOutput += this.lowerFeedforward
						.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.reference));
				this.setVoltages(upperOutput, lowerOutput);
				break;

		}
		// log("controlType", this.controlType.toString());
		// log("reference", this.reference);
		// log("topFlywheelDutyCycle", topFlywheel.get());
		// log("topflywheelSpeed", getRPM());
		// log("bottomflywheelSpeed", getBottomRRPM());
		// log("AtGoal", this.atGoal());

	}

	private void setVoltages(double topVoltage, double bottomVoltage) {
		if (Robot.isSimulation()) { // TODO: Simplation Hack
			this.topFlywheel.set(topVoltage / 12.0);
			this.bottomFlywheel.set(bottomVoltage / 12.0);
		}
		this.topFlywheel.setVoltage(topVoltage);
		this.bottomFlywheel.setVoltage(bottomVoltage);
	}

	private double getBottomRRPM() {
		if (Robot.isSimulation()) {
			return simVelocity;
		}
		return bottomEncoder.getVelocity();
	}

	private double getTopRRPM() {
		if (Robot.isSimulation()) {
			return simVelocity;
		}
		return topEncoder.getVelocity();
	}

	public double getRPM() {
		return getTopRRPM();
	}

	public void set(double goal, ControlType controlType) {
		this.reference = goal;
		this.controlType = controlType;
	}

	public boolean atGoal() {
		if (controlType == ControlType.VOLTAGE) {
			return false;
		} else {
			return this.upperPID.atSetpoint() && this.lowerPID.atSetpoint();
		}
	}

	private void initializeMotors() {
		// this.topFlywheel.restoreFactoryDefaults();
		this.topFlywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).openLoopRampRate(0);
		this.bottomFlywheelConfig.encoder.uvwAverageDepth(2).uvwMeasurementPeriod(16);

		this.bottomFlywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).openLoopRampRate(0);
		this.bottomFlywheelConfig.encoder.uvwAverageDepth(2).uvwMeasurementPeriod(16);

		SparkMaxConfigUtils.applyConfigInFlight(topFlywheel, topFlywheelConfig);
		SparkMaxConfigUtils.applyConfigInFlight(bottomFlywheel, bottomFlywheelConfig);

		// this.topFlywheel.setIdleMode(IdleMode.kCoast);
		// this.topFlywheel.setSmartCurrentLimit(60);
		// this.topFlywheel.setOpenLoopRampRate(0);
		// this.topEncoder = topFlywheel.getEncoder();
		// this.topEncoder.setAverageDepth(2);
		// this.topEncoder.setMeasurementPeriod(16);

		// this.bottomFlywheel.restoreFactoryDefaults();
		// this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		// this.bottomFlywheel.setSmartCurrentLimit(60);
		// this.bottomFlywheel.setOpenLoopRampRate(0);
		// this.bottomEncoder = bottomFlywheel.getEncoder();
		// this.bottomEncoder.setAverageDepth(2);
		// this.bottomEncoder.setMeasurementPeriod(16);

		// this.bottomFlywheel.follow(topFlywheel);
	}

	private FlywheelSim flywheelSim;
	private RevEncoderSimWrapper encSim;
	private double simVelocity = 0;

	public void initSim() {
		// flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.002);
		encSim = RevEncoderSimWrapper.create(topFlywheel);
	}

	@Override
	public void simulationPeriodic() {
		double motorVolts = MathUtil.clamp(topFlywheel.get() * 12, -12, 12);

		flywheelSim.setInputVoltage(motorVolts);
		flywheelSim.update(Constants.ksimDtSec);
		simVelocity = flywheelSim.getAngularVelocityRPM();
		if (simVelocity > 1000) {
			Robot.objSim.fire();
		}
		encSim.setVelocity(simVelocity);
	}
}
