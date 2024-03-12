package org.jmhsrobotics.frc2024.subsystems.intake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class IntakeSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax intakeMotor;

	private TimeOfFlight lowerSensor;
	private TimeOfFlight upperSensor;
	public IntakeSubsystem() {
		intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		this.lowerSensor = new TimeOfFlight(1);
		this.upperSensor = new TimeOfFlight(0);
		intakeMotor.setSmartCurrentLimit(35);

		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);

		// optimize Can Traffic
		// intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 0); // applied
		// output, faults, sticky faults, isfollower - 10ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200); // Velocity, temp, voltage,current - 20ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200); // Rel pos - 20ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog sensor volts, vel, acc - 50ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Alternate Encoder Vel/pos - 20ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Duty Cycle Absolute Encoder Position/
																			// angle - 200ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Duty Cycle Absolute Encoder Velocity/
																			// freequency - 200ms
		intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535); // I accumm

		if (RobotBase.isSimulation()) {
			simInit();
		}
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("intake/velocityRPM",
		// intakeMotor.getEncoder().getVelocity());
		// SmartDashboard.putNumber("intake/currentDrawAmps",
		// intakeMotor.getOutputCurrent());
		// SmartDashboard.putBoolean("Intake/highSwitchState",
		// this.highSwitch().isPressed());
		// SmartDashboard.putBoolean("Intake/lowSwitchState",
		// this.lowSwitch().isPressed());

		// SmartDashboard.putNumber("Intake/lowerSensorReading",
		// this.lowerSensor.getRange());
		// SmartDashboard.putNumber("Intake/upperSensorReading",
		// this.upperSensor.getRange());

		// SmartDashboard.putBoolean("intake/hasNote", this.hasNote());
		log("intakeDutyCycle", intakeMotor.get());
		log("hasNote", this.hasNote());
		log("noteTooHigh", this.noteTooHigh());
		log("lowerSensoDistance", this.lowerSensor.getRange());
		log("upperSensor", this.upperSensor.getRange());
	}

	public void set(double speed) {
		intakeMotor.set(-speed);
	}

	public TimeOfFlight lowerSensor() {
		return this.lowerSensor;
	}

	public TimeOfFlight upperSensor() {
		return this.upperSensor;
	}

	public SparkLimitSwitch lowSwitch() {
		return this.intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
	}

	public SparkLimitSwitch highSwitch() {
		return this.intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
	}

	public boolean hasNote() {
		return this.lowerSensor.getRange() < 100;
	}

	public boolean noteTooHigh() {
		return this.upperSensor.getRange() < 300;
	}
	private DIOSim intakeSwitchSim;
	private DCMotorSim intakeSim;
	private RevEncoderSimWrapper intakeEncSim;

	public void simInit() {
		intakeSwitchSim = new DIOSim(Constants.DIO.kIntakeSwitch);
		intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.3);
		intakeEncSim = RevEncoderSimWrapper.create(intakeMotor);
	}

	@Override
	public void simulationPeriodic() {
		double intakeVolts = MathUtil.clamp(intakeMotor.get() * 12, -12, 12);
		intakeSim.setInput(intakeVolts);
		intakeSim.update(Constants.ksimDtSec);
		intakeSwitchSim.setValue(true); // TODO placeholder.
		intakeEncSim.setDistance(intakeSim.getAngularPositionRotations());
		intakeEncSim.setVelocity(intakeSim.getAngularVelocityRPM());

	}
}
