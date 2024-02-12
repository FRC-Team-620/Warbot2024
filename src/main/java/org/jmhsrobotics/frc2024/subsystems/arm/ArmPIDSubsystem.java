package org.jmhsrobotics.frc2024.subsystems.arm;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.nt.NT4Util;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPIDSubsystem extends SubsystemBase {

	private MechanismLigament2d m_arm;
	private CANSparkMax armPivot = new CANSparkMax(Constants.CAN.kArmPivotRightID, MotorType.kBrushless);
	private CANSparkMax armHelper = new CANSparkMax(Constants.CAN.kArmPivotFollowerID, MotorType.kBrushless);
	private SimableAbsoluteEncoder pitchEncoder;
	private Mechanism2d mech;
	private SparkLimitSwitch pitchSwitchF;
	private SparkLimitSwitch pitchSwitchR;
	// PID vars
	private double angle;
	private ProfiledPIDController armPID;

	public ArmPIDSubsystem() {

		armPivot.restoreFactoryDefaults();
		armHelper.restoreFactoryDefaults();
		// armHelper.setInverted(true);
		armPivot.setInverted(true);

		pitchEncoder = new SimableAbsoluteEncoder(armPivot.getAbsoluteEncoder(Type.kDutyCycle));
		armPivot.setSmartCurrentLimit(40);
		armPivot.setIdleMode(IdleMode.kBrake);
		armHelper.setIdleMode(IdleMode.kBrake);

		// 1 to 25 gearbox to a 9 tooth to 66 sprocket, times 360 degrees
		
		armPivot.getEncoder().setPositionConversionFactor(((1.0 / 25.0) * (9.0 / 66.0)) * 360.0);

		armHelper.follow(armPivot, true);
		pitchEncoder.setPositionConversionFactor(360);
		armPivot.getEncoder().setPosition(getArmPitch());

		armPivot.setSoftLimit(SoftLimitDirection.kReverse, 2);
		armPivot.setSoftLimit(SoftLimitDirection.kForward, 110);
		armPivot.enableSoftLimit(SoftLimitDirection.kForward, true);
		armPivot.enableSoftLimit(SoftLimitDirection.kReverse, true);

		// not yet on Robot (02/10/24)
		// pitchSwitchF =
		// armPivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		// pitchSwitchR =
		// armPivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		// pitchSwitchF.enableLimitSwitch(true);
		// pitchSwitchR.enableLimitSwitch(true);

		// armPivot.burnFlash();
		// armHelper.burnFlash();
		this.initPid();

		init2d();
		if (RobotBase.isSimulation()) {
			initSim();
		}
	}

	public void setGoal(double angle) {
		// this.armPID.setGoal(angle);
		this.angle = angle;
	}

	private void initPid() {
		// init PID Controller
		armPID = new ProfiledPIDController(0.02, 0, 0, new Constraints(180, 180));

		// reset PID state
		armPID.reset(new State(this.getArmPitch(), 0));

		// set goal to current goal on startup
		armPID.setGoal(this.angle);

		// set tolerances
		armPID.setTolerance(.5, 3);
	}

	private void setArmSpeed(double speed) {
		armPivot.set(speed);
		// SmartDashboard.putNumber("ArmSubsystem/data/ArmPivotSpeed", amount);
	}

	public double getArmPitch() {
		return this.pitchEncoder.getPosition();
	}

	public double getArmVelocity() {
		return this.pitchEncoder.getVelocity();
	}

	public boolean atGoal() {
		return this.armPID.atGoal();
	}

	public void init2d() {
		// TODO: finish sim
		mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("base", 1.5, 1.5);
		m_arm = root.append(new MechanismLigament2d("arm", 2, 0));
		// pitchencsim = new SparkAnalogSensor(pitchEncoder);

	}

	private void calculatePiditeration() {
		// set setpoint
		this.armPID.setGoal(this.angle);

		// calculate PID output
		double PIDOut = this.armPID.calculate(this.getArmPitch());

		// clamp output for max outs
		PIDOut = MathUtil.clamp(PIDOut, -.5, .5);

		// set arm speed with PID controller output
		this.setArmSpeed(PIDOut);

		// publish odometry
		SmartDashboard.putNumber("ArmCommand/data/goal", this.angle);
		SmartDashboard.putNumber("ArmCommand/data/setPoint", this.armPID.getSetpoint().position);
		SmartDashboard.putNumber("ArmCommand/data/PIDOut", PIDOut);
		SmartDashboard.putNumber("ArmCommand/data/Current", this.getArmPitch());
	}

	private void updateOdometry() {

		SmartDashboard.putData("ArmSubsystem/armSIM", mech);
		SmartDashboard.putNumber("ArmSubsystem/velocity", this.pitchEncoder.getVelocity());
		SmartDashboard.putNumber("ArmSubsystem/encoder", pitchEncoder.getPosition());
		SmartDashboard.putNumber("ArmSubsystem/relativeAngle", armPivot.getEncoder().getPosition());
	}

	@Override
	public void periodic() {
		this.calculatePiditeration();

		this.updateOdometry();

		m_arm.setAngle(getArmPitch());

		NT4Util.putPose3d("ArmSubsystem/armpose3d",
				new Pose3d(-0.213, 0, 0.286, new Rotation3d(0, -Units.degreesToRadians(getArmPitch()), 0)));

	}

	SingleJointedArmSim armSim;

	public void initSim() {
		double armGearRatio = 183.33;
		double moi = 0.399649199419221;
		armSim = new SingleJointedArmSim(DCMotor.getNEO(2), armGearRatio, moi, Units.inchesToMeters(23), 0,
				Units.degreesToRadians(180), true, 0);
		// simEncoder = new RevEncoderSimWrapper(null, null);
	}

	@Override
	public void simulationPeriodic() {
		double armVolts = MathUtil.clamp(armPivot.get() * 12, -12, 12);
		armSim.setInputVoltage(armVolts);
		armSim.update(Constants.ksimDtSec);
		pitchEncoder.setPosition(Units.radiansToDegrees(armSim.getAngleRads()));
	}

}
