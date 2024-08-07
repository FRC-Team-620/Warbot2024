package org.jmhsrobotics.frc2024.subsystems.drive.commands.auto;

import javax.xml.crypto.dsig.Transform;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class NoteHunterCommand extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController thetaPID;
	private PIDController xPID;
	private PIDController yPID;

	private double xGoal;
	private double yGoal;
	private double thetaGoal;

	private Pose2d lastObject;

	public NoteHunterCommand(VisionSubsystem vision, DriveSubsystem drive) {
		this.drive = drive;
		this.vision = vision;

		this.xPID = new PIDController(0.1, 0, 0);
		this.yPID = new PIDController(0.1, 0, 0);
		this.thetaPID = new PIDController(0.0125, 0, 0.0001);

		this.xGoal = 1;
		this.yGoal = 1;
		this.thetaGoal = 175;

		// SmartDashboard.putData("xPID", this.xPID);
		// SmartDashboard.putData("yPID", this.yPID);
		SmartDashboard.putData("NoteHunter/thetaPID", this.thetaPID);
		addRequirements(this.drive);
	}
	@Override
	public void initialize() {
		// this.xPID.setSetpoint(this.xGoal);
		// this.xPID.setTolerance(3, 1);

		// this.yPID.reset();
		// this.yPID.setSetpoint(this.yGoal);
		// this.yPID.setTolerance(3, 1);

		// this.xPID.setSetpoint();
		this.thetaPID.reset();
		this.thetaPID.setSetpoint(0);
		this.thetaPID.setTolerance(3, 1);
		this.thetaPID.enableContinuousInput(-180, 180);

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		PhotonPipelineResult rawPiece = this.vision.objectCamera.getLatestResult();
		SmartDashboard.putBoolean("NoteHunter/hasTargets", rawPiece.hasTargets());
		PhotonTrackedTarget piece = rawPiece.getBestTarget();
		if (piece != null) {
			this.lastObject = this.vision.targetToField(piece.getBestCameraToTarget(), this.drive.getPose()).toPose2d();
		}if(lastObject != null){
			Transform2d trans = this.lastObject.minus(this.drive.getPose());
			double x = trans.getX();
			double y = trans.getY();
			double theta = Math.toDegrees(Math.atan2(trans.getY(), trans.getX()));

			SmartDashboard.putNumber("NoteHunter/currentTheta", theta);
			// var rawXOutput = this.xPID.calculate(x);
			// double xOutput = MathUtil.clamp(rawXOutput, -0.3, 0.3);

			// var rawYOutput = this.yPID.calculate(y);
			// double yOutPut = MathUtil.clamp(rawYOutput, -0.3, 0.3);

			var rawThetaOutput = this.thetaPID.calculate(theta);
			double thetaOutput = MathUtil.clamp(rawThetaOutput, -0.1, 0.1);

			SmartDashboard.putNumber("NoteHunter/thetaOutput", thetaOutput);
			SmartDashboard.putNumber("NoteHunter/goal", 0);
			this.drive.drive(0, 0, -thetaOutput, false, true);
		}
		else {
			this.drive.drive(0, 0, 0, false, true);
		}
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
