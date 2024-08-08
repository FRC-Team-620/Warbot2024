package org.jmhsrobotics.frc2024.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class VisionSubsystem extends SubsystemBase implements Logged {
	// load the apriltag field layout
	private AprilTagFieldLayout layout;

	// declare the camera
	private PhotonCamera cam;
	public PhotonCamera objectCamera;

	// get the camera position on the robot
	private Transform3d camOnRobot = new Transform3d(
			new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(0), Units.inchesToMeters(9.875)),
			new Rotation3d(0, Units.degreesToRadians(-20), (Math.PI)));

	// construct a photonPoseEstimator
	private PhotonPoseEstimator estimator;

	private DriveSubsystem drive;

	private PhotonPipelineResult objectResults;

	private double[] flucialIDs;
	List<PhotonTrackedTarget> targets;

	public VisionSubsystem(DriveSubsystem drive) {
		this.cam = new PhotonCamera("Sechenov");
		this.objectCamera = new PhotonCamera("Kirby");
		this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cam,
				this.camOnRobot);
		this.drive = drive;

		System.out.println(Filesystem.getDeployDirectory().getAbsolutePath());

		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			System.out.println(e);
			DriverStation.reportError("Fail to load the april tag map", e.getStackTrace());
		}

		estimator.setFieldTags(layout);

		// Simulation
		simulationInit();
	}

	@Override
	public void periodic() {

		PhotonPipelineResult results = this.cam.getLatestResult();

		targets = results.getTargets();

		SmartDashboard.putBoolean("Vision/isObjectCamConnected", this.objectCamera.isConnected());
		SmartDashboard.putBoolean("Vision/isConnected", this.cam.isConnected());
		int len = targets.size();
		Pose3d[] posList = new Pose3d[len];
		flucialIDs = new double[len];
		for (int i = 0; i < len; i++) {
			Pose3d robotPose3d = new Pose3d(this.drive.getPose());
			Transform3d targetTransFromCam = targets.get(i).getBestCameraToTarget();
			Pose3d outPutPose = robotPose3d.plus(camOnRobot.plus(targetTransFromCam));

			posList[i] = outPutPose;
			flucialIDs[i] = targets.get(i).getFiducialId();
		}

		SmartDashboard.putNumberArray("Vision/flucialIDs", flucialIDs);
		// SmartDashboard.putBooelean("hasTaget", this.);
		log("flucialIDs", flucialIDs);
		log("targets", posList);
		log("cameraTransform", camOnRobot); // TODO: only log once

		// Puting the estimated pose to the network table
		var estimatedPose = this.getEstimatedGlobalPose(this.drive.getPose());
		if (estimatedPose.isPresent()) {
			// NT4Util.putPose3d("Vision/EstimatedTarget",
			// estimatedPose.get().estimatedPose);
			log("postEstimation", estimatedPose.get().estimatedPose);
		}
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose) {
		this.estimator.setReferencePose(prevPose);
		return this.estimator.update();
	}
	public PhotonTrackedTarget getTarget(double fiducialID) {
		for (PhotonTrackedTarget i : targets) {
			if (i.getFiducialId() == fiducialID) {
				return i;
			}
		}
		return null;
	}

	public AprilTagFieldLayout getAprilTagLayout() {
		return this.layout;
	}

	public Transform3d targetToCamera(Transform3d target) {
		return camOnRobot.plus(target);
	}

	public Pose3d targetToField(Transform3d target, Pose2d robotPose) {
		return new Pose3d(robotPose).plus(camOnRobot.plus(target));
	}

	VisionSystemSim visionSim = new VisionSystemSim("main");
	VisionSystemSim objectVisionSim = new VisionSystemSim("gameobjects");
	TargetModel noteTarget = new TargetModel(Units.inchesToMeters(14));
	PhotonCameraSim cameraSim, objectCameraSim;

	private void simulationInit() {
		visionSim.addAprilTags(layout);
		SimCameraProperties cameraProp = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(this.cam, cameraProp);
		objectCameraSim = new PhotonCameraSim(this.objectCamera, cameraProp);
		objectVisionSim.addCamera(objectCameraSim, new Transform3d(0, 0, 0.3, new Rotation3d(0, 0, Math.PI)));
		// robot-to-camera transform.
		visionSim.addCamera(cameraSim, camOnRobot);
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		// cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
		// // Approximate detection noise with average and standard deviation error in
		// // pixels.
		// cameraProp.setCalibError(0.25, 0.08);
		// // Set the camera image capture framerate (Note: this is limited by robot
		// loop
		// // rate).
		cameraProp.setFPS(10);
		// // The average and standard deviation in milliseconds of image data latency.
		cameraProp.setAvgLatencyMs(50);
		cameraProp.setLatencyStdDevMs(5);
		// SmartDashboard.putData("VisionDebug", objectVisionSim.getDebugField());
		// objectCameraSim.enableRawStream(true);
		// objectCameraSim.enableProcessedStream(true);

		// // // Enable drawing a wireframe visualization of the field to the camera
		// // streams.
		// // // This is extremely resource-intensive and is disabled by default.
		// objectCameraSim.enableDrawWireframe(true);
	}

	@Override
	public void simulationPeriodic() {
		objectVisionSim.update(drive.simpos);
		objectVisionSim.clearVisionTargets();
		Robot.objSim.objects.forEach((Pose3d obj) -> {
			objectVisionSim.addVisionTargets(new VisionTargetSim(obj, noteTarget));
		});
		visionSim.update(drive.simpos); // Slight hack but fixes latency issue

	}
}
