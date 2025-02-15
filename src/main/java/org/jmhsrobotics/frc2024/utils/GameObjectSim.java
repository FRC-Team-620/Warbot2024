package org.jmhsrobotics.frc2024.utils;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class GameObjectSim {
	public static final double fieldLength = Units.inchesToMeters(651.223);

	public static final class StagingLocations {
		public static final double centerlineX = fieldLength / 2.0;

		// need to update
		public static final double centerlineFirstY = Units.inchesToMeters(29.638);
		public static final double centerlineSeparationY = Units.inchesToMeters(66);
		public static final double spikeX = Units.inchesToMeters(114);
		// need
		public static final double spikeFirstY = Units.inchesToMeters(161.638);
		public static final double spikeSeparationY = Units.inchesToMeters(57);

		public static final Translation2d[] centerlineTranslations = new Translation2d[5];
		public static final Translation2d[] spikeTranslations = new Translation2d[3];

		static {
			for (int i = 0; i < centerlineTranslations.length; i++) {
				centerlineTranslations[i] = new Translation2d(centerlineX,
						centerlineFirstY + (i * centerlineSeparationY));
			}
		}

		static {
			for (int i = 0; i < spikeTranslations.length; i++) {
				spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
			}
		}
	}

	// private Pose3d[] objects = new Pose3d[8];
	public ArrayList<Pose3d> objects = new ArrayList<>();
	private int numInaked = 0;
	private boolean isIntaking = false;
	private Pose2d lastRobotpos = new Pose2d();

	public GameObjectSim() {
		this.reset();

	}
	public void reset() {
		objects.clear();
		numInaked = 0;
		isIntaking = false;
		lastRobotpos = new Pose2d();
		for (var pos : StagingLocations.spikeTranslations) {
			objects.add(
					new Pose3d(new Pose2d(pos, new Rotation2d())).plus(new Transform3d(0, 0, 0.1, new Rotation3d())));
		}
		for (var pos : StagingLocations.centerlineTranslations) {
			objects.add(
					new Pose3d(new Pose2d(pos, new Rotation2d())).plus(new Transform3d(0, 0, 0.1, new Rotation3d())));
		}
	}

	public void update(Pose2d robotpos) {
		Iterator<Pose3d> iterator = objects.iterator();
		while (iterator.hasNext()) {
			Pose3d object = iterator.next();
			if (canIntake(object, robotpos) && isIntaking) {
				numInaked++;
				iterator.remove(); // Safely remove the current object using iterator
			}
		}

		var out = objects.toArray(new Pose3d[objects.size() + numInaked]);
		for (int i = 0; i < numInaked; i++) {
			out[objects.size() + i] = new Pose3d(robotpos.getX(), robotpos.getY(), 1 + (i * 0.5), new Rotation3d());
		}

		lastRobotpos = robotpos;

	}

	private boolean canIntake(Pose3d pos, Pose2d robotpos) {
		var dist = pos.getTranslation().getDistance(new Pose3d(robotpos).getTranslation());
		return dist < 0.75;
	}

	public void setIntake(boolean isIntaking) {
		this.isIntaking = isIntaking;
	}

	public boolean hasObject() {
		return numInaked > 0;
	}

	public void fire() {
		if (numInaked > 0 && isIntaking) {
			var asdf = lastRobotpos.transformBy(new Transform2d(-2, 0, new Rotation2d()));
			objects.add(new Pose3d(asdf));
			numInaked--;
		}

	}
	public void preload() {
		numInaked++;
	}

}
