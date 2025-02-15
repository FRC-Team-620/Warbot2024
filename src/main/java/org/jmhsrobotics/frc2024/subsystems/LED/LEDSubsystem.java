package org.jmhsrobotics.frc2024.subsystems.LED;

import org.jmhsrobotics.frc2024.Constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;

	private int rainBowPixelHue = 0;
	public LEDSubsystem() {
		if (RobotBase.isSimulation()) {
			initSim();
		}

		// TODO: find the real port
		this.led = new AddressableLED(Constants.LEDConstants.LEDPortID);

		// TODO: get the real length
		this.ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LEDLength);

		this.led.setLength(this.ledBuffer.getLength());
		this.led.setData(this.ledBuffer);

	}

	public void startLED() {
		this.led.start();
	}

	public void endLED() {
		this.led.stop();;
	}

	public void setRGB(int r, int g, int b) {
		for (int i = 0; i < this.ledBuffer.getLength(); i++) {
			this.ledBuffer.setRGB(i, r, g, b);
		}
		this.led.setData(this.ledBuffer);

	}

	public void setBlue() {
		this.setRGB(0, 255, 255);
	}

	public void setWhite() {
		this.setRGB(255, 255, 255);
	}

	public void setGreen() {
		this.setRGB(124, 252, 0);
	}

	public void setRed() {
		this.setRGB(255, 0, 0);
	}

	public void setYellow() {
		this.setRGB(255, 255, 0);
	}

	public void rainBow() {
		for (var i = 0; i < this.ledBuffer.getLength(); i++) {
			final var hue = (rainBowPixelHue + (i * 180 / this.ledBuffer.getLength())) % 180;
			this.ledBuffer.setHSV(i, hue, 225, 128);
		}
		this.led.setData(this.ledBuffer);
		rainBowPixelHue += Constants.LEDConstants.rainbowSpeed;

		rainBowPixelHue %= 180;
	}

	private AddressableLEDSim simLEDs;
	private void initSim() {
		simLEDs = new AddressableLEDSim(led);
		simLEDs.setLength(Constants.LEDConstants.LEDLength);

	}

	@Override
	public void simulationPeriodic() {
	}
}
