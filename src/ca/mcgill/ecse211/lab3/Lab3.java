package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Left motor connected to output A
	// Right motor connected to output D
	private static final Port sensorPort = LocalEV3.get().getPort("S1");
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	// Characteristics of our vehicle
	public static final double TRACK = 14.3; //How much I turn
	public static final double WHEEL_RAD = 2.1;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;

		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(sensorPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle

		// some objects that need to be instantiated
		final TextLCD t = LocalEV3.get().getTextLCD();
//		Odometer odometer = new Odometer(leftMotor, rightMotor,TRACK, WHEEL_RAD);
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(t);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left    |     Right >", 0, 0);
			t.drawString("          |            ", 0, 1);
			t.drawString("  With    | Without    ", 0, 2);
			t.drawString("obstacles | obstacles  ", 0, 3);
			t.drawString("          |            ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			NavigationUS navigationUS = new NavigationUS(leftMotor, rightMotor, sensorMotor, odometer, TRACK, WHEEL_RAD);
			usPoller = new UltrasonicPoller(usDistance, usData, navigationUS);
			//start our odometer
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			navigationUS.start();
			usPoller.start();

		} else {
			Navigation navigation = new Navigation(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD);
			//start our odometer
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			navigation.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}