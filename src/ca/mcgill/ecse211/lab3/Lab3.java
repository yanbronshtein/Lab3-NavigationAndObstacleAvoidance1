package ca.mcgill.ecse211.lab3;

/**
 * <h1> Lab3 - Navigation And Obstacle Avoidance </h1>
 * In this Lab, two types of Navigation are implemented: 
 * Navigation with Obstacle Avoidance and Navigation without Obstacle Avoidance
 * 
 * @author Yaniv Bronshtein
 * @author Varad Kothari
 * @version 1.0
 *  */
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	/*US Sensor: Port S1 
	 *Medium Regulated Motor: Port B
	 *Left Motor: Port A
	 *Right Motor: Port D 
	 * */
	private static final Port sensorPort = LocalEV3.get().getPort("S1");
	private static final EV3MediumRegulatedMotor radialMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	// Robot Parameters
	public static final double TRACK = 14.3; 
	public static final double WHEEL_RAD = 2.1;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;

		@SuppressWarnings("resource")							    //Never close
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(sensorPort); 
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// for usSensor distance readings 
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null; 									// the selected controller on each cycle

		
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(t);

		do {
			t.clear();

			// ask the user whether the motors should drive with obstacles or without
			t.drawString("< Left    |     Right >", 0, 0);
			t.drawString("          |            ", 0, 1);
			t.drawString("  With    | Without    ", 0, 2);
			t.drawString("obstacles | obstacles  ", 0, 3);
			t.drawString("          |            ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);
		
		/* Start odometer, Display, and depending on button pressed, 
		 * navigation without obstacle avoidance, or navigation with obstacle avoidance */
		if (buttonChoice == Button.ID_LEFT) {
			NavigationUS navigationUS = new NavigationUS(leftMotor, rightMotor, radialMotor, odometer, TRACK, WHEEL_RAD); 
			usPoller = new UltrasonicPoller(usDistance, usData, navigationUS);
			
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			navigationUS.start();
			usPoller.start();

		} else {
			Navigation navigation = new Navigation(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD);
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