package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
/**
 * <h1> Lab3 - Navigation And Obstacle Avoidance </h1>
 * This class implements motion with obstacle avoidance with the aid of an EV3 Ultrasonic Motor on a rotating axle
 * 
 * @author Yaniv Bronshtein
 * @author Varad Kothari
 * @version 1.0
 *  */
public class NavigationUS extends Thread implements UltrasonicController {

	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor; 
	private static EV3MediumRegulatedMotor sensorMotor;
	private final double WHEEL_RAD, TRACK;
	private final int MOTOR_ACCELERATION = 200;
	private final double TILEDIMENSION = 30.48;


	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;
	private static boolean isNavigating = true;
	private static double navigatingX, navigatingY;

	// variables to store sensor data
	private static int distance; 
	private int filterControl;

	// wall follower variables
	private static final int motorLow = 50, motorHigh = 200, bandCenter = 10, bandwidth = 3, FILTER_OUT = 20;

	// obstacle avoidance variables
	private double initialAngleAtBlock;
	private static boolean hasBlockPassed = false;
	private static boolean isPassingBlock = false;
	
	//to print distance
	
	

	public NavigationUS(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,  EV3MediumRegulatedMotor sensorMotor,
			Odometer odometer, double TRACK, double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
	}

	/**
	 * Reads our sensor distance
	 */
	public int readUSDistance() {
		return this.distance;
	}
	


	/**
	 * Processes our sensor data and acts accordingly based on its readings
	 */
	@Override
	public void processUSData(int distance) {
		double[] odoData = odometer.getXYT();
		double odoReadingsX, odoReadingsY, odoReadingsT;
		odoReadingsX = odoData[0];
		odoReadingsY = odoData[1];
		odoReadingsT = odoData[2];
		// filter out invalid samples of data
		filterData(distance);
		System.out.println("SensorDist is " + distance);

		// will be true if we are in the process of getting around obstacle
		if ( !isNavigating() && !hasBlockPassed ) {
			// this means we have passed our object, continue to destination
			if (initialAngleAtBlock - odoReadingsT >= 90) {
				hasBlockPassed = true;
				leftMotor.stop(true);
				rightMotor.stop(true);
				//reset motors
				for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
					motor.stop();
					motor.setAcceleration(MOTOR_ACCELERATION);
				}
				if ( navigatingX == 0 && navigatingY == 2 * TILEDIMENSION) {
					travelTo(0,2 * TILEDIMENSION);
				}
				travelTo(2 * TILEDIMENSION,0);
				return;
			}
			// otherwise execute our wall follow logic
			excecuteWallFollow();
		} else {
			if ( distance < 18 ) {
				leftMotor.stop(true);
				rightMotor.stop(true);
				initialAngleAtBlock = odoReadingsT;
				prepareForWallFollower();
			}
		}


	}

	/**
	 * Our main run method
	 * 
	 */
	public void run() {
		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(MOTOR_ACCELERATION);
		}
		// travel to coordinates
		travelTo(0, 2 * TILEDIMENSION);
		if (!isPassingBlock) {
			travelTo(2 * TILEDIMENSION, 0);
		}
	}

	/**
	 * Determine how much the motor must rotate for vehicle to reach a certain distance
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Determine the angle our motors need to rotate in order for vehicle to turn a certain angle 
	 * 
	 * @param radius
	 * @param TRACK
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

	/**
	 * A method to drive our vehicle to a certain cartesian coordinate
	 * 
	 * @param x X-Coordinate
	 * @param y Y-Coordinate
	 */
	private void travelTo(double x, double y) {

		double[] odoData; //Store XYT as outlined in OdometerData.java
		odoData = odometer.getXYT();
		double odoReadingsX, odoReadingsY, odoReadingsT;
		odoReadingsX = odoData[0];
		odoReadingsY = odoData[1];
		odoReadingsT = odoData[2];
		isNavigating = true;
		double deltaX = x - odoReadingsX;
		double deltaY = y - odoReadingsY;

		// calculate the minimum angle
		double minAngle = Math.toDegrees(Math.atan2( deltaX, deltaY)) - odoReadingsT;

		// turn to the minimum angle
		turnTo(minAngle);

		// calculate the distance to next point
		double distance  = Math.hypot(deltaX, deltaY);

		// move to the next point
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD,distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
		isNavigating = false;
	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	private void turnTo(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		if(theta < -180) { // if angle is negative, turn to the left
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta + 360), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta + 360), false);
		} 

		else if (theta > 180) { // angle is positive, turn to the right
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta - 360), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta -360), false);
		}
		else {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}

	}

	/**
	 * A method to determine whether another thread has called travelTo and turnTo methods or not
	 * 
	 * @return
	 */
	private boolean isNavigating() {
		return isNavigating; 
	}


	/**
	 * A method to filter out invalid samples of data taken from PController filter in lab 1
	 * @param distance
	 * @returns
	 */
	private void filterData(int distance) {
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
	}

	/**
	 * A method to that implements our wall following logic
	 */
	private void excecuteWallFollow() {
		// calculate our offset from the bandCenter
//		int error = distance - bandCenter - 5; // -5 for distance from sensor to side of vehicle
		int error = distance - bandCenter + 5; // -5 for distance from sensor to side of vehicle

		// Keep moving forward if vehicle is within threshold value
		if ( Math.abs(error) < this.bandwidth ) {
			goStraight();
		} 
		// We are too close to the wall, steer vehicle to the right
		else if ( error < 0 ) {
			turnRight(); 
		} 
		// We are too far away from the wall
		else { 
			if ( error > 100 ) {
				// It is just sensing something very far away, keep going straight
				goStraight(); 
			} else {
				// We are too far from the wall, steer left
				turnLeft();
			}
		}
	}


	/**
	 * A method to turn our vehicle
	 */
	private void prepareForWallFollower() {
		isNavigating = false;
		isPassingBlock = true;
		turnTo(90); // turn our angle 90 degrees
		sensorMotor.setSpeed(100);					
		sensorMotor.rotate(-100); // turn our sensor toward the wall
	}

	/**
	 * Method to steer the vehicle in a straight forward direction
	 */
	public void goStraight() {
		leftMotor.setSpeed(motorHigh);			
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Method to steer the vehicle right
	 */
	public void turnRight() {
		leftMotor.setSpeed(motorHigh);			
		rightMotor.setSpeed(motorLow);
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Method to steer the vehicle left
	 */
	public void turnLeft() {
		leftMotor.setSpeed(motorLow);			
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}


}