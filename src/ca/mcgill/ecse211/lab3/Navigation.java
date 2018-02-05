package ca.mcgill.ecse211.lab3;


/**
 * <h1> Lab3 - Navigation And Obstacle Avoidance </h1>
 * This class implements motion without obstacle avoidance simply based on the internal workings of brick
 * 
 * @author Yaniv Bronshtein
 * @author Varad Kothari
 * @version 1.0
 *  */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
	 
//	private static OdometerData odoReadings  ;

	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final double WHEEL_RAD, TRACK;
	private final int MOTOR_ACCELERATION = 200;
	
	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;
	private final double TILEDIMENSION = 30.48;

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, double TRACK, double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
	}
	
	/**
	 * run method where coordinate map defined
	 */
	public void run() {
		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(MOTOR_ACCELERATION);
		}
		// travel to coordinates
		travelTo( 2 * TILEDIMENSION, TILEDIMENSION); //(2,1)
		travelTo(TILEDIMENSION, TILEDIMENSION); //(1,1)
		travelTo(TILEDIMENSION,2 * TILEDIMENSION); //(1,2)
		travelTo(2 * TILEDIMENSION , 0);
	
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
	 * Determine the angle wheel motors need to rotate to for robot to turn to desired angle 
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
	 * Method to 
	 * 
	 * @param x X-Coordinate
	 * @param y Y-Coordinate
	 */
	private void travelTo(double x, double y) {
		
		double odoReadingsX, odoReadingsY, odoReadingsT, deltaX, deltaY, minAngle, distance;
		double[] odoData; //to store Odometer Readings for x, y, and theta
		odoData = odometer.getXYT();
		
		odoReadingsX = odoData[0];
		odoReadingsY = odoData[1];
		odoReadingsT = odoData[2];
		
		
		//calculated change in robot position
		deltaX = x - odoReadingsX;
		deltaY = y - odoReadingsY;
		
		// calculate the minimum angle robot must turn to go straight at next point
		minAngle = Math.toDegrees(Math.atan2( deltaX, deltaY)) - odoReadingsT;
					
		// turn to the minimum angle
		turnTo(minAngle);
		
		// calculate the distance to next point
		distance  = Math.hypot(deltaX, deltaY);
		
		// move straight to next point
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD,distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
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
		else {  //go straight otherwise
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
		
	}
	
	
}