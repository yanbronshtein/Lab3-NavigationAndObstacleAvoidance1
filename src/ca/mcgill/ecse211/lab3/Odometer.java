/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoReadings;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	public int lastTachoLeft, lastTachoRight;
	private Object lock; //Public or private?
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position;


	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		odoReadings = OdometerData.getOdometerData(); // Allows access to x,y, and theta
		// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoReadings.setXYT(0, 0, 0);
		lock  = new Object();
		this.leftMotorTachoCount = 0; 
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
					throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be used only if an
	 * odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		double distL, distR, deltaD, deltaT, dX, dY;
		position = odoReadings.getXYT();
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		lastTachoLeft = leftMotor.getTachoCount();
		lastTachoRight = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			lastTachoLeft = leftMotor.getTachoCount();
			lastTachoRight = rightMotor.getTachoCount();

			// TODO Calculate new robot position based on tachometer counts
			distL = Math.PI * WHEEL_RAD * (lastTachoLeft - leftMotorTachoCount)/180;
			distR = Math.PI * WHEEL_RAD * ( lastTachoRight -  rightMotorTachoCount)/180;

			leftMotorTachoCount= lastTachoLeft;
			rightMotorTachoCount = lastTachoRight;

			deltaD =  0.5 * ( distL + distR );
			deltaT = ( distL - distR ) / TRACK;



			synchronized (lock){ //as long as lock exist only this can run. else robot burns itself

				position[2] += deltaT;
				if(position[2] >= 2*Math.PI){
					position[2] = position[2] - 360*Math.PI/180;
				}
				else if(position[2] < 0){
					position[2] = position[2] + 360*Math.PI/180;
				}
				else {
					
				}

				dX = deltaD * Math.sin(position[2]);
				dY = deltaD * Math.cos(position[2]);


			}



			//Update odometer values with new calculated values
			odo.update(dX, dY, Math.toDegrees(deltaT) );

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}