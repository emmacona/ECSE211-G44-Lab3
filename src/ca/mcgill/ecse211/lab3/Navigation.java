package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;

import lejos.hardware.Sound;

public class Navigation implements Runnable {

	/**
	 *  Maps used for demo
	 */
	int [][] map1 = {{1, 3}, {2 ,2}, {3, 3}, {3, 2}, {2, 1}};
	int [][] map2 = {{2, 2}, {1, 3}, {3, 3}, {3, 2}, {2, 1}};
	int [][] map3 = {{2, 1}, {3, 2}, {3, 3}, {1, 3}, {2, 2}};
	int [][] map4 = {{1, 2}, {2, 3}, {2, 1}, {3, 2}, {3, 3}};

	/**
	 *  Navigation runs from here.
	 */
	public void run() {
		int[][] waypoints = map1;

		for (int[] point : waypoints) {
			Sound.beep();
			// since starts at (1, 1), only want to travel 2 tile sizes for (1, 3)
			// so need to subtract 1 from all the coordinates
			travelTo((point[0]-1)*TILE_SIZE, (point[1]-1)*TILE_SIZE);
		}
		setSpeeds(0, 0);
	}

	/**
	 * Travels to designated position, while constantly updating its heading.
	 * 
	 * @param x the destination x, in cm.
	 * @param y the destination y, in cm.
	 */
	public static void travelTo(double x, double y) {

		// Get coordinates
		double currentX = odometer.getXYT()[0];
		double currentY = odometer.getXYT()[1];
		// Initialize variables
		double deltaX;
		double deltaY;
		double minAng;

		while (!isDone(x, y)) { // while has not arrived at way point yet
			// Update current coordinates
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			deltaX = x - currentX;
			deltaY = y - currentY;
			// Determine angle need to turn to
			minAng = atan2(deltaX, deltaY) * (180.0 / PI); 
			if (minAng < 0.0) { // if negative angle, add 360
			  minAng += 360.0;
			}
			
			// turn to angle found previously
			turnTo(minAng, x, y);

			// set motor speeds
			setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

			while (!isDone(x, y)) { // while still not at the way point, check for obstacles
				// Update coordinates
				currentX = odometer.getXYT()[0];
				currentY = odometer.getXYT()[1];
				// fetch data from sensor, as in Lab 1
				usSensor.fetchSample(usValues, 0);
				int distance = (int) (usValues[0] * 100); // to decrease error cm --> *100
				if (distance < AVOIDANCE_DISTANCE) {// check if safe distance from a block
					obstacleAvoidance();
					break; // need to break after avoidance completed, to get out of this while loop and back to "outer" while loop
				}
			}
		}
	}

	/**
	 * Turns robot away from the block.
	 * 
	 */
	private static void obstacleAvoidance() {
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.setSpeed(ROTATE_SPEED);

		// if wall turn 90 deg
		rightMotor.rotate(-convertAngle(90), true);
		leftMotor.rotate(convertAngle(90), false);
		rightMotor.forward();
		leftMotor.forward();

		// then go straight for a bit
		rightMotor.rotate(convertDistance(TILE_SIZE), true);
		leftMotor.rotate(convertDistance(TILE_SIZE), false);
		rightMotor.forward();
		leftMotor.forward();
		
		// when no wall 90 deg back
		rightMotor.rotate(convertAngle(90), true);
		leftMotor.rotate(-convertAngle(90), false);
		rightMotor.forward();
		leftMotor.forward();
		
		rightMotor.rotate(convertDistance(TILE_SIZE));
		leftMotor.rotate(convertDistance(TILE_SIZE));
	}

	/**
	 * Turns robot towards the indicated angle.
	 * 
	 * @param angle
	 * @param x, the goal x location
	 * @param y, the goal y location
	 */
	public static void turnTo(double angle, double x, double y) {
	    double theta = odometer.getXYT()[2]; // Get current theta reading
	    double error = angle - theta; // Calculate what's left to get to the wanted angle
	    
	    while (abs(error) > DEG_ERR) { // while we are not close enough to goal angle
	      if (error > 180.0) { // if bigger than 180, turn right
	        setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
	      } else if (error < -180.0) { // if smaller than negative 180, turn left
	        setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
	      } else if (error > 0.0) { // if bigger than 0, turn left
	        setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
	      } else { // if smaller than 0, turn right
	        setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
	      }
	      theta = odometer.getXYT()[2]; // update current theta reading
	      error = angle - theta; // update error
	    }
	}

	/**
	 * Dist remaining to point
	 * 
	 * @param x
	 * @param y
	 * @return dist
	 */
	public static double distRemaining(double x, double y) {
		return(Math.hypot(x, y));
	}

	/**
	 * Returns {@code true} when done.
	 * 
	 * @param x
	 * @param y
	 * @return {@code true} when done.
	 */
	public static boolean isDone(double x, double y) {
		double error = Math.sqrt(Math.pow((odometer.getXYT()[0] - x), 2) + Math.pow((odometer.getXYT()[1] - y), 2));
		//double error = distRemaining(odometer.getXYT()[0] - x, odometer.getXYT()[1] - y);
		return error < CM_ERR;
	}


	/**
	 * Converts input distance to the total rotation of each wheel needed to cover that distance.
	 * 
	 * @param distance
	 * @return the wheel rotations necessary to cover the distance
	 */
	public static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
	 * angle.
	 * 
	 * @param angle
	 * @return the wheel rotations necessary to rotate the robot by the angle
	 */
	public static int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}

	/**
	 * Sets the motor speeds jointly.
	 */
	public static void setSpeeds(float leftSpeed, float rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		if (leftSpeed < 0) {
			leftMotor.backward();
		} else {
			leftMotor.forward();
		}
		if (rightSpeed < 0) {
			rightMotor.backward();
		} else {
			rightMotor.forward();
		}
	}


}
