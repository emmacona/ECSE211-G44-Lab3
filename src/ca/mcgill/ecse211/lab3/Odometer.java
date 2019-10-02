package ca.mcgill.ecse211.lab3;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Younes Boubekeur
 */

public class Odometer implements Runnable {

	// Class variables
	public static int lastTachoL = leftMotor.getTachoCount(); 
	public static int lastTachoR = rightMotor.getTachoCount();
	public static double X; 
	public static double Y; 
	public static double Theta;

	/**
	 * The x-axis position in cm.
	 */
	private volatile double x;

	/**
	 * The y-axis position in cm.
	 */
	private volatile double y; // y-axis position

	/**
	 * The orientation in degrees.
	 */
	private volatile double theta; // Head angle

	/**
	 * The (x, y, theta) position as an array
	 */
	private double[] position;

	// Thread control tools
	/**
	 * Fair lock for concurrent writing
	 */
	private static Lock lock = new ReentrantLock(true);

	/**
	 * Indicates if a thread is trying to reset any position parameters
	 */
	private volatile boolean isResetting = false;

	/**
	 * Lets other threads know that a reset operation is over.
	 */
	private Condition doneResetting = lock.newCondition();

	private static Odometer odo; // Returned as singleton

	// Motor-related variables
	private static int leftMotorTachoCount = 0;
	private static int rightMotorTachoCount = 0;

	/**
	 * The odometer update period in ms.
	 */
	private static final long ODOMETER_PERIOD = 25;


	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 */
	private Odometer() {
		setXYT(0, 0, 0);
	}

	/**
	 * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
	 * 
	 * @return the Odometer Object
	 */
	public synchronized static Odometer getOdometer() {
		if (odo == null) {
			odo = new Odometer();
		}

		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run.
	 * This method counts the number of times each will has turned and updates the position accordingly.
	 * (for x, y, and theta)
	 */
	
	public void run() {
		long updateStart, updateEnd;
		double deltaD, deltaT, dX, dY;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount(); // get current tacho counts
			rightMotorTachoCount = rightMotor.getTachoCount(); 

			// Calculate displacements
			double leftDistance = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180;
			double rightDistance = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180;
			
			// Update tacho counts
			lastTachoL = leftMotorTachoCount; 
			lastTachoR = rightMotorTachoCount;
			
			// Calculate differences
			deltaD = 0.5*(leftDistance + rightDistance);  //change in distance
			deltaT = (leftDistance - rightDistance)/TRACK; // change in tetha
			Theta += deltaT; // update theta
			dX = deltaD * Math.sin(Theta); // calculate displacement in x
			dY = deltaD * Math.cos(Theta); // calculate displacement in y
			
			deltaT = (deltaT * 180) / Math.PI; // Convert from radians to degree

			//Update odometer values with new calculated values
			odo.update(dX, dY, deltaT);

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

	// IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE

	/**
	 * Returns the Odometer data.
	 * <p>
	 * Writes the current position and orientation of the robot onto the odoData array. {@code odoData[0] =
	 * x, odoData[1] = y; odoData[2] = theta;}
	 * 
	 * @param position the array to store the odometer data
	 * @return the odometer data.
	 */
	public double[] getXYT() {
		double[] position = new double[3];
		lock.lock();
		try {
			while (isResetting) { // If a reset operation is being executed, wait until it is over.
				doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
			}

			position[0] = x;
			position[1] = y;
			position[2] = theta;
		} catch (InterruptedException e) {
			e.printStackTrace();
		} finally {
			lock.unlock();
		}

		return position;
	}

	/**
	 * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for
	 * odometry.
	 * 
	 * @param dx
	 * @param dy
	 * @param dtheta
	 */
	public void update(double dx, double dy, double dtheta) {
		lock.lock();
		isResetting = true;
		try {
			x += dx;
			y += dy;
			theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
			isResetting = false;
			doneResetting.signalAll(); // Let the other threads know we are done resetting
		} finally {
			lock.unlock();
		}

	}

	/**
	 * Overrides the values of x, y and theta. Use for odometry correction.
	 * 
	 * @param x the value of x
	 * @param y the value of y
	 * @param theta the value of theta in degrees
	 */
	public void setXYT(double x, double y, double theta) {
		lock.lock();
		isResetting = true;
		try {
			this.x = x;
			this.y = y;
			this.theta = theta;
			isResetting = false;
			doneResetting.signalAll();
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overwrites x. Use for odometry correction.
	 * 
	 * @param x the value of x
	 */
	public void setX(double x) {
		lock.lock();
		isResetting = true;
		try {
			this.x = x;
			isResetting = false;
			doneResetting.signalAll();
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overwrites y. Use for odometry correction.
	 * 
	 * @param y the value of y
	 */
	public void setY(double y) {
		lock.lock();
		isResetting = true;
		try {
			this.y = y;
			isResetting = false;
			doneResetting.signalAll();
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overwrites theta. Use for odometry correction.
	 * 
	 * @param theta the value of theta
	 */
	public void setTheta(double theta) {
		lock.lock();
		isResetting = true;
		try {
			this.theta = theta;
			isResetting = false;
			doneResetting.signalAll();
		} finally {
			lock.unlock();
		}
	}

}
