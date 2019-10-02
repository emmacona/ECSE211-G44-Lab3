package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;

// static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;

/**
 * The main driver class for the navigation lab.
 */
public class Main {

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		//start the threads
		new Thread(odometer).start();
		new Thread(navig).start();
		new Thread(new Display()).start();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		} // do nothing

		System.exit(0);
	}


	/**
	 * Sleeps current thread for the specified duration.
	 * 
	 * @param duration sleep duration in milliseconds
	 */
	public static void sleepFor(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

}
