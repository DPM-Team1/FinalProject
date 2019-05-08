package ca.mcgill.ecse211.Navigation;


//imports
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
* This class calculates the position of the robot (x, y, and theta) using 
* its internal tachometer count and 2 light sensors.
* @author Garine Imamedjian
* @author Rebecca Weill
* @author Alexa Normandin
* @author Sam Lantela
* @author Amine Alikacem
* @author Awais Shahid
*/
public class Odometer extends Thread {
	
	/**robot position coordinate*/
	private double x, y, t;
	/**current tachometer count of the left motor*/
	private int leftMotorNowTachoCount;
	/**current tachometer count of the right motor*/
	private int rightMotorNowTachoCount;
	/**previous tachometer count of the left motor*/
	private int leftMotorLastTachoCount;
	/**previous tachometer count of the right motor*/
	private int rightMotorLastTachoCount;
	/**motor for left wheel*/
	private EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private EV3LargeRegulatedMotor rightMotor;

	/**wheel base*/
	private final double TRACK;
	/**wheel radius*/
	private final double WHEEL_RAD;

	/**Robot position array with 3 coordinates*/
	private double[] position = new double [3];


	/**odometer update period in ms*/
	private static final long ODOMETER_PERIOD = 25; 

	/**lock object for mutual exclusion*/
	private Object lock;

	/**
	 * This is the class constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		this.x = 0.0;
		this.y = 0.0;
		this.t = 0.0;

		this.position [0] = 0;
		this.position [1] = 0;
		this.position [2] = 0;

		this.leftMotorNowTachoCount = 0;
		this.rightMotorNowTachoCount = 0;
		this.leftMotorLastTachoCount = 0;
		this.rightMotorLastTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		lock = new Object();
	}
	
	
	/**
	 * This is the run method required for the thread.
	 * It uses the internal tachometer count to estimate its position.
	 * It corrects its position and theta when crossing a gridline. 
	 */
	public void run() {
		long updateStart, updateEnd;
		double x, y;
		double dx, dy, th, dTh, dDist, leftDist, rightDist;

		//Get original tacho count
		leftMotorLastTachoCount = leftMotor.getTachoCount(); 		
		rightMotorLastTachoCount = rightMotor.getTachoCount();


		while (true) {
			updateStart = System.currentTimeMillis();

			//Get current tacho count for each wheel
			leftMotorNowTachoCount = leftMotor.getTachoCount();		
			rightMotorNowTachoCount = rightMotor.getTachoCount();

			x = getX();
			y = getY();
			th = getTheta();
			position[0] = x;			//Get position and angle of robot given by internal odometer
			position[1] = y;
			position[2] = th;


			//Calculate the distance traveled by each wheel using the number of wheel rotations
			leftDist = Math.PI * WHEEL_RAD * (leftMotorNowTachoCount - leftMotorLastTachoCount) / 180;
			rightDist = Math.PI * WHEEL_RAD * (rightMotorNowTachoCount - rightMotorLastTachoCount) / 180;

			//Set current values as the old values to be used in the next calculation
			leftMotorLastTachoCount = leftMotorNowTachoCount;
			rightMotorLastTachoCount = rightMotorNowTachoCount;

			dDist = (leftDist + rightDist) / 2;		//Average distance


			dTh = (leftDist - rightDist) / TRACK;		//Calculate theta with distance traveled by each wheel
			dTh = Math.toDegrees(dTh);
			position[2] += dTh;
			if (position[2] > 359.9)
				position[2] = position[2] - 360;

			//Calculate the real x and y positions
			dx = dDist * Math.sin(Math.toRadians(th));		
			dy = dDist * Math.cos(Math.toRadians(th));


			// Update odometer values with new calculated values
			setX(dx+position[0]);
			setY(dy+position[1]);
			setTheta(position[2]);

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

	/**
	 * This is the accessor method to get the full position of the robot (x, y and theta)
	 * @param position
	 * @param update
	 */
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = t;
		}
	}

	/**
	 * This is the accessor method to get the x position
	 * @return x
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * This is the accessor method to get the y position
	 * @return y
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * This is the accessor method to get the theta position
	 * @return theta
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = t;
		}

		return result;
	}

	/**
	 * This is the mutator method to set the full position of the robot (x, y and theta)
	 * @param position
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				t = position[2];
		}
	}

	/**
	 * This is the mutator method to set the x position
	 * @param x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * This is the mutator method to set the y position
	 * @param y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * This is the mutator method to set the theta position
	 * @param theta
	 */
	public void setTheta(double t) {
		synchronized (lock) {
			this.t = t;
		}
	}

	/**
	 * This is the accessor method to get the internal tachometer count of the left motor
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorNowTachoCount;
	}

	/**
	 * This is the mutator method to set the internal tachometer count of the left motor
	 * @param the leftMotorTachoCount
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorNowTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * This is the accessor method to get the internal tachometer count of the left motor
	 * @return the leftMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorNowTachoCount;
	}

	/**
	 * This is the mutator method to set the internal tachometer count of the left motor
	 * @param the leftMotorTachoCount
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorNowTachoCount = rightMotorTachoCount;	
		}
	}
}
