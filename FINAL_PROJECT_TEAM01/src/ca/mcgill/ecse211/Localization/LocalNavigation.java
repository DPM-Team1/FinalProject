package ca.mcgill.ecse211.Localization;


import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class takes care of the speeds set and then turning function needed when localizing
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */
public class LocalNavigation {

	/**Odometer*/
	private LocalOdometer odometer;
	/**motor for left wheel*/
	private EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private EV3LargeRegulatedMotor rightMotor;

	//Constants
	/**Motor speed when turning*/
	final static int MOTOR_SPEED = 250;	
	/**Acceleration rate to reduce drifting */
	final static int ACCELERATION = 2000;
	/**Acceptable error of degree*/
	final static double DEGREE_ERROR = 3.0;		

	/**
	 * This is the class constructor
	 * 
	 * @param odo
	 */
	public LocalNavigation(LocalOdometer odo) {
		this.odometer = odo;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * This method sets the motor speeds for both wheels at the same time
	 * @param lSpd
	 * @param rSpd
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}


	/**
	 * This method turns the robot to the angle passed
	 * it then stops the motors or not, depending on the stop boolean
	 * @param angle
	 * @param stop
	 */
	public void turnToUS(double angle, boolean stop) {

		//Calculate difference in angle
		double error = angle - this.odometer.getAng();

		//Do not turn and/or stop turning if angle to turn is too small
		while (Math.abs(error) > DEGREE_ERROR) {

			//Compute difference in angle
			error = angle - this.odometer.getAng();

			//Turn the right way
			if (error < -180.0) {
				this.setSpeeds(-MOTOR_SPEED, MOTOR_SPEED);
			} else if (error < 0.0) {
				this.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED);
			} else if (error > 180.0) {
				this.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED);
			} else {
				this.setSpeeds(-MOTOR_SPEED, MOTOR_SPEED);
			}
		}

		//boolean passed = true, stop motors
		if (stop) {
			this.setSpeeds(0, 0);
		}
	}






}
