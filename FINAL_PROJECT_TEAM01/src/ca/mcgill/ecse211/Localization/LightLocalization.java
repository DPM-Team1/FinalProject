package ca.mcgill.ecse211.Localization;

//imports
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.Arrays;

import ca.mcgill.ecse211.Localization.LocalNavigation;
import ca.mcgill.ecse211.General.*;

/** This class navigates the robot to the (0,0,0) position using 2 light sensors.
* @author Garine Imamedjian
* @author Rebecca Weill
* @author Alexa Normandin
* @author Sam Lantela
* @author Amine Alikacem
* @author Awais Shahid
*/
public class LightLocalization {

	//constants	
	/**Motor speed when advancing and turning*/
	public static int MOTOR_SPEED = 140;	
	/**Distance needed to back up to read y values*/
	public static int DISTANCE_FROM_EDGE = 8;
	/**Low acceleration rate to reduce drifting*/
	public static int ACCELERATION = 60000;	
	/**Distance from center of robot to the light sensor horizontally*/
	private static double lightSensorDistanceX = 5;
	/**Distance from center of robot to the light sensor vertically*/
	public static double lightSensorDistanceY = 4;	
	/**base intensity detected from left sensor*/
	private static double firstIntensityL;
	/**base intensity for right sensor*/
	private static double firstIntensityR;	
	/**percentage used to detect dark blue line*/
	private static double thresholdPercentage = 0.2;

	
	//class variables
	/**Odometer*/
	private LocalOdometer odo;
	/**Sample provider for the light sensor in front of the left wheel*/
	private SampleProvider leftColorSensor;
	/**Sample provider for the light sensor in front of the right wheel*/
	private SampleProvider rightColorSensor;
	/**motor for left wheel*/
	private EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private EV3LargeRegulatedMotor rightMotor;

	

	/**
	 * This is the class constructor
	 * 
	 * @param odo
	 * @param leftColorSensor
	 * @param leftColorData
	 * @param rightColorSensor
	 * @param rightColorData
	 * @param navi
	 */
	public LightLocalization(LocalOdometer odo, SampleProvider leftColorSensor, float[] leftColorData, 
			SampleProvider rightColorSensor, float[] rightColorData, LocalNavigation navi) {
		//get incoming values for variables
		this.odo = odo;
		this.leftColorSensor = leftColorSensor;
		this.rightColorSensor = rightColorSensor;
		//set up motors
		EV3LargeRegulatedMotor[] motors = odo.getMotors();
		this.leftMotor = motors[0];		
		this.rightMotor = motors[1];
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);

	}





	/**
	 * This method is the main method of the class that runs 
	 * the localization of the robot with the light sensors.
	 * We assume that theta is close enough to 0 to start moving towards the x and y directions.
	 * We correct x, y and theta to get to the (0,0,0) position.
	 */
	public void doLocalization() {

		//variable to be used when correcting odometer
		double position[] = new double [3];
		double leftColor;
		double rightColor;

		//Get intensity of regular surface
		firstIntensityL = getMedianIntensity(leftColorSensor);
		firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		//get the current intensity
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);

		//While not on crossing line, sleep to give time to other threads
		while (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			leftColor= getMedianIntensity(leftColorSensor);
			rightColor = getMedianIntensity(rightColorSensor);
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		//line detected, stop, and get intensities again
		leftMotor.stop(true);
		rightMotor.stop(false);
		Sound.beep();
		if ((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage) {
			rightColor = getMedianIntensity(rightColorSensor);
		}
		if ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage) {
			leftColor= getMedianIntensity(leftColorSensor);
		}

		//if both detected, stop both motors
		if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
		}

		//if left was seen
		else if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			//start right motor
			rightMotor.forward();
			rightColor = getMedianIntensity(rightColorSensor);

			//run until right sensor sees line
			while ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage) {
				rightColor = getMedianIntensity(rightColorSensor);
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			//stop right motor
			rightMotor.stop(false);
			Sound.beep();
		}

		//if right was seen
		else if (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {

			//start left motor
			leftMotor.forward();
			leftColor = getMedianIntensity(leftColorSensor);

			//run until left sensor sees line
			while ((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage) {
				leftColor = getMedianIntensity(leftColorSensor);
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			//stop left motor
			leftMotor.stop(false);
			Sound.beep();
		}

		// Crossed black line, stop and reset odometer
		position[0] = lightSensorDistanceX;
		position[1] = 0;
		position[2] = 0;
		boolean posBool[] = new boolean [3];
		posBool[0] = true;
		posBool[1] = false;
		posBool[2] = false;
		odo.setPosition(position, posBool);

		//now go backwards to reset and start with y
		leftMotor.rotate(-convertDistance(FinalProject.WHEEL_RAD, DISTANCE_FROM_EDGE), true);
		rightMotor.rotate(-convertDistance(FinalProject.WHEEL_RAD, DISTANCE_FROM_EDGE), false);

		//Rotate to go towards y axis
		leftMotor.rotate(-convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, 90.0), false);

		//Get intensity of regular surface
		firstIntensityL = getMedianIntensity(leftColorSensor);
		firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		//get the current intensity
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);

		//While not on crossing line, sleep to give time to other threads
		while (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			leftColor= getMedianIntensity(leftColorSensor);
			rightColor = getMedianIntensity(rightColorSensor);
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		//line detected, stop, and get intensities again
		leftMotor.stop(true);
		rightMotor.stop(false);
		Sound.beep();
		if ((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage) {
			rightColor = getMedianIntensity(rightColorSensor);
		}
		if ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage) {
			leftColor= getMedianIntensity(leftColorSensor);
		}

		//if both detected, stop both motors
		if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
		}

		//if left was seen
		else if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			//start right motor
			rightMotor.forward();
			rightColor = getMedianIntensity(rightColorSensor);

			//run until right sensor sees line
			while ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage) {
				rightColor = getMedianIntensity(rightColorSensor);
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			//stop right motor
			rightMotor.stop(false);
			Sound.beep();
		}

		//if right was seen
		else if (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {

			//start left motor
			leftMotor.forward();
			leftColor = getMedianIntensity(leftColorSensor);

			//run until left sensor sees line
			while ((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage) {
				leftColor = getMedianIntensity(leftColorSensor);
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			//stop left motor
			leftMotor.stop(false);
			Sound.beep();
		}

		// Crossed black line, stop and reset odometer
		position[0] = 0;
		position[1] = lightSensorDistanceY;
		position[2] = 0;
		posBool[0] = false;
		posBool[1] = true;
		posBool[2] = false;
		odo.setPosition(position, posBool);

		//advance to compensate for the offset of the light sensor
		leftMotor.rotate(convertDistance(FinalProject.WHEEL_RAD, lightSensorDistanceY), true);
		rightMotor.rotate(convertDistance(FinalProject.WHEEL_RAD, lightSensorDistanceY), false);

		//Rotate 90 degrees to go to 0,0
		leftMotor.rotate(convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, 90.0), false);

		//Advance to set x to 0 and copensate for the offset of the light sensor
		leftMotor.rotate(convertDistance(FinalProject.WHEEL_RAD, DISTANCE_FROM_EDGE + lightSensorDistanceX), true);
		rightMotor.rotate(convertDistance(FinalProject.WHEEL_RAD, DISTANCE_FROM_EDGE + lightSensorDistanceX), false);

		//Rotate 90 degrees to face theta 0
		leftMotor.rotate(convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, -90.0), true);
		rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RAD, FinalProject.TRACK, -90.0), false);

		//reset Odo to right position
		position[0] = FinalProject.TILE_SIZE;
		position[1] = FinalProject.TILE_SIZE;
		position[2] = 0;
		posBool[0] = true;
		posBool[1] = true;
		posBool[2] = true;
		odo.setPosition(position, posBool);


	}


	/**
	 * This method is used to convert the angle needed to turn to the number of wheel rotations necessary to do so
	 * @param radius
	 * @param width
	 * @param angle
	 * @return rotations	
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method is used to convert the distance needed to travel to the number of wheel rotations necessary to do so
	 * @param radius
	 * @param distance
	 * @return rotations	
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method gets the median intensity of color sensor values
	 * @param colorSensor
	 * @return median intensity
	 */
	private float getMedianIntensity(SampleProvider colorSensor) {

		int N= 10;
		float[] samples = new float[N];

		//iterate through array
		for (int i = 0; i < N ; i++) {
			colorSensor.fetchSample(samples, i);
		}
		Arrays.sort(samples);

		return (samples[N/2] + samples[(N/2) +1]) / 2.0f;
	}


}