package ca.mcgill.ecse211.Navigation;

//imports
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.Localization.*;
import ca.mcgill.ecse211.General.*;

import java.util.Arrays;


/**
 * This class implements the different methods needed for the robot to travel to different point on the map.
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */
public class Navigation extends Thread {

	//class variables

	/**Odometer*/
	private Odometer odometer;
	/**motor for left wheel*/
	private EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private EV3LargeRegulatedMotor rightMotor;
	/**Sample provider for the light sensor in front of the left wheel*/
	private SampleProvider leftColorSensor;
	/**Sample provider for the light sensor in front of the right wheel*/
	private SampleProvider rightColorSensor;
	/**left motor for arm*/
	private EV3LargeRegulatedMotor leftArmMotor;
	/**right motor for arm*/
	private EV3LargeRegulatedMotor rightArmMotor;

	/**distance detected by ultrasonic sensor*/
	int distance;		
	/**ideal detection distance*/
	int ringDistance = 15;



	/**
	 * This is the class constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftColorSensor
	 * @param rightColorSensor
	 * @param leftArmMotor
	 * @param rightArmMotor
	 */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			SampleProvider leftColorSensor, SampleProvider rightColorSensor, EV3LargeRegulatedMotor leftArmMotor, 
			EV3LargeRegulatedMotor rightArmMotor){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftColorSensor = leftColorSensor;
		this.rightColorSensor = rightColorSensor;
		this.leftArmMotor = leftArmMotor;
		this.rightArmMotor = rightArmMotor;
	}


	/**percentage used for dark blue line detection*/
	public static double thresholdPercentage = 0.25;	
	/**Speed of the motors when advancing to the next destination*/
	private static final int FORWARD_SPEED = 600; //500
	/**Speed of the motors when backing up to localize with the grid line*/
	public static final int LOCAL_SPEED = 130;	//110
	/**Constant multiplied to compensate for the inequality of the motors*/
	private static final double MOTOR_DIFF = 1.0192; //1.095
	/**Speed of the motors when the robot is turning towards its next destination*/
	private static final int ROTATE_SPEED = 190;	//150
	/**wheel radius*/
	private static final double WHEEL_RAD = FinalProject.WHEEL_RAD;
	/**wheel base*/
	private static final double TRACK = FinalProject.TRACK;
	/**tile size on board*/
	private static final double TILE_SIZE = 30.48;
	private static final int ACC = 500;



	/**
	 * This method implements orientation change of the robot and the navigation needed to get to the next waypoint
	 * @param x (measured in tiles)
	 * @param y (measured in tiles)
	 */
	public void travelTo(double x, double y) {

		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(ACC);
		}

		//get x and y in centimeters
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;


		//calculate trajectory path and angle
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();

		//Find smallest necessary angle to turn to
		double dTh;
		if(dX >= 0 & dY >= 0){
			dTh = (180/Math.PI) * Math.atan(dX/dY);
		}else if(dX >= 0 & dY < 0){
			dTh = 90 + (180/Math.PI) * -Math.atan(dY/dX);
		}else if(dX < 0 & dY < 0){
			dTh = 180 + (180/Math.PI) * Math.atan(dX/dY);
		}else{
			dTh = 270 + (180/Math.PI) * -Math.atan(dY/dX);
		}

		//rotate to correct angle
		//Sound.beepSequenceUp();
		//leftMotor.setSpeed(ROTATE_SPEED);
		//rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(dTh);

		//Find distance that needs to be traveled
		double distance = Math.hypot(dX, dY);

		//move forward correct distance
		//Sound.beepSequence();
		leftMotor.setSpeed((int) (FORWARD_SPEED ));
		rightMotor.setSpeed((int) (FORWARD_SPEED*MOTOR_DIFF));
		leftMotor.rotate(convertDistance(distance ),true);
		rightMotor.rotate(convertDistance(distance*MOTOR_DIFF),false);

	}



	/**
	 * This method is made to travel through the tunnel
	 *  - Turn to right angle
	 * 	- Light Localization
	 * 	- Go through tunnel
	 *  - Light Localization
	 * @param x (measured in tiles)
	 * @param y (measured in tiles)
	 */
	public void travelThroughTunnel(double x, double y, boolean toTree) {

		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(ACC);
		}

		//get x and y in centimeters
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;


		if (toTree) {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					turnTo(0);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
							((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
				else {
					turnTo(90);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 90);
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					turnTo(0);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
							((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
				else {
					turnTo(270);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 270);
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					turnTo(180);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
							((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					turnTo(270);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 270);
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					turnTo(180);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
							((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					turnTo(90);
					leftMotor.rotate(convertDistance(7),true);
					rightMotor.rotate(convertDistance(7),false);
					localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 90);
				}
			}
		}

		else {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					if (FinalProject.LEFT) {
						turnTo(180);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
								((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
					}
					else {
						turnTo(0);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
								((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
					}
				}
				else {
					if (FinalProject.LEFT) {
						turnTo(90);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 90);
					}
					else {
						turnTo(270);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 270);}
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					if (FinalProject.LEFT) {
						turnTo(0);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
								((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
					}
					else {
						turnTo(180);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
								((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
					}
				}
				else {
					if (FinalProject.LEFT) {
						turnTo(90);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 90);
					}
					else {
						turnTo(270);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_UR_Y + 1)* TILE_SIZE, 270);
					}
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					if (FinalProject.LEFT) {
						turnTo(0);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
								((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
					}
					else {
						turnTo(180);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE, 
								((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
					}
				}
				else {
					if (FinalProject.LEFT) {
						turnTo(270);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 270);
					}
					else {
						turnTo(90);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 90);
					}
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					if (FinalProject.LEFT) {
						turnTo(180);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
								((double)FinalProject.TNG_UR_Y * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
					}
					else {
						turnTo(0);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE, 
								((double)FinalProject.TNG_LL_Y * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
					}
				}
				else {
					if (FinalProject.LEFT) {
						turnTo(270);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_UR_X * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 270);
					}
					else {
						turnTo(90);
						leftMotor.rotate(convertDistance(7),true);
						rightMotor.rotate(convertDistance(7),false);
						localizeBackward(((double)FinalProject.TNG_LL_X * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
								((double)FinalProject.TNG_LL_Y - 1)* TILE_SIZE, 90);
					}
				}
			}
		}

		leftMotor.setAcceleration(ACC);
		rightMotor.setAcceleration(ACC);

		leftMotor.rotate(convertDistance(0.5*TILE_SIZE + LightLocalization.lightSensorDistanceY),true);
		rightMotor.rotate(convertDistance(0.5*TILE_SIZE + LightLocalization.lightSensorDistanceY),false);


		//calculate trajectory path and angle
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();

		//Find smallest necessary angle to turn to
		double dTh;
		if(dX >= 0 & dY >= 0){
			dTh = (180/Math.PI) * Math.atan(dX/dY);
		}else if(dX >= 0 & dY < 0){
			dTh = 90 + (180/Math.PI) * -Math.atan(dY/dX);
		}else if(dX < 0 & dY < 0){
			dTh = 180 + (180/Math.PI) * Math.atan(dX/dY);
		}else{
			dTh = 270 + (180/Math.PI) * -Math.atan(dY/dX);
		}

		//rotate to correct angle
		//Sound.beepSequenceUp();
		//leftMotor.setSpeed(ROTATE_SPEED);
		//rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(dTh);

		leftMotor.rotate(convertDistance(7),true);
		rightMotor.rotate(convertDistance(7),false);

		if (toTree) {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE),
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}
		}

		else {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					localizeBackward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
		}

		leftMotor.setAcceleration(ACC);
		rightMotor.setAcceleration(ACC);
		leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
		rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
		leftArmMotor.rotateTo(-30, true);
		rightArmMotor.rotateTo(-30, false);
		leftArmMotor.stop();
		rightArmMotor.stop();

		dX = x - odometer.getX();
		dY = y - odometer.getY();



		//Find distance that needs to be traveled
		double distance = Math.hypot(dX, dY);

		//move forward correct distance
		//Sound.beepSequence();
		leftMotor.setSpeed((int) (FORWARD_SPEED ));
		rightMotor.setSpeed((int) (FORWARD_SPEED*MOTOR_DIFF));
		leftMotor.rotate(convertDistance(distance ),true);
		rightMotor.rotate(convertDistance(distance*MOTOR_DIFF),false);

		leftArmMotor.rotateTo(-60, true);
		rightArmMotor.rotateTo(-60, false);
		leftArmMotor.stop();
		rightArmMotor.stop();

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(LOCAL_SPEED);
		rightMotor.setSpeed(LOCAL_SPEED);
		leftMotor.forward();
		rightMotor.forward();


		if (toTree) {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

		}

		else {

			if (FinalProject.GreenCorner == 0) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 1) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_LL_Y - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
			}

			if (FinalProject.GreenCorner == 2) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_UR_X + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 90);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE), 
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}

			if (FinalProject.GreenCorner == 3) {
				if (FinalProject.HORIZONTAL) {
					localizeForward((((double)FinalProject.TNG_LL_X - 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY),
							(((double)FinalProject.TNG_LL_Y + 0.5) * TILE_SIZE), 270);
				}
				else {
					localizeForward((((double)FinalProject.TNG_LL_X + 0.5) * TILE_SIZE),
							(((double)FinalProject.TNG_UR_Y + 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
		}
	}




	/**
	 * This method is made to travel to the tree once it has crossed the tunnel and 
	 * localize with light sensors once it arrives in x and y
	 * 
	 * @param x (measured in tiles)
	 * @param y (measured in tiles)
	 */
	public void travelToTree(double x, double y) {

		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(ACC);
		}

		//get x and y in centimeters
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;


		//calculate trajectory path and angle
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();

		//Find smallest necessary angle to turn to
		double dTh;
		if(dX >= 0 & dY >= 0){
			dTh = (180/Math.PI) * Math.atan(dX/dY);
		}else if(dX >= 0 & dY < 0){
			dTh = 90 + (180/Math.PI) * -Math.atan(dY/dX);
		}else if(dX < 0 & dY < 0){
			dTh = 180 + (180/Math.PI) * Math.atan(dX/dY);
		}else{
			dTh = 270 + (180/Math.PI) * -Math.atan(dY/dX);
		}

		//rotate to correct angle
		//Sound.beepSequenceUp();
		//leftMotor.setSpeed(ROTATE_SPEED);
		//rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(dTh);

		//Find distance that needs to be traveled
		double distance = Math.hypot(dX, dY);

		//move forward correct distance
		//Sound.beepSequence();
		leftMotor.setSpeed((int) (FORWARD_SPEED ));
		rightMotor.setSpeed((int) (FORWARD_SPEED*MOTOR_DIFF));
		leftMotor.rotate(convertDistance(distance ),true);
		rightMotor.rotate(convertDistance(distance*MOTOR_DIFF),false);



		if (FinalProject.HORIZONTAL) {
			if (FinalProject.TNG_LL_Y > FinalProject.TG_Y) {
				turnTo(180);
			}
			else turnTo(0);
		}
		else {
			if (FinalProject.TNG_LL_X > FinalProject.TG_X) {
				turnTo(270);
			}
			else turnTo(90);
		}

		leftMotor.rotate(convertDistance(5),true);
		rightMotor.rotate(convertDistance(5),false);

		if (FinalProject.GreenCorner == 0) {
			if (FinalProject.HORIZONTAL) {
				if (FinalProject.TNG_LL_Y > FinalProject.TG_Y) {
					localizeBackward((((double)FinalProject.TG_X - 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X - 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
			else {
				if (FinalProject.TNG_LL_X > FinalProject.TG_X) {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y - 1) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y - 1) * TILE_SIZE), 90);
				}
			}
		}

		if (FinalProject.GreenCorner == 1) {
			if (FinalProject.HORIZONTAL) {
				if (FinalProject.TNG_LL_Y > FinalProject.TG_Y) {
					localizeBackward((((double)FinalProject.TG_X + 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X + 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
			else {
				if (FinalProject.TNG_LL_X > FinalProject.TG_X) {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y - 1) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y - 1) * TILE_SIZE), 90);
				}
			}
		}

		if (FinalProject.GreenCorner == 2) {
			if (FinalProject.HORIZONTAL) {
				if (FinalProject.TNG_LL_Y > FinalProject.TG_Y) {
					localizeBackward((((double)FinalProject.TG_X + 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X + 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
			else {
				if (FinalProject.TNG_LL_X > FinalProject.TG_X) {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y + 1) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y + 1) * TILE_SIZE), 90);
				}
			}
		}

		if (FinalProject.GreenCorner == 3) {
			if (FinalProject.HORIZONTAL) {
				if (FinalProject.TNG_LL_Y > FinalProject.TG_Y) {
					localizeBackward((((double)FinalProject.TG_X - 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X - 1)* TILE_SIZE), 
							(((double)FinalProject.TG_Y) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
			}
			else {
				if (FinalProject.TNG_LL_X > FinalProject.TG_X) {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y + 1) * TILE_SIZE), 270);
				}
				else {
					localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(((double)FinalProject.TG_Y + 1) * TILE_SIZE), 90);
				}
			}
		}

		
		leftMotor.setAcceleration(ACC);
		rightMotor.setAcceleration(ACC);

		leftMotor.rotate(convertDistance(5),true);
		rightMotor.rotate(convertDistance(5),false);


		if (FinalProject.GreenCorner == 0) {
			if (FinalProject.HORIZONTAL) {
				turnTo(90);
				localizeBackward((((double)FinalProject.TG_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
						(((double)FinalProject.TG_Y) * TILE_SIZE), 90);
			}
			else {
				turnTo(0);
				localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE), 
						(((double)FinalProject.TG_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
			}
		}

		if (FinalProject.GreenCorner == 1) {
			if (FinalProject.HORIZONTAL) {
				turnTo(270);
				localizeBackward((((double)FinalProject.TG_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
						(((double)FinalProject.TG_Y) * TILE_SIZE), 270);
			}
			else {
				turnTo(0);
				localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE), 
						(((double)FinalProject.TG_Y - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
			}
		}

		if (FinalProject.GreenCorner == 2) {
			if (FinalProject.HORIZONTAL) {
				turnTo(270);
				localizeBackward((((double)FinalProject.TG_X + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 
						(((double)FinalProject.TG_Y) * TILE_SIZE), 270);
			}
			else {
				turnTo(180);
				localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE), 
						(((double)FinalProject.TG_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
			}
		}

		if (FinalProject.GreenCorner == 3) {
			if (FinalProject.HORIZONTAL) {
				turnTo(90);
				localizeBackward((((double)FinalProject.TG_X - 1) * TILE_SIZE - LightLocalization.lightSensorDistanceY), 
						(((double)FinalProject.TG_Y) * TILE_SIZE), 90);
			}
			else {
				turnTo(180);
				localizeBackward((((double)FinalProject.TG_X) * TILE_SIZE), 
						(((double)FinalProject.TG_Y + 1) * TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
			}
		}

		
	}


	/**
	 * This method changes the angle of the robot from it current theta to
	 * the theta needed to travel to the next waypoint
	 * @param th
	 */
	public void turnTo(double th) {

		//get current angle
		double currentAngle = odometer.getTheta();
		double dT = 0;

		//Get smallest angle
		dT = th - currentAngle;
		if (dT > 180)
			dT = dT -360;
		else if (dT < -180)
			dT = dT + 360;

		//rotate robot to correct theta
		leftMotor.rotate(convertAngle(dT), true);
		rightMotor.rotate(-convertAngle(dT), false);

	}




	/**
	 * This method is used to convert the distance needed to travel to the number of wheel rotations necessary to do so
	 * @param distance (in cm)
	 * @return rotations	
	 */
	public static int convertDistance(double distance) {
		int rotations = (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
		return rotations;
	}


	/**
	 * This method is used to convert the angle needed to turn to the number of wheel rotations necessary to do so
	 * @param angle (in degrees)
	 * @return rotations	
	 */
	public static int convertAngle(double angle) {
		int rotations = convertDistance(Math.PI * TRACK * angle / 360.0);
		return rotations;
	}


	/**
	 * This method is used to filter out any false values given by the ultrasonic sensor
	 * @param distance	
	 */
	public void filter(int distance){

		int FILTER_OUT = 25;
		int filterControl = 0;

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
	 * This method gets the median intensity of color sensor values
	 * @param colorSensor
	 * @return median intensity
	 */
	public float getMedianIntensity(SampleProvider colorSensor) {

		int N = 10;
		float[] samples = new float[N];

		//iterate through array
		for (int i = 0; i < N ; i++) {
			colorSensor.fetchSample(samples, i);
		}
		Arrays.sort(samples);

		return (samples[N/2] + samples[(N/2) +1]) / 2.0f;
	}


	/**
	 * This method corrects its position by moving the robot in reverse
	 * @param x
	 * @param y
	 * @param t
	 */
	public void localizeBackward(double x, double y, double t) {

		//variables to be used when correcting odometer
		double leftColor;
		double rightColor;

		//Get intensity of regular surface
		double firstIntensityL = getMedianIntensity(leftColorSensor);
		double firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(LOCAL_SPEED);
		rightMotor.setSpeed(LOCAL_SPEED);
		leftMotor.setAcceleration(60000);
		rightMotor.setAcceleration(60000);
		leftMotor.backward();
		rightMotor.backward();

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
			rightMotor.backward();
			rightColor = getMedianIntensity(rightColorSensor);

			//run until right sensor sees line
			while (((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {
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
			leftMotor.backward();
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

		odometer.setX(x);
		odometer.setY(y);
		odometer.setTheta(t);
	}

	/**
	 * This method corrects its position by moving the robot forward
	 * @param x
	 * @param y
	 * @param t
	 */
	public void localizeForward(double x, double y, double t) {

		//variables to be used when correcting odometer
		double leftColor;
		double rightColor;

		//Get intensity of regular surface
		double firstIntensityL = getMedianIntensity(leftColorSensor);
		double firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(LOCAL_SPEED);
		rightMotor.setSpeed(LOCAL_SPEED);
		leftMotor.setAcceleration(60000);
		rightMotor.setAcceleration(60000);
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
			while (((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {
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

		odometer.setX(x);
		odometer.setY(y);
		odometer.setTheta(t);
	}

}

