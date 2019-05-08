package ca.mcgill.ecse211.Rings;

import ca.mcgill.ecse211.General.FinalProject;
import ca.mcgill.ecse211.Navigation.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to collect the ring of the tree once a ring has been detected.
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */
public class RingCollection {
	/**Motor speed when collecting rings off tree*/
	private static final int MOTOR_SPEED = 150;
	/**distance needed to advance to pick up ring*/
	private static final int RING_DIST = 3;//4
	
	/**Angle needed to collect blue rings on the bottom*/
	private static final int blueAngleLow = -9;
	/**Angle needed to collect green rings on the bottom*/
	private static final int greenAngleLow = -10;
	/**Angle needed to collect yellow rings on the bottom*/
	private static final int yellowAngleLow = -6;
	/**Angle needed to collect orange rings on the bottom*/
	private static final int orangeAngleLow = -8;
	
	/**Angle needed to collect blue rings on the top*/
	private static final int blueAngleTop = -50;
	/**Angle needed to collect green rings on the top*/
	private static final int greenAngleTop = -40;
	/**Angle needed to collect yellow rings on the top*/
	private static final int yellowAngleTop = -40;
	/**Angle needed to collect orange rings on the top*/
	private static final int orangeAngleTop = -40;
	
	
	/**motor for left wheel*/
	private static EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private static EV3LargeRegulatedMotor rightMotor;
	/**left motor for arm*/
	private static EV3LargeRegulatedMotor leftArmMotor;
	/**right motor for arm*/
	private static EV3LargeRegulatedMotor rightArmMotor;


	/**
	 * This is the class constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param armMotor
	 * @param navi
	 */
	public RingCollection(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor leftArmMotor, EV3LargeRegulatedMotor rightArmMotor) {

		RingCollection.leftMotor = leftMotor;
		RingCollection.rightMotor = rightMotor;
		RingCollection.leftArmMotor = leftArmMotor;
		RingCollection.rightArmMotor = rightArmMotor;
		
		RingCollection.leftArmMotor.setAcceleration(10000);
		RingCollection.rightArmMotor.setAcceleration(10000);
	}


	/**
	 * This method will try to collect a ring if it has been placed on the top row.
	 */
	public static void collectTop(int ringKey) {
		
		int collectionAngle = 0;
		
		if (ringKey == 1) {
			collectionAngle = blueAngleTop;
		} else if (ringKey == 2) {
			collectionAngle = greenAngleTop;
		} else if (ringKey == 3) {
			collectionAngle = yellowAngleTop;
		} else if (ringKey == 4) {
			collectionAngle = orangeAngleTop;
		} 
		
		if (ringKey == 4 || ringKey == 3 || ringKey == 2) {
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED*8);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED*8);
			
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.rotate(Navigation.convertDistance(0.75),true);
			rightMotor.rotate(Navigation.convertDistance(0.75),false);
			
			leftArmMotor.rotate(-15, true); 
			rightArmMotor.rotate(-15, false);
			
			leftArmMotor.rotateTo(collectionAngle, true); 
			rightArmMotor.rotateTo(collectionAngle, false);
			
			leftArmMotor.rotateTo(collectionAngle - 20, true); 
			rightArmMotor.rotateTo(collectionAngle - 20, false);
			
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
			
			leftMotor.rotate(Navigation.convertDistance(-0.75),true);
			rightMotor.rotate(Navigation.convertDistance(-0.75),false);
			
		} else {
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.rotate(Navigation.convertDistance(-RING_DIST),true);
			rightMotor.rotate(Navigation.convertDistance(-RING_DIST),false);
	
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
			leftArmMotor.rotateTo(collectionAngle, true);
			rightArmMotor.rotateTo(collectionAngle, false); 
			leftArmMotor.stop();
			rightArmMotor.stop();
	
			leftArmMotor.rotate(-3, true); 
			rightArmMotor.rotate(-3, true); 
			leftMotor.rotate(Navigation.convertDistance(3*RING_DIST),true);
			rightMotor.rotate(Navigation.convertDistance(3*RING_DIST),false);
			
			leftArmMotor.rotateTo(-50, true); //45
			rightArmMotor.rotateTo(-50, false);
			leftMotor.rotate(Navigation.convertDistance(-13.965),true);
			rightMotor.rotate(Navigation.convertDistance(-13.965),false);
		}
		
		
		if (ringKey == 4 || ringKey == 3 || ringKey == 2) {
			leftArmMotor.rotateTo(-50, true); //45
			rightArmMotor.rotateTo(-50, false);
			leftMotor.rotate(Navigation.convertDistance(-7.965),true);
			rightMotor.rotate(Navigation.convertDistance(-7.965),false);
		}

		leftArmMotor.rotateTo(-70, true);
		rightArmMotor.rotateTo(-70, true);		

	}


	/**
	 * This method will try to collect a ring if it has been placed on the bottom row.
	 */
	public static void collectBottom(int ringKey) {

		int collectionAngle = 0;
		
		if (ringKey == 1) {
			collectionAngle = blueAngleLow;
		} else if (ringKey == 2) {
			collectionAngle = greenAngleLow;
		} else if (ringKey == 3) {
			collectionAngle = yellowAngleLow;
		} else if (ringKey == 4) {
			collectionAngle = orangeAngleLow;
		} 
		
		//if (ringKey == 3 || ringKey == 4) {
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.rotate(Navigation.convertDistance(-RING_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(-RING_DIST),false);
		
		
		leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
		rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
		leftArmMotor.rotateTo(collectionAngle, true);//20
		rightArmMotor.rotateTo(collectionAngle, false);//20
		leftArmMotor.stop();
		rightArmMotor.stop();

		leftMotor.rotate(Navigation.convertDistance(1.5*RING_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(1.5*RING_DIST),false);
		
		leftArmMotor.rotateTo(-23, true);
		rightArmMotor.rotateTo(-23, false);
		leftArmMotor.rotate(-3, true);
		rightArmMotor.rotate(-3, true);
		leftMotor.rotate(Navigation.convertDistance(-2.47*RING_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(-2.47*RING_DIST),false);

		leftArmMotor.rotateTo(-70, true);
		rightArmMotor.rotateTo(-70, true);

		//}
		
	/*	if (ringKey == 1 || ringKey == 2) {
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED*8);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED*8);
			
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			//leftMotor.rotate(Navigation.convertDistance(0.75),true);
			//rightMotor.rotate(Navigation.convertDistance(0.75),false);
			
			leftArmMotor.rotate(-3, true); 
			rightArmMotor.rotate(-3, false);
			
			leftArmMotor.rotateTo(-8, true); 
			rightArmMotor.rotateTo(-8, false);
			
			leftMotor.rotate(Navigation.convertDistance(-5.7),true);
			rightMotor.rotate(Navigation.convertDistance(-5.7),false);
			
			leftArmMotor.rotateTo(-12, true); 
			rightArmMotor.rotateTo(12, false);
			
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
		}*/
	}

}
