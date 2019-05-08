package ca.mcgill.ecse211.Rings;

import ca.mcgill.ecse211.Navigation.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.General.*;
import ca.mcgill.ecse211.Localization.LightLocalization;

/**
 * This class is used to detect where the rings are hung on the tree.
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */
public class RingDetection {
	/**Odometer*/
	private Odometer odo;
	/**motor for left wheel*/
	private static EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private static EV3LargeRegulatedMotor rightMotor;
	/**left motor for arm*/
	private static EV3LargeRegulatedMotor leftArmMotor;
	/**right motor for arm*/
	private static EV3LargeRegulatedMotor rightArmMotor;
	/**Navigation*/
	private static Navigation navi;
	/**Sample provider for the light sensor in front of the left wheel*/
	private SampleProvider leftColorSensor;
	/**Sample provider for the light sensor in front of the right wheel*/
	private SampleProvider rightColorSensor;

	/**Motor speed when collecting rings off tree*/
	private static final int MOTOR_SPEED = 120;
	/**Distance needed to advance to detect color*/
	private static final double TREE_DISTANCE = 5.9; //7


	/**
	 * This is the class constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftArmMotor
	 * @param rightArmMotor
	 * @param navi
	 * @param leftColorSensor
	 * @param rightColorSensor
	 */

	public RingDetection(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor leftArmMotor, EV3LargeRegulatedMotor rightArmMotor, Navigation navi,
			SampleProvider leftColorSensor, SampleProvider rightColorSensor) {
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftArmMotor = leftArmMotor;
		this.rightArmMotor = rightArmMotor;
		this.navi = navi;
		this.leftColorSensor = leftColorSensor;
		this.rightColorSensor = rightColorSensor;
	}



	/**
	 * This method is used to move around the tree, all while scanning for rings.
	 * It will verify all 4 sides of the tree to collect a ring
	 * It will start by looking for bottom ring
	 * If a ring was found, it picks it up and goes to the next side
	 * If a ring wasn't found, it looks for a top ring
	 * If a ring a found it picks it up and goes to the next side
	 * If a ring wasn't found it goes to the next side.
	 */
	@SuppressWarnings("static-access")
	public void goAroundTree(SensorMode brightnessSensorMode) {

		RingCollection ringCol = new RingCollection(leftMotor, rightMotor, leftArmMotor, rightArmMotor);
		int sidesCompleted = 0;
		int ringColor;
		double startingX = odo.getX();
		double startingY = odo.getY();
		double startingT = odo.getTheta();

		while (sidesCompleted != 4) {

			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
			leftArmMotor.rotateTo(-20, true);
			rightArmMotor.rotateTo(-20, false);
			leftArmMotor.stop();
			rightArmMotor.stop();

			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.rotate(navi.convertDistance(TREE_DISTANCE),true);
			rightMotor.rotate(navi.convertDistance(TREE_DISTANCE),false);

			ringColor = (int) ColorIdentification.meanRingColor(brightnessSensorMode)[0];

			if (ringColor != 0) {
				ringCol.collectBottom(ringColor);
			}

			else {
				leftMotor.rotate(navi.convertDistance(-TREE_DISTANCE),true);//ARM_MOV
				rightMotor.rotate(navi.convertDistance(-TREE_DISTANCE),false);
			}

			leftArmMotor.rotateTo(-55, true);
			rightArmMotor.rotateTo(-55, false);
			
			leftArmMotor.setSpeed(150);
			rightArmMotor.setSpeed(150);
			leftArmMotor.rotate(-5);
			rightArmMotor.rotate(-5);
			leftArmMotor.rotate(5);
			rightArmMotor.rotate(5);
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
			
			leftArmMotor.stop();
			rightArmMotor.stop();

			if (ringColor == 0) {
				leftMotor.rotate(navi.convertDistance(TREE_DISTANCE*1.35),true); //ARM_MOV
				rightMotor.rotate(navi.convertDistance(TREE_DISTANCE*1.35),false);

				ringColor = ColorIdentification.relativeRingColor(brightnessSensorMode);

				if (ringColor != 0) {
					ringCol.collectTop(ringColor);
				}
				else {
					leftMotor.rotate(navi.convertDistance(-TREE_DISTANCE*1.35),true);
					rightMotor.rotate(navi.convertDistance(-TREE_DISTANCE*1.35),false);
				}
			}
			
			
			leftArmMotor.rotateTo(-70, true);
			rightArmMotor.rotateTo(-70, false);
			
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED*6);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED*6);
			leftArmMotor.rotate(-5, true);
			rightArmMotor.rotate(-5, false);
			leftArmMotor.rotate(5, true);
			rightArmMotor.rotate(5, true);
			leftArmMotor.setSpeed(FinalProject.ARM_SPEED);
			rightArmMotor.setSpeed(FinalProject.ARM_SPEED);
			
			leftArmMotor.stop();
			rightArmMotor.stop();
			
			leftArmMotor.rotateTo(-10, true);
			rightArmMotor.rotateTo(-10, false);
			leftArmMotor.rotateTo(-70, true);
			rightArmMotor.rotateTo(-70, false);

			leftMotor.rotate(navi.convertDistance(LightLocalization.lightSensorDistanceY),true);
			rightMotor.rotate(navi.convertDistance(LightLocalization.lightSensorDistanceY),false);

			leftMotor.setSpeed(300);
			leftMotor.setSpeed(300);
			leftMotor.rotate(navi.convertAngle(45), true);
			rightMotor.rotate(-navi.convertAngle(45), false);

			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);
			leftMotor.rotate(navi.convertDistance(Math.sqrt(2*(FinalProject.TILE_SIZE * FinalProject.TILE_SIZE))),true);
			rightMotor.rotate(navi.convertDistance(Math.sqrt(2*(FinalProject.TILE_SIZE * FinalProject.TILE_SIZE))),false);

			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.rotate(-navi.convertAngle(45), true);
			rightMotor.rotate(navi.convertAngle(45), false);


			sidesCompleted++;

			leftMotor.rotate(navi.convertDistance(3),true);
			rightMotor.rotate(navi.convertDistance(3),false);

			if ((Math.abs(startingT - 0 ) < 5) || (Math.abs(startingT - 360 ) < 5)) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE), (startingY + FinalProject.TILE_SIZE), 0);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward(startingX + LightLocalization.lightSensorDistanceY, 
							(startingY + (FinalProject.TILE_SIZE)*2 + LightLocalization.lightSensorDistanceY), 270);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE), 
							(startingY + FinalProject.TILE_SIZE + (LightLocalization.lightSensorDistanceY)*2), 180);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX - LightLocalization.lightSensorDistanceY, 
							startingY + LightLocalization.lightSensorDistanceY, 90);
				}
			}

			if (Math.abs(startingT - 90 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE), (startingY - FinalProject.TILE_SIZE), 90);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward((startingX + (FinalProject.TILE_SIZE)*2 + LightLocalization.lightSensorDistanceY), 
							startingY - LightLocalization.lightSensorDistanceY, 0);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE + (LightLocalization.lightSensorDistanceY)*2), 
							(startingY + FinalProject.TILE_SIZE), 270);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX + LightLocalization.lightSensorDistanceY, 
							startingY + LightLocalization.lightSensorDistanceY, 180);
				}
			}

			if (Math.abs(startingT - 180 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE), (startingY - FinalProject.TILE_SIZE), 180);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward(startingX - LightLocalization.lightSensorDistanceY, 
							(startingY - (FinalProject.TILE_SIZE)*2 - LightLocalization.lightSensorDistanceY), 90);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE), 
							(startingY - FinalProject.TILE_SIZE - (LightLocalization.lightSensorDistanceY)*2), 0);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX + LightLocalization.lightSensorDistanceY, 
							startingY - LightLocalization.lightSensorDistanceY, 270);
				}
			}

			if (Math.abs(startingT - 270 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE), (startingY + FinalProject.TILE_SIZE), 270);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward((startingX - (FinalProject.TILE_SIZE)*2 - LightLocalization.lightSensorDistanceY), 
							startingY + LightLocalization.lightSensorDistanceY, 180);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE - (LightLocalization.lightSensorDistanceY)*2), 
							(startingY - FinalProject.TILE_SIZE), 90);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX - LightLocalization.lightSensorDistanceY, 
							startingY - LightLocalization.lightSensorDistanceY, 0);
				}
			}


			leftMotor.rotate(navi.convertDistance(LightLocalization.lightSensorDistanceY+1), true);
			rightMotor.rotate(navi.convertDistance(LightLocalization.lightSensorDistanceY+1), false);
			leftMotor.rotate(-navi.convertAngle(90), true);
			rightMotor.rotate(navi.convertAngle(90), false);

			leftMotor.rotate(navi.convertDistance(2),true);
			rightMotor.rotate(navi.convertDistance(2),false);

			if ((Math.abs(startingT - 0 ) < 5) || (Math.abs(startingT - 360 ) < 5)) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(startingY + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 270);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward(startingX, 
							(startingY + (FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY)*2), 180);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(startingY + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 90);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX, startingY, 0);
				}
			}

			if (Math.abs(startingT - 90 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(startingY - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward((startingX + (FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY)*2), 
							startingY, 270);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(startingY + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX, startingY, 90);
				}
			}

			if (Math.abs(startingT - 180 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(startingY - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 90);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward(startingX, 
							(startingY - (FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY)*2), 0);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 
							(startingY - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 270);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX, startingY, 180);
				}
			}

			if (Math.abs(startingT - 270 ) < 5) {
				if (sidesCompleted == 1) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(startingY + FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY), 180);
				}
				if (sidesCompleted == 2) {
					navi.localizeBackward((startingX - (FinalProject.TILE_SIZE + LightLocalization.lightSensorDistanceY)*2), 
							startingY, 90);
				}
				if (sidesCompleted == 3) {
					navi.localizeBackward((startingX - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 
							(startingY - FinalProject.TILE_SIZE - LightLocalization.lightSensorDistanceY), 0);
				}
				if (sidesCompleted == 4) {
					navi.localizeBackward(startingX, startingY, 270);
				}
			}
		}	
	}


}
