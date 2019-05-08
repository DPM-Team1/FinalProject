package ca.mcgill.ecse211.Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drop the rings once the robot arrives at the starting corner.
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */
public class RingDrop {

	/**motor for left wheel*/
	private EV3LargeRegulatedMotor leftMotor;
	/**motor for right wheel*/
	private EV3LargeRegulatedMotor rightMotor;
	/**left motor for arm*/
	private EV3LargeRegulatedMotor leftArmMotor;
	/**right motor for arm*/
	private EV3LargeRegulatedMotor rightArmMotor;

	/**
	 * This is the class constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param rightArmMotor
	 * @param leftArmMotor
	 * @param navigation
	 */
	public RingDrop(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor leftArmMotor, EV3LargeRegulatedMotor rightArmMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftArmMotor = leftArmMotor;
		this.rightArmMotor = rightArmMotor;
	}

	/**Speed of the motors when backing up*/
	private static final int BACKWARD_SPEED = 150;
	/**Speed of the arm motors when going down when dropping rings*/
	private static final int ARM_DROP_SPEED = 900;
	/**Speed of the arm motors when going up when dropping rings*/
	private static final int ARM_RAISE_SPEED = 900;
	/**Distance needed to back up to drop rings*/
	private static final int BACKWARD_DIST = 20;

	/**
	 * This method rotates the arm of the robot vigorously to throw off the rings
	 * The robot also backs up to ensure that the rings come off the arm
	 * It then returns to the starting corner
	 */
	public void dropRings() {

		leftMotor.setSpeed(BACKWARD_SPEED);
		rightMotor.setSpeed(BACKWARD_SPEED);

		leftArmMotor.setSpeed(ARM_DROP_SPEED);
		rightArmMotor.setSpeed(ARM_DROP_SPEED);
		leftArmMotor.setAcceleration(1000000);
		rightArmMotor.setAcceleration(1000000);

		leftArmMotor.rotateTo(-10, true);
		rightArmMotor.rotateTo(-10, false);
		leftArmMotor.stop();
		rightArmMotor.stop();


		leftMotor.rotate(Navigation.convertDistance(-BACKWARD_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(-BACKWARD_DIST),true);



		for (int i=0 ; i <= 15 ; i++){
			leftArmMotor.setSpeed(ARM_DROP_SPEED);
			rightArmMotor.setSpeed(ARM_DROP_SPEED);
			leftArmMotor.rotateTo(-10, true);
			rightArmMotor.rotateTo(-10, false);

			leftArmMotor.setSpeed(ARM_RAISE_SPEED);
			rightArmMotor.setSpeed(ARM_RAISE_SPEED);
			leftArmMotor.rotateTo(-40, true);
			rightArmMotor.rotateTo(-40, false);
		}

		leftMotor.rotate(Navigation.convertDistance(BACKWARD_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(BACKWARD_DIST),false);


		leftArmMotor.setSpeed(50);
		rightArmMotor.setSpeed(50);
		leftArmMotor.rotateTo(0, true);
		rightArmMotor.rotateTo(0, false);

		leftMotor.rotate(Navigation.convertDistance(-BACKWARD_DIST),true);
		rightMotor.rotate(Navigation.convertDistance(-BACKWARD_DIST),true);

	}




}
