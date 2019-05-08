package ca.mcgill.ecse211.General;

//imports
import ca.mcgill.ecse211.Localization.*;
import ca.mcgill.ecse211.Navigation.*;
import ca.mcgill.ecse211.Rings.*;
import ca.mcgill.ecse211.General.Wifi;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Import of project to github
 * This is the main class of the program
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */

public class FinalProject {
	//initialization of motors and sensors
	/**motor for left wheel*/
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	/**motor for right wheel*/
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	/**left motor for arm*/
	private static final EV3LargeRegulatedMotor leftArmMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	/**right motor for arm*/
	private static final EV3LargeRegulatedMotor rightArmMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	/**port for ultrasonic sensor*/
	private static final Port usPort = LocalEV3.get().getPort("S3");
	/**port for light sensor in front of the left wheel*/
	private static final Port leftPort = LocalEV3.get().getPort("S4");	
	/**port for light sensor in front of the right wheel*/
	private static final Port rightPort = LocalEV3.get().getPort("S1");	
	/**sensor used to detect the color of the rings*/
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);	


	//initialization of global variables
	/**wheel radius*/
	public static final double WHEEL_RAD = 2.09;	//2.1	//Wheel radius
	/**wheel base*/
	public static final double TRACK = 11.674;	//11.73	//wheel base
	/**tile size*/
	public static final double TILE_SIZE = 30.48;
	/**arm motor speed*/
	public static final int ARM_SPEED = 50;
	/**Orientation of the tunnel*/
	public static boolean HORIZONTAL = false;
	/**Location of the tree relative to the tunnel exit*/
	public static boolean LEFT = false;

	/**Green Team number*/
	public static int GreenTeam = 0;
	/**Green Starting Corner*/
	public static int GreenCorner = 0;

	/**Lower Left x coordinate of the green zone*/
	public static int GREEN_LL_X = 0;
	/**Lower Left y coordinate of the green zone*/
	public static int GREEN_LL_Y = 0;
	/**Upper right x coordinate of the green zone*/
	public static int GREEN_UR_X = 0;
	/**Upper right y coordinate of the green zone*/
	public static int GREEN_UR_Y = 0;

	/**Lower Left x coordinate of the island*/
	public static int ISLAND_LL_X = 0;
	/**Lower Left y coordinate of the island*/
	public static int ISLAND_LL_Y = 0;
	/**Upper right x coordinate of the island*/
	public static int ISLAND_UR_X = 0;
	/**Upper right y coordinate of the island*/
	public static int ISLAND_UR_Y = 0;

	/**Lower Left x coordinate of the green tunnel*/
	public static int TNG_LL_X = 0;
	/**Lower Left y coordinate of the green tunnel*/
	public static int TNG_LL_Y = 0;
	/**Upper right x coordinate of the green tunnel*/
	public static int TNG_UR_X = 0;
	/**Upper right y coordinate of the green tunnel*/
	public static int TNG_UR_Y = 0;

	/**x coordinate of the green tree*/
	public static int TG_X = 0;
	/**y coordinate of the green tree*/
	public static int TG_Y = 0;


	/**
	 * This main method of the final project
	 *  - Initialization of all sensors
	 *  - Arm Motors locked to keep position
	 *  - Get the location variables with WiFi class
	 *  - Ultrasonic Localization
	 *  - Light Localization
	 *  - Determination of the points the robot needs to travel to with
	 *    the starting corner, the position of the tunnel and the position of the tree
	 *  - Navigation to Tunnel
	 *  - Navigation Through Tunnel
	 *  - Navigation to Tree
	 *  - Go around tree to pick up rings
	 *  - Go to tunnel
	 *  - Go back through tunnel
	 *  - Go to starting corner
	 *  - Drop Rings
	 *  - End program
	 *
	 * @param args Not used.
	 */
	@SuppressWarnings("resource")
	public static void main(String[] args) {


		// Setting up the Ultrasonic Sensor							    // Because we don't bother to close this resource
		EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usValue = usSensor.getMode("Distance");		// usValue provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];			// usData is the buffer in which data are returned

		//Setting up the left Color Sensor
		SensorModes leftSensor = new EV3ColorSensor(leftPort);		// leftSensor is the instance
		SampleProvider leftColorValue = leftSensor.getMode("Red");			// leftColorValue provides samples from this instance
		float[] leftColorData = new float[leftColorValue.sampleSize()];			// leftColorData is the buffer in which data are returned

		//Setting up the right Color Sensor
		SensorModes rightSensor = new EV3ColorSensor(rightPort);		// rightSensor is the instance
		SampleProvider rightColorValue = rightSensor.getMode("Red");			// rightColorValue provides samples from this instance
		float[] rightColorData = new float[rightColorValue.sampleSize()];			// rightColorData is the buffer in which data are returned

		colorSensor.setFloodlight(false);
		SensorMode brightnessSensorMode = colorSensor.getRGBMode();


		//Setting up display + instantiation
		LocalOdometer odoL = new LocalOdometer(leftMotor, rightMotor, 30, true);

		//lock arm motors
		leftArmMotor.setSpeed(ARM_SPEED);
		rightArmMotor.setSpeed(ARM_SPEED);
		leftArmMotor.rotateTo(-70, true);
		rightArmMotor.rotateTo(-70, false);
		leftArmMotor.stop();
		rightArmMotor.stop();

		//get wifi variables
		Wifi.WifiVariables();

		//Ultrasonic Localization
		UltrasonicLocalization usl = new UltrasonicLocalization(odoL, usValue, usData, 
				UltrasonicLocalization.LocalizationMethod.FALLING_EDGE);
		usl.doLocalization();
		
		usSensor.close();
		
		//Light Localization
		LocalNavigation naviL = new LocalNavigation(odoL);
		LightLocalization lsl = new LightLocalization(odoL, leftColorValue, leftColorData, 
				rightColorValue, rightColorData, naviL);
		lsl.doLocalization();	

		Sound.beep();
		Sound.beep();
		Sound.beep();


		//Initialization of Navigation odometer
		Odometer odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		odo.start();
		Navigation navi = new Navigation(odo, leftMotor, rightMotor, leftColorValue, rightColorValue, 
				leftArmMotor, rightArmMotor);


		if ((TNG_UR_X - TNG_LL_X) > (TNG_UR_Y - TNG_LL_Y)) {
			HORIZONTAL = true;
		}
		if (GreenCorner == 0 || GreenCorner == 3) {
			if (GREEN_UR_X <= ISLAND_LL_X) {
				HORIZONTAL = true;
			}
		}
		if (GreenCorner == 1 || GreenCorner == 2) {
			if (GREEN_LL_X >= ISLAND_UR_X) {
				HORIZONTAL = true;
			}
		}


		boolean toTree = true;

		if (GreenCorner == 0) {
			odo.setX(TILE_SIZE);
			odo.setY(TILE_SIZE);
			odo.setTheta(0);

			if (HORIZONTAL) {
				navi.travelTo((double)TNG_LL_X - 1, (double)TNG_LL_Y);
				navi.travelThroughTunnel((double)TNG_UR_X + 0.75, (double)TNG_LL_Y + 0.5, toTree);
				navi.travelToTree((double)TG_X - 1.0, (double)TG_Y);
			}
			else {
				navi.travelTo((double)TNG_LL_X, (double)TNG_LL_Y - 1);
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_UR_Y + 0.75, toTree);
				navi.travelToTree((double)TG_X, (double)TG_Y - 1.0);
			}
		}

		if (GreenCorner == 1) {
			
			odo.setX(14*TILE_SIZE);
			odo.setY(TILE_SIZE);
			 
			//odo.setX(7*TILE_SIZE);
			//odo.setY(TILE_SIZE);
			odo.setTheta(270);

			if (HORIZONTAL) {
				navi.travelTo((double)TNG_UR_X + 1, (double)TNG_LL_Y);
				navi.travelThroughTunnel((double)TNG_LL_X - 0.75, (double)TNG_LL_Y + 0.5, toTree);
				navi.travelToTree((double)TG_X + 1.0, (double)TG_Y);
			}
			else {
				navi.travelTo((double)TNG_UR_X, (double)TNG_LL_Y - 1);
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_UR_Y + 0.75, toTree);
				navi.travelToTree((double)TG_X, (double)TG_Y - 1.0);
			}
		}

		if (GreenCorner == 2) {
			
			odo.setX(14*TILE_SIZE);
			odo.setY(8*TILE_SIZE);
			 
			//odo.setX(7*TILE_SIZE);
			//odo.setY(7*TILE_SIZE);
			odo.setTheta(180);

			if (HORIZONTAL) {
				navi.travelTo((double)TNG_UR_X + 1, (double)TNG_UR_Y);
				navi.travelThroughTunnel((double)TNG_LL_X - 0.75, (double)TNG_LL_Y + 0.5, toTree);
				navi.travelToTree((double)TG_X + 1.0, (double)TG_Y);
			}
			else {
				navi.travelTo((double)TNG_UR_X, (double)TNG_UR_Y + 1);
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_LL_Y - 0.75, toTree);
				navi.travelToTree((double)TG_X, (double)TG_Y + 1.0);
			}
		}

		if (GreenCorner == 3) {
			
			odo.setX(TILE_SIZE);
			odo.setY(8*TILE_SIZE);
			 
			//odo.setX(TILE_SIZE);
			//odo.setY(7*TILE_SIZE);
			odo.setTheta(90);

			if (HORIZONTAL) {
				navi.travelTo((double)TNG_LL_X - 1, (double)TNG_UR_Y);
				navi.travelThroughTunnel((double)TNG_UR_X + 0.75, (double)TNG_LL_Y + 0.5, toTree);
				navi.travelToTree((double)TG_X - 1.0, (double)TG_Y );
			}
			else {
				navi.travelTo((double)TNG_LL_X, (double)TNG_UR_Y + 1);
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_LL_Y - 0.75, toTree);
				navi.travelToTree((double)TG_X, (double)TG_Y + 1.0);
			}
		}

		
		Sound.beep();
		Sound.beep();
		Sound.beep();

		
		RingDetection findRing = new RingDetection(odo, leftMotor, rightMotor, leftArmMotor, rightArmMotor, 
				navi, leftColorValue, rightColorValue);
		findRing.goAroundTree(brightnessSensorMode);
		

		colorSensor.close();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		

		toTree = false;

		if (HORIZONTAL) {
			if (GreenCorner == 0 && GreenCorner == 3) {
				if (TNG_UR_Y <= TG_X) {
					LEFT = true;
				}
			}	
			if (GreenCorner == 1 && GreenCorner == 2) {
				if (TNG_LL_Y >= TG_X) {
					LEFT = true;
				}
			}
		}
		else {
			if (GreenCorner == 0 && GreenCorner == 1) {
				if (TNG_UR_X >= TG_X) {
					LEFT = true;
				}
			}	
			if (GreenCorner == 2 && GreenCorner == 3) {
				if (TNG_UR_X <= TG_X) {
					LEFT = true;
				}
			}
		}


		//go back
		if (GreenCorner == 0) {
			if (HORIZONTAL) {
				if (LEFT) {
					navi.travelTo((double)TNG_UR_X + 1, (double)TNG_UR_Y);
				}
				else {
					navi.travelTo((double)TNG_UR_X + 1, (double)TNG_LL_Y);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X - 0.75, (double)TNG_LL_Y + 0.5, toTree);
			}
			else {
				if (LEFT) {
					navi.travelTo((double)TNG_LL_X, (double)TNG_UR_Y + 1);
				}
				else {
					navi.travelTo((double)TNG_UR_X, (double)TNG_UR_Y + 1);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_LL_Y - 0.75, toTree);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			navi.travelTo(0.75, 0.75);
		}

		if (GreenCorner == 1) {
			if (HORIZONTAL) {
				if (LEFT) {
					navi.travelTo((double)TNG_LL_X - 1, (double)TNG_LL_Y);
				}
				else {
					navi.travelTo((double)TNG_LL_X - 1, (double)TNG_UR_Y);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_UR_X + 0.75, (double)TNG_LL_Y + 0.5, toTree);
			}
			else {
				if (LEFT) {
					navi.travelTo((double)TNG_LL_X, (double)TNG_UR_Y + 1);
				}
				else {
					navi.travelTo((double)TNG_UR_X, (double)TNG_UR_Y + 1);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_LL_Y - 0.75, toTree);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			navi.travelTo(14.25, 0.75);
			//navi.travelTo(7.25, 0.75);
		}

		if (GreenCorner == 2) {
			if (HORIZONTAL) {
				if (LEFT) {
					navi.travelTo((double)TNG_LL_X - 1, (double)TNG_LL_Y);
				}
				else {
					navi.travelTo((double)TNG_LL_X - 1, (double)TNG_UR_Y);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_UR_X + 0.75, (double)TNG_LL_Y + 0.5, toTree);
			}
			else {
				if (LEFT) {
					navi.travelTo((double)TNG_UR_X, (double)TNG_LL_Y - 1);
				}
				else {
					navi.travelTo((double)TNG_LL_X, (double)TNG_LL_Y - 1);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_UR_Y + 0.75, toTree);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			navi.travelTo(14.25, 8.25);
			//navi.travelTo(7.25, 7.25);
		}


		if (GreenCorner == 3) {
			if (HORIZONTAL) {
				if (LEFT) {
					navi.travelTo((double)TNG_UR_X + 1, (double)TNG_UR_Y);
				}
				else {
					navi.travelTo((double)TNG_UR_X + 1, (double)TNG_LL_Y);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X - 0.75, (double)TNG_LL_Y + 0.5, toTree);
			}
			else {
				if (LEFT) {
					navi.travelTo((double)TNG_UR_X, (double)TNG_LL_Y - 1);
				}
				else {
					navi.travelTo((double)TNG_LL_X, (double)TNG_LL_Y - 1);
				}
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				navi.travelThroughTunnel((double)TNG_LL_X + 0.5, (double)TNG_UR_Y + 0.75, toTree);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			navi.travelTo(0.75, 8.25);
			//navi.travelTo(0.75, 7.25);
		}


		RingDrop ringDrop = new RingDrop(leftMotor, rightMotor, leftArmMotor, rightArmMotor);
		ringDrop.dropRings();



		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();



		//Terminate
		System.exit(0);	

	}


}

