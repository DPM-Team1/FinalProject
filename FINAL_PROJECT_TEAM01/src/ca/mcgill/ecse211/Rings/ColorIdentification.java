package ca.mcgill.ecse211.Rings;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.SensorMode;




/**
 * This class  is used to detect the color of the ring on the tree. 
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 */

public class ColorIdentification {

	static double blueRThresholdB = 0.15; //0.2 //0.23 //0.25 0.27 &&
	static double blueRThresholdG = 0.25; //0.30 //0.34 //0.36 0.28
	static double yellowRThresholdR = 0.025; //0.030 //0.038 0.036 0.025
	
	// Model data for each ring color, as mean and standard deviation respectively.
	static double orangeRGB [][] = {{80, 22.86},{22, 6.75},{3.6, 4.65}};
	static double yellowRGB [][] = {{112.3, 46.65},{87, 34.15},{9.4, 10.12}};
	static double greenRGB [][] = {{33, 13.50},{86.7, 27.06},{8.3, 6.98}};
	static double blueRGB [][] = {{12.2, 9.19},{84, 28.53},{52.5, 19.10}};
	
	//MODEL DATA using scaled percentage RGB and SD values
	/*static double orangeRGB [][] = {{75.76, 21.65},{20.83, 6.39},{3.41, 4.40}};
	static double yellowRGB [][] = {{53.81, 22.35},{41.69, 16.36},{4.50, 4.84}};
	static double greenRGB [][] = {{25.78, 10.55},{67.73, 21.14},{6.48, 5.45}};
	static double blueRGB [][] = {{8.20, 6.18},{56.50, 19.19},{35.31, 12.84}};*/


	/**
	 * This method is used to detect if there is a ring on the tree in that position.
	 * If there is a ring, we need to identify the color.
	 * It is also used to beep the correct amount of times to get our points.
	 * This method compares our two other methods to determine which one gave us the best answer
	 */
	public static int ringColor(SensorMode brightnessSensorMode) {
		int result = 0;
		
		LCD.clear();
		
		int relColor = relativeRingColor(brightnessSensorMode);
		double meanColor[] = meanRingColor(brightnessSensorMode);
		
		if (relColor == meanColor[0]) {
			result = relColor;
		} else if (meanColor[1] <= 1) {// Color within 1 standard deviation, trust accuracy. (meanColor[1] <= 1)
			result = (int) meanColor[0];
		} else if (relColor == 2 && meanColor[0] == 3) {
			result = 2;
		} else if (relColor == 0 && meanColor[0] == 1) {
			if (meanColor[1] <= 2.7) {
				result = 1;
			} else {
				result = 0;
			}	
		} else if (relColor == 4 && meanColor[0] == 3) { //SD >3
			if (meanColor[1] >= 3.5) {
				result = 0;
			} else {
				result = 4;
			}	
		} else {
			result = relColor;
		} /*else {
			result = (int) meanColor[0];
		} /*else if (relColor == 3){
			result = 3;
		} else {
			result = relColor;
		}*/
		
		for (int i = 0; i < result; i++) {
			Sound.beep();
		}
		
		return result;
	}


	/**
	 * This method identifies the color of the ring by comparing it to the mean calculated previously during tests
	 * It returns the colors it is closest too and the standard deviation from the mean
	 * This is most useful for bottom level rings
	 */
	public static double[] meanRingColor(SensorMode brightnessSensorMode) {
		int color = 0; // Blue = 1; Green = 2; Yellow = 3; Orange = 4;
		double stdDev = 100; // to determine certainty by number of standard deviations (initialize to 100 as sentinel)
		
		//prepare sensor to read color values
		LCD.setAutoRefresh(true);
		float[] sample = new float[brightnessSensorMode.sampleSize()];
		float RP, GP, BP;
		LCD.clear();
		
		//get the color sample
		brightnessSensorMode.fetchSample(sample, 0);
		//sleep a bit
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		LCD.refresh();
		LCD.clear();
		
		RP = sample[0]*1000;
		GP = sample[1]*1000;
		BP = sample[2]*1000;
		
		//
		double devBlue = Math.sqrt((RP - blueRGB[0][0])*(RP - blueRGB[0][0])/blueRGB[0][1]/blueRGB[0][1] + (GP - blueRGB[1][0])*(GP - blueRGB[1][0])/blueRGB[1][1]/blueRGB[1][1] + (BP - blueRGB[2][0])*(BP - blueRGB[2][0])/blueRGB[2][1]/blueRGB[2][1]);
		double devGreen = Math.sqrt((RP - greenRGB[0][0])*(RP - greenRGB[0][0])/greenRGB[0][1]/greenRGB[0][1] + (GP - greenRGB[1][0])*(GP - greenRGB[1][0])/greenRGB[1][1]/greenRGB[1][1] + (BP - greenRGB[2][0])*(BP - greenRGB[2][0])/greenRGB[2][1]/greenRGB[2][1]);
		double devYellow = Math.sqrt((RP - yellowRGB[0][0])*(RP - yellowRGB[0][0])/yellowRGB[0][1]/yellowRGB[0][1] + (GP - yellowRGB[1][0])*(GP - yellowRGB[1][0])/yellowRGB[1][1]/yellowRGB[1][1] + (BP - yellowRGB[2][0])*(BP - yellowRGB[2][0])/yellowRGB[2][1]/yellowRGB[2][1]);
		double devOrange = Math.sqrt((RP - orangeRGB[0][0])*(RP - orangeRGB[0][0])/orangeRGB[0][1]/orangeRGB[0][1] + (GP - orangeRGB[1][0])*(GP - orangeRGB[1][0])/orangeRGB[1][1]/orangeRGB[1][1] + (BP - orangeRGB[2][0])*(BP - orangeRGB[2][0])/orangeRGB[2][1]/orangeRGB[2][1]);
		
		double deviation[] = {devBlue, devGreen, devYellow, devOrange};
		
		int smallestIndex = 0;
		
		for (int i = 1 ; i<4 ; i++) {
			if (deviation[i] < deviation[smallestIndex]) {
				smallestIndex = i;
			}
		}
		
		color = smallestIndex  + 1;
		stdDev = deviation[smallestIndex];
		
		if (color == 3 && stdDev > 3) {
			if (devOrange > 3.8) {
				color = 0;
			} else {
				color = 4;
			}
		}		
		
		double result[] = {color, stdDev};
		//LCD.clear();
		LCD.drawString("Object Detected: "+ color , 0, 2);
		LCD.drawString("SD: " + stdDev, 0, 3);
		
		for (int i = 0; i < color; i++) {
			Sound.beep();
		}
		
		return result;
	}
	
	/**
	 * This method identifies the color of the ring by calculating the percentage of R, G, and B in the RGB color code
	 * This is most useful for top level rings
	 */
	public static int relativeRingColor(SensorMode brightnessSensorMode) {
		float[] sample = new float[brightnessSensorMode.sampleSize()];
		float RP, GP, BP;
		int colorIdentified = 0;
		LCD.clear();


		//get the color sample
		brightnessSensorMode.fetchSample(sample, 0);
		//sleep a bit
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		//LCD.refresh();
		//LCD.clear();

		//get the percentage of R G and B colors
		RP = sample[0] / (sample[0] + sample[1] + sample[2]);
		GP = sample[1] / (sample[0] + sample[1] + sample[2]);
		BP = sample[2] / (sample[0] + sample[1] + sample[2]);

		//green case
		if (GP > RP && GP > BP && RP > BP) {
			LCD.drawString("Object Detected:", 0, 0);
			LCD.drawString("Green", 0, 1);
			Sound.beep();
			Sound.beep();
			colorIdentified = 2;
		}

		//blue case
		else if (GP > RP && GP > BP && BP > RP) {
			LCD.drawString("Object Detected:", 0, 0);
			LCD.drawString("Blue", 0, 1);
			Sound.beep();
			colorIdentified = 1;
		}

		//orange case
		else if (RP > GP && RP > BP && RP > 0.7) {
			LCD.drawString("Object Detected:", 0, 0);
			LCD.drawString("Orange", 0, 1);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			colorIdentified = 4;
		}

		//yellow case
		else if (RP > GP && RP > BP && RP < 0.7) {
			if (sample[0] > yellowRThresholdR) {
				LCD.drawString("Object Detected:", 0, 0);
				LCD.drawString("Yellow", 0, 1);
				Sound.beep();
				Sound.beep();
				Sound.beep();
				colorIdentified = 3;
			}	
		}
		
		if (colorIdentified == 0 && (GP > blueRThresholdG && BP > blueRThresholdB)) {
			colorIdentified = 1;
			Sound.beep();
		}
		
		return colorIdentified;

	}






}
