package ca.mcgill.ecse211.General;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;


/**
 * This is the class used to get the starting variables from the server.
 * We preset the server IP address and the team number.
 * We get the location variables for both teams with the server.
 * 
 * @author Michael Smith, Tharsan Ponnampalam
 * @author Garine Imamedjian
 * @author Rebecca Weill
 * @author Alexa Normandin
 * @author Sam Lantela
 * @author Amine Alikacem
 * @author Awais Shahid
 *
 */
public class Wifi {

  /**Server IP address*/
  private static final String SERVER_IP = "192.168.2.5";
  /**Green Team Number*/
  private static final int TEAM_NUMBER = 1;

  /** Enable/disable printing of debug info from the WiFi class*/
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /** This method gets the variables from the server and assigns them to our main location variables  */
  @SuppressWarnings("rawtypes")
  public static void WifiVariables() {


    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();

      FinalProject.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
      if (FinalProject.GreenTeam != TEAM_NUMBER) {
    	  System.exit(0);
      }
      FinalProject.GreenCorner = ((Long) data.get("GreenCorner")).intValue();
      
      FinalProject.GREEN_LL_X = ((Long) data.get("Green_LL_x")).intValue();
      FinalProject.GREEN_LL_Y = ((Long) data.get("Green_LL_y")).intValue();
      FinalProject.GREEN_UR_X = ((Long) data.get("Green_UR_x")).intValue();
      FinalProject.GREEN_UR_Y = ((Long) data.get("Green_UR_y")).intValue();
      
      FinalProject.ISLAND_LL_X = ((Long) data.get("Island_LL_x")).intValue();
      FinalProject.ISLAND_LL_Y = ((Long) data.get("Island_LL_y")).intValue();
      FinalProject.ISLAND_UR_X = ((Long) data.get("Island_UR_x")).intValue();
      FinalProject.ISLAND_UR_Y = ((Long) data.get("Island_UR_y")).intValue();
      
      FinalProject.TNG_LL_X = ((Long) data.get("TNG_LL_x")).intValue();
      FinalProject.TNG_LL_Y = ((Long) data.get("TNG_LL_y")).intValue();
      FinalProject.TNG_UR_X = ((Long) data.get("TNG_UR_x")).intValue();
      FinalProject.TNG_UR_Y = ((Long) data.get("TNG_UR_y")).intValue();
      
      FinalProject.TG_X = ((Long) data.get("TG_x")).intValue();
      FinalProject.TG_Y = ((Long) data.get("TG_y")).intValue();
      


    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

    // Wait until user decides to end program
   // Button.waitForAnyPress();
  }
}

