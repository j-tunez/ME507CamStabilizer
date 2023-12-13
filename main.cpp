/** @file main.cpp
 * This file is uused to run a multi-tasking program designed to stabilize a platform
 * using an 6-DOF IMU and 3 motors.
 * 
 * @author Jathun Somasundaram
 * @date 2023-Dec-11 
 * 
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "IMU.h"
#include "motor_obj.h"
#include "taskqueue.h"
#include "mycerts.h"

#define USE_LAN

uint16_t MPU_ADDR = 0x68; ///< I2C address of the MPU-6050
uint16_t I2C_SDA = 23;    ///< I2C data pin
uint16_t I2C_SCL = 22;    ///< I2C clock pin
uint16_t PWR_MGMT_1 = 0x6B; ///< MPU-6050 power management register address

uint8_t m1_in1_pin = 21;    ///< Input pin 1 for motor 1
uint8_t m1_in2_pin = 13;    ///< Input pin 2 for motor 1
uint8_t m2_in1_pin = 12;    ///< Input pin 1 for motor 2
uint8_t m2_in2_pin = 27;    ///< Input pin 2 for motor 2
uint8_t m3_in1_pin = 33;    ///< Input pin 1 for motor 3
uint8_t m3_in2_pin = 15;    ///< Input pin 2 for motor 3

uint32_t m1_freq = 16000;   ///< PWM Frequency for Motor 1
uint32_t m2_freq = 16000;   ///< PWM Frequency for Motor 2
uint32_t m3_freq = 1000;    ///< PWM Frequency for Motor 3

uint8_t pwm_resolution = 8; ///< Resolution of pwm frequency value

IMU mpu; ///< IMU Object
Motor pitch_motor;  ///< Pitch motor object
Motor roll_motor;   ///< Roll motor object
Motor yaw_motor;    ///< Yaw motor object

Share<int16_t> pitch("Reading Angle (main)"); ///< Share variable from IMU class


/** @brief   The web server object.
 *  @details This server is responsible for responding to HTTP requests from
 *           other computers, replying with useful information.
*/
WebServer server (80);

void setup_wifi (void)
{
  Serial << "Connecting to " << ssid;
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(1000);
    Serial.print("Not connected");
  }

  Serial << "Connected at IP address " << WiFi.localIP() << endl;
}

/** @brief   Put a web page header into an HTML string. 
 *  @details This header may be modified if the developer wants some actual
 *           @a style for her or his web page. It is intended to be a common
 *           header (and stylle) for each of the pages served by this server.
 *  @param   a_string A reference to a string to which the header is added; the
 *           string must have been created in each function that calls this one
 *  @param   page_title The title of the page
*/
void HTML_header (String& a_string, const char* page_title)
{
    a_string += "<!DOCTYPE html> <html>\n";
    a_string += "<head><meta name=\"viewport\" content=\"width=device-width,";
    a_string += " initial-scale=1.0, user-scalable=no\">\n<title> ";
    a_string += page_title;
    a_string += "</title>\n";
    a_string += "<style>html { font-family: Helvetica; display: inline-block;";
    a_string += " margin: 0px auto; text-align: center;}\n";
    a_string += "body{margin-top: 50px;} h1 {color: #4444AA;margin: 50px auto 30px;}\n";
    a_string += "p {font-size: 24px;color: #222222;margin-bottom: 10px;}\n";
    a_string += "</style>\n</head>\n";
}

/** @brief   Callback function that responds to HTTP requests without a subpage
 *           name.
 *  @details When another computer contacts this ESP32 through TCP/IP port 80
 *           (the insecure Web port) with a request for the main web page, this
 *           callback function is run. It sends the main web page's text to the
 *           requesting machine.
 */
void handle_DocumentRoot ()
{
    Serial << "HTTP request from client #" << server.client () << endl;

    String a_str;
    HTML_header (a_str, "ESP32 Web Server Test");
    a_str += "<body>\n<div id=\"webpage\">\n";
    a_str += "<h1>ME 507 Cam Stabilizer</h1>\n";
    a_str += "Jathun Somasundaram\n";
    a_str += "<p><p> <a href=\"/toggle\">WOOOO</a>\n";
    a_str += "<p><p> <a href=\"/csv\">IT WORKS :D</a>\n";
    a_str += "</div>\n</body>\n</html>\n";

    server.send (200, "text/html", a_str); 
}


/** @brief   Respond to a request for an HTTP page that doesn't exist.
 *  @details This function produces the Error 404, Page Not Found error. 
 */
void handle_NotFound (void)
{
    server.send (404, "text/plain", "Not found");
}

void handle_CSV (void)
{
    // The page will be composed in an Arduino String object, then sent.
    // The first line will be column headers so we know what the data is
    String csv_str = "Time, Jumpiness\n";

    // Create some fake data and put it into a String object. We could just
    // as easily have taken values from a data array, if such an array existed
    for (uint8_t index = 0; index < 20; index++)
    {
        csv_str += index;
        csv_str += ",";
        csv_str += String (sin (index / 5.4321), 3);       // 3 decimal places
        csv_str += "\n";
    }

    // Send the CSV file as plain text so it can be easily saved as a file
    server.send (404, "text/plain", csv_str);
}

/** @brief   Task which sets up and runs a web server.
 *  @details After setup, function @c handleClient() must be run periodically
 *           to check for page requests from web clients. One could run this
 *           task as the lowest priority task with a short or no delay, as there
 *           generally isn't much rush in replying to web queries.
 *  @param   p_params Pointer to unused parameters
 */
void task_SERVER (void* p_params)
{
    // The server has been created statically when the program was started and
    // is accessed as a global object because not only this function but also
    // the page handling functions referenced below need access to the server
    server.on ("/", handle_DocumentRoot);
    server.onNotFound (handle_NotFound);

    // Get the web server running
    server.begin ();
    Serial.println ("HTTP server started");

    for (;;)
    {
        // The web server must be periodically run to watch for page requests
        server.handleClient ();
        vTaskDelay (600);
    }
}



/** @brief   Task that reads the angles from the IMU class.
 *  @details This task runs the read angle functions from the IMU class every 100 ms.
 *  @param   p_params Pointer to unused parameters
 */
void task_read_IMU (void* p_params )
{
  while(true)
  {
    Serial << "Reading Pitch Angle" << endl;
    mpu.read_acc_pitch(MPU_ADDR);
    mpu.read_acc_roll(MPU_ADDR);
    vTaskDelay(100);
  }
}

void task_PITCH (void* p_params)
{
  
  // Variables related to desired angles
  int16_t pitch_home = 0;
  int16_t pitch_max = 40;
  int16_t pitch_min = -40;
  int16_t err_accept = 10;
  int16_t err_pitch;

  int16_t current_pitch; // Pitch from IMU
  int16_t prev_pitch; 
  int16_t set_pitch;     // Pitch from controller
  
  int16_t pitch_kp = 10;

  int16_t state = 0;

  while(true)
  {
    if(state == 0)
    {
      //Serial << "Motor Stopped" << endl;
      pitch_motor.brake();
      prev_pitch = current_pitch;
      current_pitch = pitch.get();
      //Serial << "Retrieved pitch: " << current_pitch << endl;
      
      //delay(1500);

      //Serial << prev_pitch << " , " << current_pitch << endl;
      err_pitch = pitch_home - current_pitch; 
      
      Serial << "Calculated error: " << err_pitch << endl;
      
      if(abs(err_pitch) < abs(err_accept)) 
      {
        //Serial << "This is fine :)" << endl;
        state = 4;
      }

      if(abs(err_pitch) > abs(err_accept))
      {
        //Serial << "This is not fine, " << err_pitch << endl;
        state = 1;
      }
    }

    if(state == 1)
    {
      set_pitch = err_pitch * pitch_kp;
      //Serial << "Setpoint acquired: " << set_pitch << endl;

      if(set_pitch < 0)
      {
        //pitch_motor.spin(50,0);
        state = 2;
      }
      if(set_pitch > 0)
      {
        //pitch_motor.spin(0,50);
        state = 3;
      }
      if(set_pitch = 0)
      {
        //pitch_motor.brake();
        //delay(1000);
        state = 4;
      }
    }

    if(state == 2)
    {
      //Serial << "Setpoint was counter, motor spinning clockwise" << endl;
      pitch_motor.spin(25,0);
      delay(100);
      state = 4;
    }

    if(state == 3)
    {
      //Serial << "Setpoint was clock, motor spinning counter" << endl;
      pitch_motor.spin(0,50);
      delay(60);
      state = 4;
    }

    if(state == 4)
    {
      //Serial << "Motor braking" << endl;
      pitch_motor.brake();
      delay(100);
      state = 0; 
    }

    vTaskDelay(300);  
  }
}
void task_YAW (void* p_params)
{
  while(true)
  {
    
  }
}

void task_ROLL (void* p_params)
{
  while(true)
  {
    
  }
}

void setup() 
{
  Serial.begin (115200);
    while (!Serial) 
    {
    }

  setup_wifi();
  mpu.IMU_init(I2C_SDA, I2C_SCL, MPU_ADDR, PWR_MGMT_1);
  pitch_motor.init(m1_in1_pin, m1_in2_pin, m1_freq, pwm_resolution);
  roll_motor.init(m2_in1_pin, m2_in2_pin,m2_freq, pwm_resolution);
  yaw_motor.init(m3_in1_pin, m3_in2_pin,m3_freq, pwm_resolution);

  Serial << "Hold IMU flat" << endl;
  delay (1000);
  mpu.cal_acc_pitch (MPU_ADDR);
  mpu.cal_acc_roll (MPU_ADDR);

  xTaskCreate (task_read_IMU, "Reading" , 2048, NULL, 2, NULL);
  xTaskCreate (task_PITCH, "Testing Pitch Axis", 2048, NULL, 2, NULL);
  //xTaskCreate (task_ROLL, "Testing Roll Axis", 2048, NULL, 1, NULL);
  xTaskCreate (task_SERVER, "Handling webpage", 2048, NULL, 1, NULL);

  

}

void loop() 
{
  // put your main code here, to run repeatedly:
  vTaskDelay(60000);
}
