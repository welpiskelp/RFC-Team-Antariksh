/*************************************************************************
COPYRIGHT NOTICE

   (c) 2025 Team Antariksh
   Author: Aarush Jaiswal & Rik Seth
   Contributer:
   Last changed: 24-10-25

   All rights reserved. Unauthorized copying, distribution, or use of this
   file or its contents is strictly prohibited without express permission
   from Team Antariksh.

*************************************************************************/


/*Standard Libraries*/
#include <stddef.h>
#include <Time.h>
#include <stdbool.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>
#include <Arduino.h>
#include <TinyGPS++.h>

/*Custom Libraries*/
#include "state.h"
#include "buzzer.h"
#include "logger.h"
#include "com_logger.h"
#include "sdcard_logger.h"
#include "lora_logger.h"

/*PIN'S*/
#define OFFSET_BMP 1 //BMP Offset FIXME: Change to integer
#define CS_BMP390 28
#define CS_LSM 5
#define GPS_BAUD 115200
#define BATTERY_PIN 41
#define MAINCHUTE_GPIO_1 14
#define MAINCHUTE_GPIO_2 0
#define REEFING_GPIO_1 28
#define PAYLOAD_GPIO_1 31
#define PAYLOAD_GPIO_2 33
#define SCK  27   // SPI1 SCK
#define MISO 1    // SPI1 MISO
#define MOSI 26   // SPI1 MOSI

/*  NEW STATE MACHINE DEFINITIONS  */
// LIFTOFF LOGIC
#define LIFTOFF_ACCELERATION_THRESHOLD 20.0f // m/s^2 (approx 2 G's, adjust as needed)
#define LIFTOFF_ALTITUDE_THRESHOLD     50.0f  // meters
// REEFING LOGIC
#define MAIN_CHUTE_ALTITUDE            350.0f // meters
// TOUCHDOWN LOGIC
#define TOUCHDOWN_ALTITUDE_THRESHOLD   20.0f  // meters (must be stable near ground)
#define TOUCHDOWN_ACCEL_THRESHOLD      1.5f   // m/s^2 (detects low-G, stable state)
// REDUNDANT LOGIC
#define FLIGHT_TIME_THRESHOLD          50.0f // should be between 15.066 seconds and 43.45 seconds HIGHLY ADVISED to not keep it in range of 30s or 40s
#define REDUNDANT_PARACHUTE_TIME_THRESHOLD 7000 //in milliseconds
#define PRESSURE_ZERO_LEVEL 10325.0f
#define LOG_INTERVAL_MS 7000
/* APOGEE Definitions */

#define ALTITUDE_WINDOW_SIZE 400
#define APOGEE_SAMPLE_RATE_MS 500
#define SMA_SIZE 10
#define DESCENT_DETECTION_COUNT 10     // Number of consecutive descending samples
#define NOISE_THRESHOLD 1.00f          // Altitude change (meters) to ignore as noise
#define MIN_TOTAL_DROP 0.5f            // Minimum total drop from the peak (meters)
#define REQUIRED_STABLE_SAMPLES  300; //For touchdown confirmed(50 = 1 sec)


/*EEPROM*/
// These memory states are overwritten with each state change
#define EEPROM_FLAG_ADDR     0  // Can change this
#define EEPROM_STATE_ADDR    (EEPROM_FLAG_ADDR + sizeof(state_flags_t))
#define EEPROM_MAGIC_ADDR    (EEPROM_STATE_ADDR + sizeof(state_t))
#define EEPROM_MAGIC_VALUE   0xA4
#define EEPROM_LOG_NEXT_ADDR (EEPROM_MAGIC_ADDR + sizeof(uint8_t))     
#define EEPROM_DATA_ADDR     (EEPROM_LOG_NEXT_ADDR + sizeof(uint16_t)) 

/* sensor object creation */
Adafruit_BMP3XX bmp390;
Adafruit_LSM6DSOX lsm6dso32;
TinyGPSPlus gps;


/* logger implementations */
extern logger_interface_t com_logger_interface;
extern logger_interface_t sdcard_logger_interface;
extern logger_interface_t lora_logger_interface;

/* altitude and apogeee */
float altitude_window[ALTITUDE_WINDOW_SIZE];
int window_index = 0;
bool apogee_detected = false;
bool  recovered_from_eeprom = false;

int reefing_flag, eep_sd_flag = 0;
float base_altitude, altitude, previous_altitude=0; //prevalti should be given the previous value of alti before it takes the new value
float altitude_offset=0;
float battery_voltage = 0;
const int BUF_SIZE = 10;
float bmpBuffer[BUF_SIZE];
int bmpIndex = 0;
float ax,ay,az,gx,gy,gz,tem=0;

/* global timestamp variable */
int payload_timestamp=0;
int parachute_timestamp=0;
int liftoff_timestamp=0;
int redundant_parachute_timestamp=0;
float  last_log_time = 0, last_apogee_log=0;

int packet_count_num = 0;

/*flags and bools*/
bool liftoff_acceleration_bool= false;
bool redundant_apogee_flag = false;


/* finite automata (state machine)*/
state_t prev_state = STATE_INITIALIZING;
state_t curr_state = STATE_INITIALIZING; //enum
state_flags_t flags = 0; // int is just typeef as state_flags_t

//TODO: EEPROM
/* store everything to flash and only store to the sd card towards the end, need to change logger.h*/
/* Write flags and current state to EEPROM */
void store_state_to_eeprom(state_flags_t flags, state_t state) {
   EEPROM.put(EEPROM_FLAG_ADDR, flags);
   EEPROM.put(EEPROM_STATE_ADDR, state);
   //EEPROM.put(EEPROM_LOG_NEXT_ADDR, logger_get_next_eeprom_addr());
   EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE); //Storing this magic num

}

/*
  * Objective: Try to recover flags and state from EEPROM
  * Return: true if valid data is found
*/
//uint16_t *log_addr_out // bool recover_state_from_eeprom(state_flags_t *flags_out, state_t *state_out,uint16_t *log_addr_out)
bool recover_state_from_eeprom(state_flags_t *flags_out, state_t *state_out) {
   // At least enough for our state/flag/magic (/rik)
   uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
   if (magic == EEPROM_MAGIC_VALUE) {
      EEPROM.get(EEPROM_FLAG_ADDR, *flags_out);
      EEPROM.get(EEPROM_STATE_ADDR, *state_out);
	 // EEPROM.get(EEPROM_LOG_NEXT_ADDR, *log_addr_out);

	  //if (*log_addr_out < EEPROM_DATA_ADDR || *log_addr_out > EEPROM.length()) {
      //   *log_addr_out = EEPROM_DATA_ADDR; // Reset if invalid
      //}
      return true;
   }
   return false;
}

/*
  * Erase EEPROM (telecommand)
*/
void erase_eeprom() {
   for (int i = 0; i < 32; ++i) {
      EEPROM.write(i, 0xFF); // Erase all bytes
   }
   LOG_INFO("EEPROM erased by telecommand");
   buzzer_beep_code(BUZZER_CODE_GENERAL_ERR); // Signal with buzzer
}

/*LoRA wait for telecommand*/

//TODO
/* Write the lora wait for teleccommand function */
/*
 * Wait for a LoRa telecommand ("START", "ERASE", "CALIBRATE")
 * Puts LoRa module into RX mode.
 */
void wait_for_telecommand() {
   char buffer[32];
   Serial.println("Waiting for LoRa telecommand ('START', 'ERASE', 'CALIBRATE')...");

   // Put LoRa module into continuous Receive mode
   LoRa.receive();

   while (true) {
      int packetSize = LoRa.parsePacket();

      if (packetSize) {
         // Received a packet, read it into the buffer
         int len = 0;
         while (LoRa.available() && abs(len) < sizeof(buffer) - 1) {
            buffer[len++] = (char)LoRa.read();
         }
         buffer[len] = '\0';

         // Optional: Remove trailing newline/carriage return
         if (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
            buffer[len - 1] = '\0';
         }
         if (len > 1 && (buffer[len - 2] == '\n' || buffer[len - 2] == '\r')) {
            buffer[len - 2] = '\0';
         }

         Serial.print("Received LoRa command: '");
         Serial.print(buffer);
         Serial.println("'");

         if (strcmp(buffer, "START") == 0) {
            LOG_INFO("'START' telecommand received, starting main flight loop.");
            lora_send_ack("ACK_START"); // Send ACK
            LoRa.idle(); // Stop RX mode
            break; // Exit the while loop

         } else if (strcmp(buffer, "ERASE") == 0) {
            LOG_INFO("'ERASE' telecommand received, erasing EEPROM.");
            erase_eeprom();
            lora_send_ack("ACK_ERASE"); // Send ACK and stay in RX mode
            Serial.println("EEPROM erased. Waiting for next command...");

         } else if (strcmp(buffer, "CALIBRATE") == 0) {
            LOG_INFO("'CALIBRATE' telecommand received, calibrating sensors.");
            calibrate();
            lora_send_ack("ACK_CALIBRATE"); // Send ACK and stay in RX mode
            Serial.println("Calibration complete. Waiting for next command...");

         } else {
            lora_send_ack("ACK_UNKNOWN"); // Acknowledge unknown command
         }
      }

      delay(10); // Small delay to prevent spamming
   }
}
/*void wait_for_telecommand() {
   // Use a 'char' buffer for C-style string functions
   char buffer[32];

   Serial.println("Enter 'START', 'ERASE', or 'CALIBRATE' in the Serial Monitor...");

   while (true) {
      // Check if there is data available to read from the serial port
      if (Serial.available() > 0) {

         // Read the incoming characters until a newline is received
         int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);

         // Add a null terminator to make it a valid C-string
         buffer[len] = '\0';

         // The Arduino Serial Monitor might send a carriage return ('\r') before the newline.
         // This optional check removes it if it's there.
         if (len > 0 && buffer[len - 1] == '\r') {
             buffer[len - 1] = '\0';
         }

         // Only process if the command is not empty
         if (strlen(buffer) > 0) {
            Serial.print("Received command: '");
            Serial.print(buffer);
            Serial.println("'");

            if (strcmp(buffer, "START") == 0) {
               LOG_INFO("'START' telecommand received, starting main flight loop.");
               // gs_ack(START);
               // curr_state = STATE_LIFTOFF_CONFIRMED; // Or whatever your first state after start is
               // store_state_to_eeprom(flags, curr_state);
               // LOG_INFO("Initial state saved to EEPROM. State=%d", curr_state);
               break; // Exit the while loop and continue the program

            } else if (strcmp(buffer, "ERASE") == 0) {
               LOG_INFO("'ERASE' telecommand received, erasing EEPROM.");
               erase_eeprom();
               // gs_ack(ERASE);
               Serial.println("EEPROM erased. Waiting for next command...");

            } else if (strcmp(buffer, "CALIBRATE") == 0) {
               LOG_INFO("'CALIBRATE' telecommand received, calibrating sensors.");
               calibrate();
               // gs_ack(CALIBRATE);
               Serial.println("Calibration complete. Waiting for next command...");
            }
         }
      }
   }
}
*/ //Old serial monitor based code, now commented out


/*
 * Special function: fire pyro channels during apogee event
*/

void fire_main_chute_1() {
   //TODO: delay 1.5s for apogee
   digitalWrite(MAINCHUTE_GPIO_1, HIGH);
   LOG_INFO("RECOVERY event: Main GPIO1 fired!");

}

void fire_main_chute_2() {
   //TODO: delay 1.5s for apogee
   digitalWrite(MAINCHUTE_GPIO_2, HIGH);
   LOG_INFO("RECOVERY event: Main GPIO2 fired!");
}

void fire_payload(){
   digitalWrite(PAYLOAD_GPIO_1, HIGH);
   LOG_INFO("APOGEE event: Payload GPIO1 fired!");
   digitalWrite(PAYLOAD_GPIO_2, HIGH);
   LOG_INFO("APOGEE event: Payload GPIO2 fired!");
}

void fire_reefing(){
   digitalWrite(REEFING_GPIO_1, HIGH);
   LOG_INFO("RECOVERY_REEFING event: Reefing GPIO1 fired!");
}

void shutdown_channels(){
   digitalWrite(MAINCHUTE_GPIO_1, LOW);
   LOG_INFO("Main GPIO1 Shutdown!");
   digitalWrite(MAINCHUTE_GPIO_2, LOW);
   LOG_INFO("Main GPIO2 Shutdown!");
   digitalWrite(REEFING_GPIO_1, LOW);
   LOG_INFO("Reefing GPIO1 Shutdown!");
   digitalWrite(PAYLOAD_GPIO_1, LOW);
   LOG_INFO("Payload GPIO1 Shutdown!");
   digitalWrite(PAYLOAD_GPIO_2, LOW);
   LOG_INFO("Payload GPIO2 Shutdown!");
   LOG_INFO("All channels Shutdown!");
}

void channel_init(){
   pinMode(MAINCHUTE_GPIO_1, OUTPUT);
   digitalWrite(MAINCHUTE_GPIO_1, LOW);
   LOG_INFO("Main GPIO1 Initialized!");
   pinMode(MAINCHUTE_GPIO_2, OUTPUT);
   digitalWrite(MAINCHUTE_GPIO_2, LOW);
   LOG_INFO("Main GPIO2 Initialized!");
   pinMode(REEFING_GPIO_1, OUTPUT);
   digitalWrite(REEFING_GPIO_1, LOW);
   LOG_INFO("Reefing GPIO1 Initialized!");
   pinMode(PAYLOAD_GPIO_1, OUTPUT);
   digitalWrite(PAYLOAD_GPIO_1, LOW);
   LOG_INFO("Payload GPIO1 Initialized!");
   pinMode(PAYLOAD_GPIO_2, OUTPUT);
   digitalWrite(PAYLOAD_GPIO_2, LOW);
   LOG_INFO("Payload GPIO2 Iniialized!");
   LOG_INFO("All Channels Iniialized!");
}


/* TODO:FAILSAFE Functiins */
/* 1. Redundant Apogee Detect (return bool) [DONE]
   2. Check Payload fire? (for 1 second then refire) [DONE]
   3. Check if Reefing Fire? (for descent, if not then refire)
*/

void redundant_apogee_detect(){
	int status = 0;
	float sum_apogee =0;
   float pressure = 0;
	bool allNegative = false, allConstant = false; // Flags to check conditions
    bmpBuffer[0]=(bmp_pressure_to_altitude(PRESSURE_ZERO_LEVEL, pressure) - altitude_offset);

  	for (int i = 1; i < BUF_SIZE; i++) {
   		delay(500);
		digitalWrite(CS_BMP390, LOW);
		// BMP Read
		if (!bmp390.performReading()) {
 			 LOG_ERROR("Failed to read from BMP390",-1);
		}
		else {
		     pressure=bmp390.readPressure();
		     altitude = (bmp_pressure_to_altitude(PRESSURE_ZERO_LEVEL, pressure)-altitude_offset);
			 float diff = bmpBuffer[i] - bmpBuffer[i - 1] ;//Calculate Differences
		     sum_apogee += diff;  // Accumulate Differences
		}
		digitalWrite(CS_BMP390, HIGH);

		if(sum_apogee<0) allNegative=true;
		if(sum_apogee==0) allConstant=true;

		if(allNegative) status=1;
        if(allConstant) status=2;

		switch(status) {

			case 1: //Descent Detected
				fire_payload();
				delay(500);
		        fire_payload();
            	delay(1000);
           	    fire_main_chute_1();
				// TODO: Implement a checker
				break;
			case 2: // BMP stuck / constant
            	fire_payload();
            	delay(500);
            	//redudant check
            	fire_payload();
            	delay(1000);
            	fire_main_chute_1();
            	delay(12000);
            	fire_reefing();
      			// Eject Reefing line
      			break;

    		default: // nothing to do
      			break;
  		}
}
}


/*TODO:LOGIC FUNCTIONS*/
/*
  1. apogee_detect()? true:False @param: window of altitude readings
  2. liftoff_confirm()?
  2. payload_eject()?
  3. parachute_detect()?
  4. reefing_eject()?
*/

void apogee_detect(){
   float pressure=0;

	float delta = altitude - previous_altitude;
	if (delta <= (-1)) {
		previous_altitude = altitude;
		delay(500);
		digitalWrite(CS_BMP390, LOW);
		// BMP Read
		if (!bmp390.performReading()) {
 			 LOG_ERROR("Failed to read from BMP390",-1);
		}
		else {
		     pressure=bmp390.readPressure();
		     altitude = (bmp_pressure_to_altitude(PRESSURE_ZERO_LEVEL, pressure)-altitude_offset);
			 delta = altitude - previous_altitude;
			 if (delta < (-2)) {
				apogee_detected = true;
			 }
		}
		digitalWrite(CS_BMP390, HIGH);
	}
}


/* TODO:Helper functions*/

// bmp_pressure_to_altitude
float bmp_pressure_to_altitude(float zeropressure, float pressure_pa) {
   // Altitude = 44330 * (1 - (P/P0)^(1/5.255))
   float altitude = 44330.0 * (1.0 - pow(pressure_pa / zeropressure, 0.1902949));
   return altitude;
}

// Telemetry Packet state sender
const char* get_state_string(state_t state) {
   switch (state) {
      case STATE_INITIALIZING: return "INITIALIZING";
      case STATE_STANDBY: return "STANDBY";
      case STATE_LIFTOFF_CONFIRMED: return "LIFTOFF";
      case STATE_APOGEE_REACHED: return "APOGEE_REACHED";
      case STATE_MAIN_EJECTED: return "MAIN_EJECTED";
      case STATE_PARACHUTE_EJECTED: return "PARACHUTE_EJECTED";
      case STATE_DESCENT_STARTED: return "DESCENT_STARTED";
      case STATE_TOUCHDOWN_CONFIRMED: return "TOUCHDOWN";
      default: return "UNKNOWN";
   }
}

//battery status
void battery_status(){
  float temp_battery_voltage = analogRead(BATTERY_PIN);
  float actual_voltage = (temp_battery_voltage/1023.0)*3.3;  //10- bit adc
  battery_voltage = 4.3*actual_voltage;
}



/*TODO: Initializing Functions*/

void gps_init(){
   Serial8.begin(GPS_BAUD);
}

void bmp_init(){
   pinMode(CS_BMP390, OUTPUT);
   digitalWrite(CS_BMP390, HIGH);

   if (!bmp390.begin_SPI(CS_BMP390, &SPI1)) {
      buzzer_beep_code(BUZZER_CODE_INIT_FAIL);
      LOG_ERROR("Failed to initialize BMP390 SPI!");
      while (1) delay(10);
   } else {
      LOG_INFO("BMP initialized!");
      buzzer_beep_code(BUZZER_CODE_INIT_SUCCESS);
   }

   bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
   bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
   bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
   bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
}

void lsm_init(){
   pinMode(CS_LSM, OUTPUT);
   digitalWrite(CS_LSM, HIGH);

   if (!lsm6dso32.begin_SPI(CS_LSM, &SPI1)) {
      LOG_ERROR("Failed to initialize LSM6DSO32 SPI!");
      buzzer_beep_code(BUZZER_CODE_INIT_FAIL);
      while (1) delay(10);
   } else {
      LOG_INFO("LSM6DSO32 initialized!");
      buzzer_beep_code(BUZZER_CODE_INIT_SUCCESS);
   }
   lsm6dso32.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
   lsm6dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
   lsm6dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
   lsm6dso32.setGyroDataRate(LSM6DS_RATE_104_HZ);

}

void altitude_window_init(){
   /* initialize window with zeros */
   for (int i = 0; i < ALTITUDE_WINDOW_SIZE; ++i) {
      altitude_window[i] = 0.0f;
   }
}

void FC_init(){
   SPI1.setSCK(SCK);
   SPI1.setMISO(MISO);
   SPI1.setMOSI(MOSI);
   SPI1.begin();

   channel_init(); // Initialize all Pyro Channels
   buzzer_init(); // Buzzer Initialize
   bmp_init(); // BMP 390 CS to OUTPUT and HIGH
   lsm_init(); // LSM CS to OUTPUT and HIGH
   // EEPROM.begin(32);
   logger_init();
   logger_register_interface(&com_logger_interface);
   logger_register_interface(&sdcard_logger_interface); // replaces sdcard_logger
   logger_register_interface(&lora_logger_interface);
   /* setup lora */
   // if (!LoRa.begin(433E6)) {
   //    Serial.println("LoRa initialization failed!");

   // }
   /* initialize window with zeros */
   altitude_window_init();
   LOG_INFO("RFC Flight Systems Initialized");
   buzzer_beep_code(BUZZER_CODE_INIT_SUCCESS);
}

void calibrate(){
  LOG_INFO("Calibrating altitude...");
  float sum_alti = 0;
  for (int j = 0; j < 30; j++) {
    // We can't use read_calibrated_altitude() because offset isn't set yet
    digitalWrite(CS_BMP390, LOW);
    if (!bmp390.performReading()) {
        LOG_ERROR("Failed to read from BMP390 during calibration", -1);
        j--; // Retry the reading
        delay(100);
        continue;
    }
    float pressure = bmp390.readPressure();
    digitalWrite(CS_BMP390, HIGH);

    sum_alti += bmp_pressure_to_altitude(PRESSURE_ZERO_LEVEL, pressure);
    delay(100);
  }
  altitude_offset = sum_alti / 30.0;
  LOG_INFO("Calibration complete. Altitude offset: %f", altitude_offset);
}





/*TODO: SETUP*/

void setup(){
   FC_init();

   //uint16_t recovered_log_addr = EEPROM_DATA_ADDR;

   /* Recover previous state if available */
   recovered_from_eeprom = recover_state_from_eeprom(&flags, &curr_state);
   if (recovered_from_eeprom) {
      prev_state = curr_state;
      LOG_INFO("Recovered state from EEPROM: State=%d, Flags=0x%08lx", curr_state, (unsigned long)flags);
      buzzer_beep_code(BUZZER_CODE_INIT_SUCCESS); // Optionally indicate recovery
	//  logger_init_eeprom(EEPROM_DATA_ADDR, recovered_log_addr);
   } else {
      /* Wait for telecommand before starting main flight loop */
      wait_for_telecommand();
   }
}

void loop(){
   float pressure = 0.0f;
   float temperature = 0.0f;
   float altitude = 0.0f;
   float acceleration[3] = {0};
   float angular_rate[3] = {0};
   float gps_lat = 0.0f, gps_lon = 0.0f;
   bool state_changed = false;

   gps.encode(Serial8.read());

   // Calculate helper variables needed for state logic
   float altitude_agl = altitude - altitude_offset;
   float total_acceleration = sqrt(acceleration[0] * acceleration[0] + acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2]);

   digitalWrite(CS_LSM, HIGH);
   digitalWrite(CS_BMP390, LOW);


   // BMP Read
   if (!bmp390.performReading()) {
      LOG_ERROR("Failed to read from BMP390",-1);
   } else {
      temperature=bmp390.readTemperature();
      pressure=bmp390.readPressure();
      altitude = bmp_pressure_to_altitude(PRESSURE_ZERO_LEVEL, pressure);
   }
   digitalWrite(CS_BMP390, HIGH);

   /* store altitude in sliding window */
   altitude_window[window_index] = altitude_agl;
   window_index = (window_index + 1) % ALTITUDE_WINDOW_SIZE;

   /* read imu  */
   digitalWrite(CS_LSM, LOW);
   //
   sensors_event_t accel, gyro;
   lsm6dso32.getAccelerometerSensor()->getEvent(&accel);
   lsm6dso32.getGyroSensor()->getEvent(&gyro); // read only gyroscope

   //
   acceleration[0] = accel.acceleration.x;
   acceleration[1] = accel.acceleration.y;
   acceleration[2] = accel.acceleration.z;
   //
   angular_rate[0] = gyro.gyro.x;
   angular_rate[1] = gyro.gyro.y;
   angular_rate[2] = gyro.gyro.z;
   //
   digitalWrite(CS_LSM, HIGH);

   // GPS Read
   gps_lat = gps.location.lat();
   gps_lon = gps.location.lng();

   battery_status();

   /* STATE_LOGIC */

   switch (curr_state) {
      case STATE_INITIALIZING:
		 calibrate();
         break;
      case STATE_STANDBY:
         if (total_acceleration > LIFTOFF_ACCELERATION_THRESHOLD){
             liftoff_acceleration_bool = true;
         }

         if (liftoff_acceleration_bool && altitude_agl > LIFTOFF_ALTITUDE_THRESHOLD) {
            LOG_INFO("LIFTOFF DETECTED! Accel: %f, Alt AGL: %f", total_acceleration, altitude_agl);
			liftoff_timestamp = millis();
            flags |= FLAG_LIFTOFF_CONFIRMED_BIT;
            curr_state = STATE_LIFTOFF_CONFIRMED;
         }
         break;
      case STATE_LIFTOFF_CONFIRMED:
		    /* apogee detection: observer algorithm */

		if(millis()-liftoff_timestamp > FLIGHT_TIME_THRESHOLD){
			redundant_apogee_detect();
			flags |= FLAG_APOGEE_REACHED_BIT;
      		curr_state = STATE_APOGEE_REACHED;
			redundant_apogee_flag=true;          //initilaise flag

		}
		else{
		apogee_detect(); // call apogee detect to change the bool value in case we have hit apogee
   		if (apogee_detected) {
      		//fire_main_chute1();
      		//apogee_detected = true;
      		flags |= FLAG_APOGEE_REACHED_BIT;
      		curr_state = STATE_APOGEE_REACHED;
   		}
		}
        break;


      case STATE_APOGEE_REACHED:
	        if(!redundant_apogee_flag){

           	 	LOG_INFO("APOGEE REACHED");
            	fire_payload();
				payload_timestamp = millis();
            	delay(500);
            	//redudant check
				fire_payload();
				payload_timestamp = millis();

            	delay(1000);
            	fire_main_chute_1();
				parachute_timestamp = millis();

                if(millis()-parachute_timestamp>REDUNDANT_PARACHUTE_TIME_THRESHOLD){
				fire_main_chute_2();
			    redundant_parachute_timestamp = millis();
				}
			flags |= FLAG_MAIN_EJECTED_BIT;
      		curr_state = STATE_MAIN_EJECTED;

         	break;

      case STATE_MAIN_EJECTED:
			flags |= FLAG_DESCENT_STARTED_BIT;
      		curr_state = STATE_DESCENT_STARTED;
        break;

      case STATE_DESCENT_STARTED:

		 if(altitude<350 && reefing_flag==0){
			fire_reefing();
		    reefing_flag = 1;
		    eep_sd_flag=1;  //TO DO:In void loop use this as condition for starting to flush from eeprom to sd card
		 }
			flags |= FLAG_DESCENT_STARTED_BIT;
      		curr_state = STATE_DESCENT_STARTED;
         break;

      case STATE_PARACHUTE_EJECTED:
		 //float delta_touchdown = 0;
		 //float previous_altitude = altitude_agl;
		 //int score = 0;
		 //for(int i = 0; i<ALTITUDE_WINDOW_SIZE; i++){
			//delta_touchdown = altitude_window[i+1] - altitude_window[i];
			//if(delta_touchdown > TOUCHDOWN_THRESHOLD){
			//	score++;
			//}
			//if(score<
		 //}
		 //if(touchdown_confirmed()){
		 	//flags |= FLAG_TOUCHDOWN_CONFIRMED_BIT;
      		//curr_state = STATE_TOUCHDOWN	_CONFIRMED;
		 //}

         break;


      case STATE_TOUCHDOWN_CONFIRMED:
         shutdown_channels();
		 if (eep_sd_flag == 1) {
            LOG_INFO("TOUCHDOWN. Flushing EEPROM data to SD card...");
           // flashlogger_flush_to_sd("EE_DUMP.TXT"); //TODO: Flusher
            LOG_INFO("EEPROM flush complete.");
            eep_sd_flag = 0; // Set flag to 0 to prevent re-flushing
         }

         break;

      default:
         LOG_ERROR("Unknown state: %d", curr_state);
         break;
   }

   /*
      update state machine, log and store state if changed
      (RUNS AT END OF EVERY LOOP BEFORE PRINTING)
   */
   update_state(flags, &prev_state, &curr_state, &state_changed);
   if (state_changed) {
      LOG_INFO("STATE CHANGE: %d -> %d, Flags: 0x%08lx", prev_state, curr_state, (unsigned long)flags);
      store_state_to_eeprom(flags, curr_state);
      prev_state = curr_state;
   }


   /* log data periodically */
   if (millis() - last_log_time > LOG_INTERVAL_MS) {
      last_log_time = millis();

      // 1. Create a packet
      telemetry_data_t packet;

      // 2. Fill the packet with your data
      packet.team_id = "2024-ASI-ROCKETRY-027"; // As defined in your logger.c
      packet.time_stamp = millis() / 1000.0;    // Use rocket's ON-time
      packet.packet_count = packet_count_num;
      packet.altitude = altitude;
      packet.pressure = pressure;
      packet.temp = temperature;
      packet.voltage = battery_voltage;

      /* GNSS Data*/
      packet.gnss_time = gps.time.value();
      packet.gnss_lat = gps.location.lat();
      packet.gnss_lon = gps.location.lng();
      packet.gnss_alt = gps.altitude.meters();
      packet.gnss_sats = gps.satellites.value();

      // IMU Data (Uncomment when your IMU code is working)
      packet.accel_data_x = acceleration[0]; // Example: Just logging Z-axis
      packet.accel_data_y = acceleration[1];
      packet.accel_data_z = acceleration[2];
      packet.gyro_data_x = angular_rate[0];
      packet.gyro_data_y = angular_rate[1];
      packet.gyro_data_z = angular_rate[2];
      // State Data
      packet.flight_state = get_state_string(curr_state);

      // 3. Log the single packet
      // We set 'critical' to false for standard telemetry
      log_telemetry(&packet, false);
      packet_count_num++;
      /* flush the logger */
      logger_flush();
   }

   if(curr_state == STATE_TOUCHDOWN_CONFIRMED){
   delay(500);
    }else{
   delay(20); /*50 Hz Loop*/ //TODO: Find actual frequency
	}
}
}
