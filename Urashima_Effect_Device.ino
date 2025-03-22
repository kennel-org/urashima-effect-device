#include <SPI.h>        // SPI library for SD card
#include <SD.h>         // SD card library
#include <FS.h>         // File system library
#include <M5Unified.h>  // Unified library for M5Stack devices
#include <TinyGPS++.h>  // GPS parsing library

// Constants
const double ORIGINAL_LIGHT_SPEED = 299792.458;  // Actual speed of light (km/s)
const double MODIFIED_LIGHT_SPEED = 0.3;         // Modified speed of light (km/s)

// GPIO pin for button on ATOMS3R
#define BUTTON_PIN 41

// SD Card settings for AtomicBase GPS unit
#define SD_CS_PIN 22     // CS pin for SD card on AtomicBase GPS
#define LOG_INTERVAL 1000 // Log data every 1000ms

// GPS connection settings for AtomicBase GPS
#define GPS_RX_PIN 1     // ATOMS3R RX pin (G1) when using AtomicBase GPS
#define GPS_TX_PIN 2     // ATOMS3R TX pin (G2) when using AtomicBase GPS

// Mode and timing constants
#define GPS_TIMEOUT 5000        // Time (ms) after which GPS is considered lost if no data
#define ACC_CALIBRATION_TIME 10000  // Time (ms) to calibrate accelerometer with GPS data
#define DRIFT_CORRECTION_FACTOR 0.2 // How quickly to correct accelerometer drift (0-1)

// Log file information
File logFile;
bool sd_available = false;
unsigned long last_log_time = 0;
String log_filename = "";

// Global variables
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Serial port for GPS
boolean imu_available = false; // Whether IMU is available
boolean show_raw_gps = false;  // Whether to show raw GPS data

unsigned long lastGpsDataTime = 0;
boolean gpsDataReceived = false;
double current_speed = 0.0;    // Current speed (km/h)
double gps_speed = 0.0;        // Last valid GPS speed (km/h)
double acc_speed = 0.0;        // Accelerometer-derived speed (km/h)
double time_dilation = 0.0;    // Time dilation factor
double elapsed_device_time = 0.0;  // Elapsed device time (seconds)
double elapsed_relativistic_time = 0.0;  // Relativistically elapsed time (seconds)
double time_difference = 0.0;  // Time difference (seconds)
unsigned long last_update = 0;  // Last update time
bool using_gps_data = false;   // Tracks if we're currently using GPS data
bool acc_calibrated = false;   // Whether accelerometer has been calibrated with GPS
float acc_drift_factor = 1.0;  // Correction factor for accelerometer drift

// Accelerometer data
float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;
float prev_accX = 0.0;
float prev_accY = 0.0;
float prev_accZ = 0.0;
float acc_threshold = 0.03;    // Threshold to detect movement (to avoid noise)
float acc_velocity_x = 0.0;    // Integrated velocity in X direction
float acc_velocity_y = 0.0;    // Integrated velocity in Y direction
float acc_velocity_z = 0.0;    // Integrated velocity in Z direction
unsigned long acc_last_time = 0; // Last time accelerometer was read

// Forward declarations
void updateDisplay();
void calculateRelativisticEffect(double delta_t);
void resetTimeCalculation();
void updateAccelerometerData();
void calibrateAccelerometer();
void adjustAccelerometerDrift();
void initSDCard();
void logData();

// Initialize SD card and create log file
void initSDCard() {
  Serial.println("Initializing SD card...");
  
  // Initialize SPI for SD card
  SPI.begin(23, 33, 19, SD_CS_PIN); // SCLK, MISO, MOSI, CS for AtomicBase GPS
  
  // Initialize SD card
  if (SD.begin(SD_CS_PIN)) {
    sd_available = true;
    Serial.println("SD card initialized.");
    
    // Create a unique filename based on time
    char filename[32];
    sprintf(filename, "/urashima_%lu.csv", millis());
    log_filename = String(filename);
    
    // Open the file for writing
    logFile = SD.open(log_filename, FILE_WRITE);
    
    if (logFile) {
      // Write CSV header
      logFile.println("Timestamp,GPS_Valid,Latitude,Longitude,GPS_Speed_kmh,Accelerometer_Speed_kmh,Used_Speed_kmh,Device_Time_s,Relativistic_Time_s,Time_Diff_s,Dilation_Factor,GPS_DateTime,Satellites,HDOP");
      logFile.flush();
      Serial.print("Log file created: ");
      Serial.println(log_filename);
    } else {
      Serial.println("Error creating log file");
      sd_available = false;
    }
  } else {
    Serial.println("SD card initialization failed.");
    sd_available = false;
  }
}

// Log data to SD card
void logData() {
  if (!sd_available) return;
  
  // Only log at the specified interval
  if (millis() - last_log_time < LOG_INTERVAL) return;
  last_log_time = millis();
  
  // Try to reopen the file if it was closed
  if (!logFile) {
    logFile = SD.open(log_filename, FILE_APPEND);
    if (!logFile) {
      Serial.println("Error reopening log file");
      return;
    }
  }
  
  // Format: Timestamp,GPS_Valid,Latitude,Longitude,GPS_Speed_kmh,Accelerometer_Speed_kmh,Used_Speed_kmh,Device_Time_s,Relativistic_Time_s,Time_Diff_s,Dilation_Factor,GPS_DateTime,Satellites,HDOP
  
  // Format GPS datetime if valid
  String gpsDateTime = "invalid";
  if (gps.date.isValid() && gps.time.isValid()) {
    char dateTime[32];
    sprintf(dateTime, "%04d-%02d-%02d %02d:%02d:%02d", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsDateTime = String(dateTime);
  }
  
  // Create the log entry
  String logEntry = String(millis()) + "," +
                   String(gps.location.isValid() ? "1" : "0") + "," +
                   String(gps.location.isValid() ? gps.location.lat(), 6 : 0.0, 6) + "," +
                   String(gps.location.isValid() ? gps.location.lng(), 6 : 0.0, 6) + "," +
                   String(gps_speed, 2) + "," +
                   String(acc_speed, 2) + "," +
                   String(current_speed, 2) + "," +
                   String(elapsed_device_time, 3) + "," +
                   String(elapsed_relativistic_time, 3) + "," +
                   String(time_difference, 3) + "," +
                   String(time_dilation, 6) + "," +
                   gpsDateTime + "," +
                   String(gps.satellites.value()) + "," +
                   String(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 1);
                   
  // Write to SD card
  logFile.println(logEntry);
  logFile.flush();  // Make sure data is written immediately
  
  // Indicate logging status on display when logging
  static uint8_t log_count = 0;
  if (++log_count % 5 == 0) {  // Every 5 logs
    // Small display indicator in corner
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(RED);
    M5.Display.print("*");
    M5.Display.setTextColor(WHITE);
  }
}

// Calculate relativistic effect
void calculateRelativisticEffect(double delta_t) {
  // Convert speed from km/h to km/s
  double speed_kms = current_speed / 3600.0;
  
  // Light speed ratio
  double c_ratio = MODIFIED_LIGHT_SPEED / ORIGINAL_LIGHT_SPEED;
  
  // Modified relativistic speed (how the actual speed relates to the modified light speed)
  double relative_speed = speed_kms / MODIFIED_LIGHT_SPEED;
  
  // Calculate time dilation factor (Lorentz factor)
  // γ = 1/√(1-(v²/c²))
  if (relative_speed < 1.0) {  // Below light speed
    time_dilation = 1.0 / sqrt(1.0 - (relative_speed * relative_speed));
  } else {
    time_dilation = 99999.0;  // Very large value for speeds at or above light speed
  }
  
  // Update relativistically elapsed time
  elapsed_relativistic_time += delta_t / time_dilation;
  
  // Calculate time difference
  time_difference = elapsed_device_time - elapsed_relativistic_time;
}

// Reset time calculations
void resetTimeCalculation() {
  elapsed_device_time = 0.0;
  elapsed_relativistic_time = 0.0;
  time_difference = 0.0;
  
  // Also reset accelerometer-derived velocities
  acc_velocity_x = 0.0;
  acc_velocity_y = 0.0;
  acc_velocity_z = 0.0;
  
  // Create a new log file when resetting
  if (sd_available) {
    // Close current log file
    if (logFile) {
      logFile.close();
    }
    
    // Create a new log file
    char filename[32];
    sprintf(filename, "/urashima_%lu.csv", millis());
    log_filename = String(filename);
    
    logFile = SD.open(log_filename, FILE_WRITE);
    if (logFile) {
      // Write CSV header
      logFile.println("Timestamp,GPS_Valid,Latitude,Longitude,GPS_Speed_kmh,Accelerometer_Speed_kmh,Used_Speed_kmh,Device_Time_s,Relativistic_Time_s,Time_Diff_s,Dilation_Factor,GPS_DateTime,Satellites,HDOP");
      logFile.flush();
      Serial.print("New log file created: ");
      Serial.println(log_filename);
    } else {
      Serial.println("Error creating new log file");
    }
  }
  
  // Reset message - minimal
  Serial.println("Time measurement reset");
}

// Update accelerometer data
void updateAccelerometerData() {
  // Get time since last update
  unsigned long current_time = millis();
  float delta_t = (current_time - acc_last_time) / 1000.0; // Time in seconds
  acc_last_time = current_time;
  
  // Limit delta_t to avoid huge jumps if updates are delayed
  if (delta_t > 0.5) delta_t = 0.5;
  
  bool imu_success = false;
  
  // Try to read from M5.Imu
  float ax, ay, az;
  if (M5.Imu.getAccelData(&ax, &ay, &az)) {
    accX = ax;
    accY = ay;
    accZ = az;
    imu_success = true;
    
    // Log IMU data occasionally
    static unsigned long last_imu_log = 0;
    if (millis() - last_imu_log > 5000) {
      Serial.printf("IMU: X=%.3f Y=%.3f Z=%.3f\n", accX, accY, accZ);
      last_imu_log = millis();
    }
  }
  
  if (imu_success) {
    // High-pass filter to remove gravity and noise
    float delta_accX = accX - prev_accX;
    float delta_accY = accY - prev_accY;
    float delta_accZ = accZ - prev_accZ;
    
    // Store current values for next time
    prev_accX = accX;
    prev_accY = accY;
    prev_accZ = accZ;
    
    // Apply threshold to detect real movement
    if (abs(delta_accX) > acc_threshold) {
      // Integrate acceleration to get velocity
      acc_velocity_x += delta_accX * delta_t;
    } else {
      // Decay velocity gradually to simulate friction
      acc_velocity_x *= 0.95;
    }
    
    if (abs(delta_accY) > acc_threshold) {
      acc_velocity_y += delta_accY * delta_t;
    } else {
      acc_velocity_y *= 0.95;
    }
    
    if (abs(delta_accZ) > acc_threshold) {
      acc_velocity_z += delta_accZ * delta_t;
    } else {
      acc_velocity_z *= 0.95;
    }
    
    // Calculate total velocity magnitude in 3D space
    float velocity = sqrt(acc_velocity_x*acc_velocity_x + 
                         acc_velocity_y*acc_velocity_y + 
                         acc_velocity_z*acc_velocity_z);
    
    // Apply drift correction
    velocity *= acc_drift_factor;
    
    // Convert to km/h
    double new_acc_speed = velocity * 3.6;
    
    // Apply smoothing - more aggressive smoothing when accelerometer is not calibrated
    float smoothing = acc_calibrated ? 0.8 : 0.95;
    acc_speed = acc_speed * smoothing + new_acc_speed * (1.0 - smoothing);
    
    // Occasionally log accelerometer data
    static unsigned long last_acc_log = 0;
    if (millis() - last_acc_log > 3000) {
      Serial.printf("Accel: X=%.3f Y=%.3f Z=%.3f, Vel=%.3f, Speed=%.2f km/h\n", 
                    accX, accY, accZ, velocity, acc_speed);
      last_acc_log = millis();
    }
  } else {
    // IMU read failed, use fallback simulation for demo purposes
    static unsigned long last_sim_update = 0;
    if (millis() - last_sim_update > 500) {
      // Simulate acceleration data
      accX = random(-100, 100) / 1000.0;
      accY = random(-100, 100) / 1000.0;
      accZ = random(900, 1100) / 1000.0; // Roughly 1g with small variations
      
      // Simple acceleration to speed conversion
      float acceleration = sqrt(accX*accX + accY*accY + accZ*accZ) - 1.0; // Subtract gravity
      if (acceleration < 0) acceleration = 0;
      
      // Apply drift correction
      acceleration *= acc_drift_factor;
      
      // Rough conversion to km/h
      double new_acc_speed = acceleration * 3.6 * 10; // Convert to km/h with scaling
      
      // Smoothing
      acc_speed = acc_speed * 0.9 + new_acc_speed * 0.1;
      
      last_sim_update = millis();
      Serial.println("Using simulated accelerometer data (IMU read failed)");
    }
  }
}

// Calibrate accelerometer with GPS data
void calibrateAccelerometer() {
  if (!gps.speed.isValid()) return; // Can't calibrate without GPS
  
  // Reset integrated velocities when calibrating
  acc_velocity_x = 0;
  acc_velocity_y = 0;
  acc_velocity_z = 0;
  
  // Since we're using an integrated velocity approach, the calibration
  // is mainly adjusting the drift factor and resetting accumulated velocities
  
  // If GPS says we're not moving, just reset accelerometer-based speed
  if (gps_speed < 0.5) {
    acc_speed = 0;
    acc_drift_factor = 1.0; // Reset to default
    acc_calibrated = true;
    Serial.println("Calibrated accelerometer at rest position");
    return;
  }
  
  // If we have movement according to GPS, calculate appropriate drift factor
  if (acc_speed > 0.5) { // Avoid division by near-zero
    // Calculate correction factor to match GPS speed
    acc_drift_factor = gps_speed / acc_speed;
    
    // Limit drift factor to reasonable range (0.1 to 10)
    if (acc_drift_factor < 0.1) acc_drift_factor = 0.1;
    if (acc_drift_factor > 10.0) acc_drift_factor = 10.0;
    
    // Update the accelerometer-based speed with the new factor
    acc_speed = gps_speed;
    
    // Mark as calibrated
    acc_calibrated = true;
    
    Serial.printf("Calibrated accelerometer: GPS=%.2f, Acc=%.2f, Factor=%.3f\n", 
                  gps_speed, acc_speed, acc_drift_factor);
  } else {
    // No significant movement detected by accelerometer but GPS shows movement
    // This might be a very smooth motion - just copy GPS speed and use default drift
    acc_speed = gps_speed;
    acc_drift_factor = 1.0;
    acc_calibrated = true;
    
    Serial.printf("Calibrated with default factor: GPS=%.2f, AccSensor too low\n", gps_speed);
  }
}

// Gradually adjust accelerometer drift based on GPS
void adjustAccelerometerDrift() {
  if (!gps.speed.isValid()) return; // Need GPS data
  
  // Only adjust if we're moving (both sensors detect movement)
  if (gps_speed > 1.0 && acc_speed > 1.0) {
    // Calculate "ideal" drift factor
    double ideal_factor = gps_speed / acc_speed * acc_drift_factor;
    
    // Gradually adjust current drift factor
    acc_drift_factor = acc_drift_factor * (1 - DRIFT_CORRECTION_FACTOR) + 
                        ideal_factor * DRIFT_CORRECTION_FACTOR;
    
    // Limit drift factor to reasonable range (0.1 to 10)
    if (acc_drift_factor < 0.1) acc_drift_factor = 0.1;
    if (acc_drift_factor > 10.0) acc_drift_factor = 10.0;
  }
}

// Update display
void updateDisplay() {
  if (M5.Display.width() <= 0) return;  // Skip if no display available
  
  // Check if we're using a small display
  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
  
  // If showing raw GPS data, display that instead of regular UI
  if (show_raw_gps) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(GREEN);
    M5.Display.println("GPS RAW DATA:");
    M5.Display.setTextColor(WHITE);
    
    // Show date and time if available
    if (gps.date.isValid() && gps.time.isValid()) {
      char dateTime[32];
      sprintf(dateTime, "%04d-%02d-%02d %02d:%02d:%02d", 
              gps.date.year(), gps.date.month(), gps.date.day(),
              gps.time.hour(), gps.time.minute(), gps.time.second());
      M5.Display.println(dateTime);
    } else {
      M5.Display.println("Date/Time: Invalid");
    }
    
    // Show location if available
    if (gps.location.isValid()) {
      M5.Display.printf("Lat: %.6f\n", gps.location.lat());
      M5.Display.printf("Lng: %.6f\n", gps.location.lng());
    } else {
      M5.Display.println("Location: Invalid");
    }
    
    // Show altitude if available
    if (gps.altitude.isValid()) {
      M5.Display.printf("Alt: %.1fm\n", gps.altitude.meters());
    } else {
      M5.Display.println("Altitude: Invalid");
    }
    
    // Show speed if available
    if (gps.speed.isValid()) {
      M5.Display.printf("Speed: %.1fkm/h\n", gps.speed.kmph());
    } else {
      M5.Display.println("Speed: Invalid");
    }
    
    // Show satellite info
    M5.Display.printf("Satellites: %d\n", gps.satellites.value());
    
    // Show HDOP (horizontal dilution of precision) if available
    if (gps.hdop.isValid()) {
      M5.Display.printf("HDOP: %.1f\n", gps.hdop.hdop());
    }
    
    // Show SD card logging status
    M5.Display.println("");
    if (sd_available) {
      M5.Display.setTextColor(GREEN);
      M5.Display.printf("SD: Logging to %s\n", log_filename.c_str());
    } else {
      M5.Display.setTextColor(RED);
      M5.Display.println("SD: Not available");
    }
    
    return;  // Skip the rest of the normal display
  }
  
  // Normal display mode below
  // Use static variables to track previous values and reduce flicker
  static double prev_speed = -1;
  static double prev_dilation = -1;
  static double prev_device_time = -1;
  static double prev_relative_time = -1;
  static double prev_difference = -1;
  static int prev_gps_status = -1;
  static bool prev_sensor_mode = !using_gps_data;
  static bool prev_calibration = !acc_calibrated;
  static uint32_t prev_gps_datetime = 0;
  static uint32_t prev_real_time = 0;
  
  // Check if we need to update date and time display
  uint32_t current_second = millis() / 1000;
  if (current_second != prev_real_time || 
      (gps.time.isValid() && gps.time.second() != prev_gps_datetime)) {
    
    // Update real-world time
    int yPos = isSmallDisplay ? 10 : 15;
    
    // Get current time
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    char timeStr[32];
    
    // Format and display real-world time
    M5.Display.fillRect(0, yPos, displayWidth/2 - 2, 8, BLACK);
    M5.Display.setCursor(0, yPos);
    M5.Display.setTextColor(YELLOW);
    sprintf(timeStr, "R:%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    M5.Display.print(timeStr);
    
    // Format and display GPS time if valid
    if (gps.time.isValid() && gps.date.isValid()) {
      M5.Display.fillRect(displayWidth/2, yPos, displayWidth/2, 8, BLACK);
      M5.Display.setCursor(displayWidth/2, yPos);
      M5.Display.setTextColor(MAGENTA);
      
      // Apply time dilation effect to GPS time if moving
      if (current_speed > 0.5 && time_dilation > 1.001) {
        // Create a "dilated" time by subtracting the time difference
        int virtual_hour = gps.time.hour();
        int virtual_minute = gps.time.minute();
        int virtual_second = gps.time.second();
        
        // Apply time difference (simplified)
        int time_shift = (int)time_difference;
        virtual_second -= time_shift % 60;
        if (virtual_second < 0) {
          virtual_second += 60;
          virtual_minute--;
        }
        
        virtual_minute -= (time_shift / 60) % 60;
        if (virtual_minute < 0) {
          virtual_minute += 60;
          virtual_hour--;
        }
        
        virtual_hour -= (time_shift / 3600) % 24;
        if (virtual_hour < 0) {
          virtual_hour += 24;
        }
        
        sprintf(timeStr, "V:%02d:%02d:%02d", virtual_hour, virtual_minute, virtual_second);
      } else {
        // Just show GPS time when not moving or no significant dilation
        sprintf(timeStr, "V:%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      }
      
      M5.Display.print(timeStr);
    }
    
    // Update prev values
    prev_real_time = current_second;
    if (gps.time.isValid()) {
      prev_gps_datetime = gps.time.second();
    }
    
    M5.Display.setTextColor(WHITE);
  }
  
  int current_gps_status = 0;
  if (!gpsDataReceived) current_gps_status = 0;  // Not Connected
  else if (millis() - lastGpsDataTime > 5000) current_gps_status = 1;  // No Signal
  else if (gps.location.isValid()) current_gps_status = 3;  // Connected
  else current_gps_status = 2;  // Acquiring

  // Only update GPS status if changed
  if (prev_gps_status != current_gps_status) {
    if (isSmallDisplay) {
      // Compact GPS status for small displays - moved below title area
      M5.Display.fillRect(2, 22, 45, 8, BLACK);
      M5.Display.setCursor(2, 22);
    } else {
      M5.Display.fillRect(displayWidth - 80, 10, 80, 10, NAVY);
      M5.Display.setCursor(displayWidth - 80, 10);
    }
    M5.Display.print("GPS:");
    
    switch(current_gps_status) {
      case 0:
        M5.Display.setTextColor(RED);
        M5.Display.println(isSmallDisplay ? "NO" : "NO CONN");
        break;
      case 1:
        M5.Display.setTextColor(YELLOW);
        M5.Display.println(isSmallDisplay ? "NS" : "NO SIGNAL");
        break;
      case 2:
        M5.Display.setTextColor(BLUE);
        M5.Display.println(isSmallDisplay ? "AQ" : "ACQUIRING");
        break;
      case 3:
        M5.Display.setTextColor(GREEN);
        M5.Display.println(isSmallDisplay ? "OK" : "CONNECTED");
        break;
    }
    M5.Display.setTextColor(WHITE);
    prev_gps_status = current_gps_status;
  }
  
  // Only update speed if changed significantly
  if (abs(prev_speed - current_speed) > 0.1) {
    int yPos = isSmallDisplay ? 40 : 60;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 16, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPD:");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(2, yPos+8);
      M5.Display.printf("    %.6f km/s", current_speed / 3600.0);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 20, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPEED: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(5, yPos+10);
      M5.Display.printf("       %.6f km/s", current_speed / 3600.0);
    }
    prev_speed = current_speed;
  }
  
  // Only update time dilation if changed
  if (abs(prev_dilation - time_dilation) > 0.00001) {
    int yPos = isSmallDisplay ? 58 : 85;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DIL:");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.6f", time_dilation);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DILATION: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.6f", time_dilation);
    }
    prev_dilation = time_dilation;
  }
  
  // Draw separator line
  static bool line_drawn = false;
  if (!line_drawn) {
    int yPos = isSmallDisplay ? 68 : 100;
    M5.Display.drawLine(0, yPos, displayWidth, yPos, CYAN);
    line_drawn = true;
  }
  
  // Only update time displays if changed significantly
  if (abs(prev_device_time - elapsed_device_time) > 0.1) {
    int yPos = isSmallDisplay ? 70 : 105;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEV:");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f s", elapsed_device_time);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEVICE TIME: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f s", elapsed_device_time);
    }
    prev_device_time = elapsed_device_time;
  }
  
  if (abs(prev_relative_time - elapsed_relativistic_time) > 0.1) {
    int yPos = isSmallDisplay ? 78 : 115;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL:");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f s", elapsed_relativistic_time);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL TIME: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f s", elapsed_relativistic_time);
    }
    prev_relative_time = elapsed_relativistic_time;
  }
  
  // Only update time difference if changed significantly
  if (abs(prev_difference - time_difference) > 0.1) {
    int yPos = isSmallDisplay ? 90 : 130;
    int boxHeight = isSmallDisplay ? 12 : 15;
    
    M5.Display.fillRect(0, yPos, displayWidth, boxHeight, BLACK);
    
    // Show time difference with box
    int boxWidth = displayWidth - (isSmallDisplay ? 8 : 20);
    int boxX = isSmallDisplay ? 4 : 10;
    int boxY = yPos;
    
    // Draw box
    M5.Display.fillRect(boxX, boxY, boxWidth, boxHeight, DARKGREY);
    
    // Use absolute value of time difference for consistent display
    float absDiff = abs(time_difference);
    
    if (time_difference > 0) {
      // Relativistic time is behind normal time
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, YELLOW);
      M5.Display.setTextColor(YELLOW);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:-%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: -%.1f s", absDiff);
      }
    } else {
      // Relativistic time is ahead of normal time
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, CYAN);
      M5.Display.setTextColor(CYAN);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:+%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: +%.1f s", absDiff);
      }
    }
    
    M5.Display.setTextColor(WHITE);
    prev_difference = time_difference;
  }
  
  // Always show reset button with nice styling
  static bool reset_shown = false;
  if (!reset_shown) {
    int btnY = isSmallDisplay ? 110 : (displayHeight - 20);
    if (btnY < (isSmallDisplay ? 110 : 150)) btnY = isSmallDisplay ? 110 : 150; // Ensure button is visible
    
    M5.Display.fillRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, NAVY);
    M5.Display.drawRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, CYAN);
    
    // Center the text
    String btnText = isSmallDisplay ? "TOUCH TO RESET" : "TOUCH HERE TO RESET";
    int textWidth = btnText.length() * 6; // Approximate width
    int textX = (displayWidth - textWidth) / 2;
    
    M5.Display.setCursor(textX > 0 ? textX : 0, btnY + (isSmallDisplay ? 4 : 6));
    M5.Display.setTextColor(WHITE);
    M5.Display.print(btnText);
    
    reset_shown = true;
  }
  
  // Display drift factor occasionally if using accelerometer
  static unsigned long last_drift_update = 0;
  if (!using_gps_data && acc_calibrated && millis() - last_drift_update > 5000) {
    // Find a good place to show the calibration info
    int yPos = isSmallDisplay ? 105 : (displayHeight - 35);
    
    M5.Display.fillRect(0, yPos, displayWidth, 10, BLACK);
    M5.Display.setTextColor(CYAN);
    M5.Display.setCursor(5, yPos);
    M5.Display.printf("CALIB: %.2fx", acc_drift_factor);
    M5.Display.setTextColor(WHITE);
    
    last_drift_update = millis();
  }
}

void setup() {
  // For debugging - initialize Serial first
  Serial.begin(115200);
  delay(500); // Give serial time to start
  Serial.println("\n\n=== Urashima Effect Device starting... ===");
  
  // Initialize M5Unified with auto-detection
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;  // default=115200. if "Serial" is old, set it to 9600.
  cfg.clear_display = true;      // clear the screen when boot
  M5.begin(cfg);
  
  Serial.println("M5 initialized");
  
  // Synchronize time if possible - use a default time if not set
  if (time(nullptr) < 1000000) {
    // Set a default time if not set yet
    struct tm timeinfo;
    timeinfo.tm_year = 2025 - 1900;  // Year - 1900
    timeinfo.tm_mon = 2 - 1;         // Month, 0-based
    timeinfo.tm_mday = 1;            // Day of the month
    timeinfo.tm_hour = 12;           // Hours
    timeinfo.tm_min = 0;             // Minutes
    timeinfo.tm_sec = 0;             // Seconds
    
    time_t t = mktime(&timeinfo);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
    
    Serial.println("Default time set");
  }
  
  // Initialize display
  // Set initial display content
  if (M5.Display.width() > 0) {  // Check if display is available
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // ATOMS3R has a 128x128 display
    if (isSmallDisplay) {
      Serial.println("ATOMS3R display detected");
    }
    
    M5.Display.setTextSize(1);
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    
    // Display cool title - adjusted for small screens
    M5.Display.fillRect(0, 0, displayWidth, isSmallDisplay ? 20 : 30, NAVY);
    
    if (isSmallDisplay) {
      // Smaller title for 128x128 displays - centered
      M5.Display.setTextSize(1);
      int titleX = (displayWidth - 15 * 6) / 2; // Centering "URASHIMA EFFECT"
      M5.Display.setCursor(titleX > 0 ? titleX : 0, 7);
      M5.Display.setTextColor(CYAN);
      M5.Display.println("URASHIMA EFFECT");
    } else {
      // Larger title for bigger displays
      M5.Display.setTextSize(2);
      int titleX = (displayWidth - 17 * 12) / 2; // Centering "URASHIMA EFFECT"
      M5.Display.setCursor(titleX > 0 ? titleX : 0, 5);
      M5.Display.setTextColor(CYAN);
      M5.Display.println("URASHIMA EFFECT");
    }
    M5.Display.setTextSize(1);
    
    // Draw line below title
    M5.Display.drawLine(0, isSmallDisplay ? 20 : 30, displayWidth, isSmallDisplay ? 20 : 30, CYAN);
    
    // Light speed info with better formatting - adjusted for screen size
    if (isSmallDisplay) {
      // For small displays, more compact information
      M5.Display.setCursor(50, 22);  // Moved right to avoid overlap with GPS status
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792.458");
      
      M5.Display.setCursor(50, 30);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.3");
      
      // Draw another line
      M5.Display.drawLine(0, 38, displayWidth, 38, CYAN);
    } else {
      // For larger displays, more spaced information
      M5.Display.setCursor(5, 35);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792.458 km/s");
      
      M5.Display.setCursor(5, 45);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.3 km/s");
      
      // Draw another line
      M5.Display.drawLine(0, 55, displayWidth, 55, CYAN);
    }
    
    updateDisplay(); // Initial display update
    Serial.println("Display initialized");
  } else {
    Serial.println("No display detected");
  }
  
  // Initialize SD card on AtomicBase GPS
  Serial.println("Initializing SD card on AtomicBase GPS...");
  SPI.begin(23, 33, 19, SD_CS_PIN); // SCLK, MISO, MOSI, CS for AtomicBase GPS
  initSDCard();
  
  // Initialize GPS - use Serial1 for AtomicBase GPS
  Serial.println("Initializing GPS on AtomicBase");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Waiting for GPS data...");
  
  // Initialize IMU
  Serial.println("Initializing IMU...");
  // Try M5.Imu
  if (M5.Imu.begin()) {
    Serial.println("M5.Imu initialized successfully");
    imu_available = true;
    acc_last_time = millis();
  } else {
    Serial.println("No IMU detected, will use simulation mode");
  }
  
  last_update = millis();
}

void loop() {
  // Loop updates
  M5.update();  // Update button and touch states
  
  // Read GPS data
  bool gps_updated = false;
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    
    if (gps.encode(c)) {
      gpsDataReceived = true;
      lastGpsDataTime = millis();
      
      // Log GPS info when location updates
      if (gps.location.isUpdated()) {
        Serial.printf("GPS Location: %.6f, %.6f Alt: %.1f Sats: %d\n", 
                      gps.location.lat(), gps.location.lng(), 
                      gps.altitude.meters(), gps.satellites.value());
      }
      
      // Check if speed was updated
      if (gps.speed.isUpdated()) {
        gps_updated = true;
        gps_speed = gps.speed.kmph();
        Serial.printf("GPS Speed: %.2f km/h\n", gps_speed);
      }
    }
  }
  
  // Update screen periodically
  if (millis() - last_update > 500) {  // Reduce update frequency to 500ms
    unsigned long current_millis = millis();
    double delta_t = (current_millis - last_update) / 1000.0;  // Time elapsed since last update (seconds)
    
    // Update device time
    elapsed_device_time += delta_t;
    
    // Check if GPS is valid
    bool gps_valid = gps.speed.isValid() && (millis() - lastGpsDataTime < GPS_TIMEOUT);
    
    // Get accelerometer data regardless of mode
    updateAccelerometerData();
    
    // Auto mode selection and sensor fusion
    if (gps_valid) {
      // We have good GPS data
      using_gps_data = true;
      current_speed = gps_speed;
      
      // Calibrate accelerometer based on GPS data if we've had GPS for some time
      static unsigned long gps_stable_time = 0;
      if (!acc_calibrated) {
        if (gps_stable_time == 0) {
          gps_stable_time = millis();
        } else if (millis() - gps_stable_time > ACC_CALIBRATION_TIME) {
          calibrateAccelerometer();
        }
      } else {
        // Already calibrated, keep adjusting the drift
        adjustAccelerometerDrift();
      }
    } else {
      // No valid GPS, switch to accelerometer
      using_gps_data = false;
      
      // Use calibrated accelerometer data if possible
      current_speed = acc_speed;
      
      // Log occasional debug info
      static unsigned long last_mode_log = 0;
      if (millis() - last_mode_log > 5000) {
        Serial.printf("Using accelerometer data: %.2f km/h (drift factor: %.3f)\n", 
                      acc_speed, acc_drift_factor);
        last_mode_log = millis();
      }
    }
    
    // Calculate relativistic effect
    calculateRelativisticEffect(delta_t);
    
    // Update display
    updateDisplay();
    
    // Log data to SD card
    logData();
    
    last_update = current_millis;
  }
  
  // Reset with physical button pressed - using M5Unified's BtnA
  static unsigned long lastBtnAPress = 0;
  if (M5.BtnA.wasPressed() && millis() - lastBtnAPress > 1000) { // 1 second debounce
    resetTimeCalculation();
    Serial.println("Reset via M5.BtnA");
    lastBtnAPress = millis();
  }

  // Manual Calibration with long press
  if (M5.BtnA.wasHold()) {
    static bool actionProcessed = false;
    if (!actionProcessed) {
      if (gps.speed.isValid()) {
        calibrateAccelerometer();
        Serial.println("Manually calibrated accelerometer with GPS data");
      } else {
        Serial.println("Cannot calibrate - no valid GPS data");
      }
      actionProcessed = true;
    }
  } else {
    static bool actionProcessed = false;
    actionProcessed = false;
  }

  // Toggle GPS raw data display with short press (using M5.BtnA)
  if (M5.BtnA.wasClicked()) {
    static unsigned long lastToggle = 0;
    if (millis() - lastToggle > 500) { // Debounce
      show_raw_gps = !show_raw_gps;
      Serial.printf("GPS raw data display: %s\n", show_raw_gps ? "ON" : "OFF");
      lastToggle = millis();
    }
  }
}
