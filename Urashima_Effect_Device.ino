#include <M5Unified.h>    // Unified library for M5Stack devices
#include <TinyGPS++.h>    // GPS parsing library
#include <SPI.h>          // SPI library
#include <Preferences.h>  // Library for non-volatile memory
#include <SD.h>           // SD card library

// Constants
const double ORIGINAL_LIGHT_SPEED = 299792.458;  // Actual speed of light (km/s)
const double MODIFIED_LIGHT_SPEED = 0.03;       // Modified speed of light (km/s)

// GPS connection pin settings
#define GPS_RX_PIN 5     // ATOMS3R GPIO5 for GPS TX (receiving data from GPS)
#define GPS_TX_PIN -1    // GPS RX is not connected

// SD Card pin settings for AtomS3R with AtomicBase GPS
#define SD_MOSI_PIN 6    // MOSI pin (G6) for SD card
#define SD_MISO_PIN 8    // MISO pin (G8) for SD card
#define SD_SCK_PIN 7     // CLK pin (G7) for SD card
#define SD_CS_PIN 38     // CS pin for SD card (M5AtomS3R)
#define SD_LOG_INTERVAL 5000  // Log data every 5 seconds
#define SD_LOG_FILENAME "/urashima_log.csv"  // Log file name - must start with /

// IMU related constants
#define USE_IMU_WHEN_GPS_LOST true  // Use IMU when GPS signal is lost
#define IMU_ACCEL_THRESHOLD 0.05f   // Acceleration threshold (movement detected above this value)
#define IMU_GYRO_THRESHOLD 1.0f     // Gyroscope threshold (rotation detected above this value)
#define IMU_SPEED_DECAY 0.98f       // Speed decay factor (simulates friction and air resistance)
#define IMU_REST_THRESHOLD 0.03f    // Threshold to detect when device is at rest
#define IMU_REST_DURATION 500       // Duration in ms to consider device at rest
#define IMU_DRIFT_CORRECTION 0.02f  // Drift correction factor (higher = faster correction)
#define IMU_CALIBRATION_TIME 5000   // IMU calibration time (milliseconds)
#define IMU_CALIBRATION_SAMPLES 100 // Number of samples for calibration
#define GPS_IMU_FUSION_WEIGHT 0.2f  // Weight for sensor fusion (0.0 = IMU only, 1.0 = GPS only)
#define GPS_VALID_TIMEOUT 5000      // Time in ms after which GPS data is considered stale
#define SAVE_DATA_INTERVAL 60000    // Interval to save data to persistent storage (60 seconds)

// TinyGPS++ object
TinyGPSPlus gps;

// Hardware serial
HardwareSerial GPSSerial(1);  // Using UART1

// Global variables
double current_speed = 0.0;  // Current speed (km/h)
double time_dilation = 1.0;  // Time dilation factor
double elapsed_device_time = 0.0;  // Elapsed device time (seconds)
double elapsed_relativistic_time = 0.0;  // Elapsed relativistic time (seconds)
double time_difference = 0.0;  // Time difference (seconds)
unsigned long last_update = 0;  // Last update time
bool gpsDataReceived = false;  // Whether GPS signal has been received
unsigned long lastGpsDataTime = 0;  // Last time GPS data was received
bool show_raw_gps = false;  // Whether to show raw GPS data (kept for compatibility)
bool show_raw_imu = false;  // Whether to show raw IMU data (kept for compatibility)
int display_mode = 0;  // Display mode (0:Main, 1:GPS, 2:IMU)
double total_distance = 0.0;  // Total distance traveled (km)
unsigned long last_persistent_save = 0;  // Last time data was saved to persistent storage
unsigned long last_distance_update = 0;  // Last time distance was updated

// Preferences object for persistent storage
Preferences preferences;

// Display update tracking variables
double prev_device_time = 0.0;     // Previous device time for display updates
double prev_rel_time = 0.0;        // Previous relativistic time for display updates
double prev_time_diff = 0.0;       // Previous time difference for display updates

// IMU related variables
bool imuInitialized = false;
bool imuCalibrated = false;
bool usingImuForSpeed = false;     // Whether using IMU for speed measurement
float imuAccelOffset[3] = {0, 0, 0};
float imuGyroOffset[3] = {0, 0, 0};
float imuVelocity[3] = {0, 0, 0};
float imuSpeed = 0;
float currentAccelX = 0, currentAccelY = 0, currentAccelZ = 0;
float currentGyroX = 0, currentGyroY = 0, currentGyroZ = 0;
float filteredAccel[3] = {0, 0, 0};
unsigned long lastImuUpdate = 0;
unsigned long restStartTime = 0;
bool isAtRest = false;
float gpsImuFusedSpeed = 0;        // Speed calculated from GPS-IMU fusion
float lastValidGpsSpeed = 0;       // Last valid GPS speed reading
bool hasValidGpsSpeed = false;     // Whether we have a valid GPS speed reading

// SD card related variables
bool sdCardInitialized = false;
File logFile;
unsigned long lastLogTime = 0;

void setup() {
  // Initialize M5 device
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Urashima Effect Device Starting...");
  
  // Initialize display
  if (M5.Display.width() > 0) {
    Serial.println("Display detected");
    
    // Clear screen
    M5.Display.fillScreen(BLACK);
    
    // Check if we're using a small display
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // Draw title bar
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
    
    // Display light speed information
    if (isSmallDisplay) {
      // For small displays, more compact information
      M5.Display.setCursor(50, 22);  // Moved right to avoid overlap with GPS status
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792");
      
      M5.Display.setCursor(50, 30);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.03");
      
      // Draw another line
      M5.Display.drawLine(0, 38, displayWidth, 38, CYAN);
    } else {
      // For larger displays, more spaced information
      M5.Display.setCursor(5, 35);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792 km/s");
      
      M5.Display.setCursor(5, 45);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.03 km/s");
      
      // Draw another line
      M5.Display.drawLine(0, 55, displayWidth, 55, CYAN);
    }
    
    // Initialize time variables
    elapsed_device_time = 0.0;
    elapsed_relativistic_time = 0.0;
    time_difference = 0.0;
    time_dilation = 1.0;
    
    // Initialize display tracking variables
    prev_device_time = 0.0;
    prev_rel_time = 0.0;
    prev_time_diff = 0.0;
    
    // Initialize last update time
    last_update = millis();
    
    // Initialize display
    forceCompleteRedraw();
    Serial.println("Display initialized");
  } else {
    Serial.println("No display detected");
  }
  
  // Initialize GPS
  Serial.println("Initializing GPS");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // RX=2, TX=-1 (not used)
  
  Serial.println("Waiting for GPS data...");
  
  // Initialize IMU
  initializeImu();
  
  // Initialize SD card with explicit SPI settings
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  initSDCard();
  
  // Initialize persistent storage
  preferences.begin("urashima", false);
  
  // Load saved data from persistent storage
  loadSavedData();
}

void loop() {
  M5.update();  // Update button and touch states
  
  // Read GPS data
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
        
        // Save last valid GPS location to persistent storage
        if (gps.location.isValid()) {
          preferences.putDouble("last_lat", gps.location.lat());
          preferences.putDouble("last_lng", gps.location.lng());
          preferences.putFloat("last_alt", gps.altitude.meters());
          preferences.putUInt("last_gps_time", lastGpsDataTime);
          
          // Only log to serial occasionally to avoid flooding
          if (millis() % 10000 < 100) {  // Log approximately every 10 seconds
            Serial.println("GPS location saved to persistent storage");
          }
        }
      }
      
      // Log speed when updated
      if (gps.speed.isUpdated()) {
        Serial.printf("GPS Speed: %.1f km/h Course: %.1f°\n", 
                     gps.speed.kmph(), gps.course.deg());
      }
    }
  }
  
  // Update IMU data if initialized
  if (imuInitialized) {
    updateImuData();
  }
  
  // Handle button presses - cycle through display modes
  if (M5.BtnA.wasClicked()) {  // Using wasClicked instead of wasPressed for better response
    // Switch to next display mode
    display_mode = (display_mode + 1) % 3;  // Cycle through 0→1→2→0...
    
    // Update legacy variables for compatibility
    show_raw_gps = (display_mode == 1);
    show_raw_imu = (display_mode == 2);
    
    // Output display mode change to serial
    switch(display_mode) {
      case 0:
        Serial.println("Display Mode: Main");
        break;
      case 1:
        Serial.println("Display Mode: GPS Raw Data");
        break;
      case 2:
        Serial.println("Display Mode: IMU Raw Data");
        break;
    }
    
    // Clear the entire screen before redrawing to prevent display corruption
    M5.Display.fillScreen(BLACK);
    
    // Reset display cache and force complete redraw
    resetDisplayCache();
    forceCompleteRedraw();
    
    // Call updateDisplay directly to ensure immediate update
    updateDisplay();
  }
  
  // Handle button long press for time reset
  static bool resetConfirmMode = false;
  static unsigned long resetConfirmTime = 0;
  
  // Check for long press on BtnA (2 seconds)
  if (M5.BtnA.pressedFor(2000)) {
    if (!resetConfirmMode) {
      resetConfirmMode = true;
      resetConfirmTime = millis();
      
      // Display reset confirmation message
      int displayWidth = M5.Display.width();
      int displayHeight = M5.Display.height();
      bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
      
      // Position at bottom of screen
      int messageY = isSmallDisplay ? (displayHeight - 40) : (displayHeight - 60);
      
      // Draw confirmation box
      M5.Display.fillRect(0, messageY, displayWidth, isSmallDisplay ? 40 : 60, RED);
      M5.Display.setTextColor(WHITE, RED);
      
      if (isSmallDisplay) {
        M5.Display.setCursor(5, messageY + 5);
        M5.Display.println("RESET TIME?");
        M5.Display.setCursor(5, messageY + 20);
        M5.Display.println("Press A to confirm");
      } else {
        M5.Display.setCursor(10, messageY + 10);
        M5.Display.println("RESET DEVICE TIME?");
        M5.Display.setCursor(10, messageY + 30);
        M5.Display.println("Press Button A again to confirm");
      }
      
      Serial.println("Time reset confirmation requested");
    }
  }
  
  // If in reset confirmation mode
  if (resetConfirmMode) {
    // If button A is pressed again while in confirmation mode, reset the time
    if (M5.BtnA.wasPressed()) {
      // Reset time variables
      elapsed_device_time = 0.0;
      elapsed_relativistic_time = 0.0;
      time_difference = 0.0;
      time_dilation = 1.0;
      
      // Reset distance variable
      total_distance = 0.0;
      
      // Reset display tracking variables
      prev_device_time = 0.0;
      prev_rel_time = 0.0;
      prev_time_diff = 0.0;
      
      // Clear confirmation message
      M5.Display.fillScreen(BLACK);
      forceCompleteRedraw();
      
      // Display reset confirmation
      int displayWidth = M5.Display.width();
      int displayHeight = M5.Display.height();
      bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
      
      // Center of screen
      int messageY = displayHeight / 2 - 10;
      
      M5.Display.fillRect(0, messageY, displayWidth, 20, GREEN);
      M5.Display.setTextColor(BLACK, GREEN);
      M5.Display.setCursor(isSmallDisplay ? 15 : 50, messageY + 5);
      M5.Display.println("TIME RESET COMPLETE");
      
      Serial.println("Device time reset completed");
      
      // Exit reset confirmation mode
      resetConfirmMode = false;
      delay(1500);  // Show message for 1.5 seconds
      forceCompleteRedraw();
    }
    
    // If no confirmation within 5 seconds, cancel reset
    if (millis() - resetConfirmTime > 5000) {
      resetConfirmMode = false;
      
      // Clear confirmation message
      M5.Display.fillScreen(BLACK);
      
      // Display cancellation message
      int displayWidth = M5.Display.width();
      int displayHeight = M5.Display.height();
      bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
      
      // Center of screen
      int messageY = displayHeight / 2 - 10;
      
      M5.Display.fillRect(0, messageY, displayWidth, 20, YELLOW);
      M5.Display.setTextColor(BLACK, YELLOW);
      M5.Display.setCursor(isSmallDisplay ? 15 : 50, messageY + 5);
      M5.Display.println("RESET CANCELLED");
      
      Serial.println("Time reset cancelled (timeout)");
      
      // Show message for 1.5 seconds then redraw
      delay(1500);
      forceCompleteRedraw();
    }
  }
  
  // Update screen periodically
  if (millis() - last_update > 500) {  // Reduce update frequency to 500ms
    unsigned long current_millis = millis();
    double delta_t = (current_millis - last_update) / 1000.0;  // Time elapsed since last update (seconds)
    
    // Update device time
    elapsed_device_time += delta_t;
    
    // Update speed based on GPS or IMU
    updateSpeed();
    
    // Calculate relativistic effect
    calculateRelativisticEffect(delta_t);
    
    // Update distance calculation
    updateDistance();
    
    // Update display
    updateDisplay();
    
    last_update = current_millis;
  }
  
  // Log data to SD card periodically
  if (sdCardInitialized && (millis() - lastLogTime > SD_LOG_INTERVAL)) {
    logDataToSD();
    lastLogTime = millis();
  }
}

// Calculate relativistic effect
void calculateRelativisticEffect(double delta_t) {
  // Convert speed to km/s
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

// Update display
void updateDisplay() {
  // Check if display is available
  if (M5.Display.width() == 0) {
    return;
  }
  
  // Get display dimensions
  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
  
  // Calculate GPS status
  int current_gps_status = 0;
  static int prev_gps_status = -1;
  static double prev_speed = -1.0;
  static double prev_time_dilation = -1.0;
  static double prev_time_diff = -1.0;
  
  if (!gpsDataReceived) {
    current_gps_status = 0; // No connection
  } else if (millis() - lastGpsDataTime > 5000) {
    current_gps_status = 1; // No signal (timeout)
  } else if (!gps.location.isValid() || !gps.speed.isValid()) {
    current_gps_status = 2; // Acquiring
  } else {
    current_gps_status = 3; // Connected with valid data
  }
  
  // Update display based on display mode
  if (display_mode == 1 || show_raw_gps) {  // GPS raw data display mode
    // Clear the main area (below the title)
    int startY = isSmallDisplay ? 21 : 31;
    M5.Display.fillRect(0, startY, displayWidth, displayHeight - startY, BLACK);
    
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(2, startY + 2);
    M5.Display.println("RAW GPS DATA:");
    M5.Display.setTextColor(WHITE);
    
    int y = startY + 12;
    
    // Satellites
    M5.Display.setCursor(2, y);
    M5.Display.print("Sats:");
    if (gps.satellites.isValid()) {
      M5.Display.print(gps.satellites.value());
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Location
    M5.Display.setCursor(2, y);
    M5.Display.print("Lat:");
    if (gps.location.isValid()) {
      M5.Display.print(gps.location.lat(), 6);
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    M5.Display.setCursor(2, y);
    M5.Display.print("Lng:");
    if (gps.location.isValid()) {
      M5.Display.print(gps.location.lng(), 6);
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Altitude
    M5.Display.setCursor(2, y);
    M5.Display.print("Alt:");
    if (gps.altitude.isValid()) {
      M5.Display.print(gps.altitude.meters(), 1);
      M5.Display.print("m");
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Speed
    M5.Display.setCursor(2, y);
    M5.Display.print("Spd:");
    if (gps.speed.isValid()) {
      M5.Display.print(gps.speed.kmph(), 1);
      M5.Display.print("km/h");
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Course
    M5.Display.setCursor(2, y);
    M5.Display.print("Crs:");
    if (gps.course.isValid()) {
      M5.Display.print(gps.course.deg(), 1);
      M5.Display.print("°");
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Date & Time
    M5.Display.setCursor(2, y);
    M5.Display.print("Date:");
    if (gps.date.isValid()) {
      char dateStr[12];
      sprintf(dateStr, "%04d-%02d-%02d", 
              gps.date.year(), gps.date.month(), gps.date.day());
      M5.Display.print(dateStr);
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // Time on separate line
    M5.Display.setCursor(2, y);
    M5.Display.print("Time:");
    if (gps.time.isValid()) {
      char timeStr[10];
      sprintf(timeStr, "%02d:%02d:%02d", 
              gps.time.hour(), gps.time.minute(), gps.time.second());
      M5.Display.print(timeStr);
    } else {
      M5.Display.print("--");
    }
    y += 8;
    
    // HDOP (Horizontal Dilution of Precision)
    M5.Display.setCursor(2, y);
    M5.Display.print("HDOP:");
    if (gps.hdop.isValid()) {
      M5.Display.print(gps.hdop.hdop(), 1);
    } else {
      M5.Display.print("--");
    }
    
    return; // Skip the normal display update
  }
  
  // Display raw IMU data if requested
  if (display_mode == 2 || show_raw_imu) {  // IMU raw data display mode
    // Clear the main area (below the title)
    int startY = isSmallDisplay ? 21 : 31;
    M5.Display.fillRect(0, startY, displayWidth, displayHeight - startY, BLACK);
    
    M5.Display.setTextColor(MAGENTA);
    // Adjust the title position to be higher
    int centerY = startY - 15;  // Moved up even more
    M5.Display.setCursor(2, centerY);
    M5.Display.println("RAW IMU DATA:");
    M5.Display.setTextColor(WHITE);
    
    int y = centerY + 10;
    
    // Accelerometer data
    M5.Display.setTextColor(YELLOW);
    M5.Display.setCursor(2, y);
    M5.Display.println("Accelerometer (G):");
    M5.Display.setTextColor(WHITE);
    y += 9; // Reduced spacing
    
    // Display X and Y on the same line
    M5.Display.setCursor(2, y);
    M5.Display.printf("X: %.3f  Y: %.3f", currentAccelX, currentAccelY);
    y += 9; // Reduced spacing
    
    // Display Z on its own line
    M5.Display.setCursor(2, y);
    M5.Display.printf("Z: %.3f", currentAccelZ);
    y += 12; // Reduced spacing
    
    // Gyroscope data
    M5.Display.setTextColor(YELLOW);
    M5.Display.setCursor(2, y);
    M5.Display.println("Gyroscope (deg/s):");
    M5.Display.setTextColor(WHITE);
    y += 9; // Reduced spacing
    
    // Display X and Y on the same line
    M5.Display.setCursor(2, y);
    M5.Display.printf("X: %.3f  Y: %.3f", currentGyroX, currentGyroY);
    y += 9; // Reduced spacing
    
    // Display Z on its own line
    M5.Display.setCursor(2, y);
    M5.Display.printf("Z: %.3f", currentGyroZ);
    y += 12; // Reduced spacing
    
    // Calculated velocity
    M5.Display.setTextColor(YELLOW);
    M5.Display.setCursor(2, y);
    M5.Display.println("Velocity (m/s):");
    M5.Display.setTextColor(WHITE);
    y += 9; // Reduced spacing
    
    // Display X and Y on the same line
    M5.Display.setCursor(2, y);
    M5.Display.printf("X: %.3f  Y: %.3f", imuVelocity[0], imuVelocity[1]);
    y += 9; // Reduced spacing
    
    // Display Z on its own line
    M5.Display.setCursor(2, y);
    M5.Display.printf("Z: %.3f", imuVelocity[2]);
    y += 12; // Reduced spacing
    
    // Display total speed with green color for better visibility
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(2, y);
    M5.Display.printf("Speed: %.2f km/h", imuSpeed);
    
    return; // Skip the normal display update
  }
  
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
      M5.Display.print("SPD");
      
      // IMU is being used for speed measurement
      if (usingImuForSpeed) {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print("(IMU)");
      }
      
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print(":");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(2, yPos+8);
      M5.Display.printf("    %.4f km/s", current_speed / 3600.0);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 20, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPEED");
      
      // IMU is being used for speed measurement
      if (usingImuForSpeed) {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print(" (IMU)");
      }
      
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print(": ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(5, yPos+10);
      M5.Display.printf("       %.5f km/s", current_speed / 3600.0);
    }
    prev_speed = current_speed;
  }
  
  // Only update time dilation if changed
  if (abs(prev_time_dilation - time_dilation) > 0.00001) {
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
    prev_time_dilation = time_dilation;
  }
  
  // Draw separator line
  int lineYPos = isSmallDisplay ? 68 : 100;
  M5.Display.drawLine(0, lineYPos, displayWidth, lineYPos, CYAN);
  
  // Only update device time if changed significantly
  if (abs(elapsed_device_time - prev_device_time) > 0.1) {
    int yPos = isSmallDisplay ? 70 : 105;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEV:");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_device_time, timeStr);
      M5.Display.print(timeStr);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEVICE TIME: ");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_device_time, timeStr);
      M5.Display.print(timeStr);
    }
    prev_device_time = elapsed_device_time;
  }
  
  // Only update relativistic time if changed significantly
  if (abs(elapsed_relativistic_time - prev_rel_time) > 0.1) {
    int yPos = isSmallDisplay ? 78 : 115;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL:");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL TIME: ");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
    }
    prev_rel_time = elapsed_relativistic_time;
  }
  
  // Only update time difference if changed significantly
  if (abs(time_difference - prev_time_diff) > 0.1) {
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
    
    // Very small values (close to 0) are treated as 0
    if (absDiff < 0.05) {
      // Special display for values close to 0
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, GREEN);
      M5.Display.setTextColor(GREEN);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:0.0s");
      } else {
        M5.Display.printf("TIME DIFF: 0.0 s");
      }
    } else if (time_difference > 0) {
      // Relativistic time is slower than normal time (expected behavior)
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, YELLOW);
      M5.Display.setTextColor(YELLOW);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:-%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: -%.1f s", absDiff);
      }
    } else {
      // Relativistic time is faster than normal time (unexpected behavior)
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
    prev_time_diff = time_difference;
  }
  
  // Display total distance
  int distYPos = isSmallDisplay ? 105 : 150;
  M5.Display.fillRect(0, distYPos, displayWidth, 10, BLACK);
  M5.Display.setCursor(isSmallDisplay ? 4 : 10, distYPos);
  M5.Display.setTextColor(GREEN);
  M5.Display.print("TOTAL DIST: ");
  M5.Display.setTextColor(WHITE);
  M5.Display.print(total_distance, 2);
  M5.Display.print(" km");
  
  // Display SD card status
  int sdYPos = isSmallDisplay ? 115 : 160;
  M5.Display.fillRect(0, sdYPos, displayWidth, 10, BLACK);
  M5.Display.setCursor(isSmallDisplay ? 4 : 10, sdYPos);
  M5.Display.setTextColor(GREEN);
  M5.Display.print("SD: ");
  
  if (sdCardInitialized) {
    M5.Display.setTextColor(GREEN);
    M5.Display.print("OK");
  } else {
    M5.Display.setTextColor(RED);
    M5.Display.print("FAIL");
  }
}

// Reset time calculations
void resetTimeCalculation() {
  elapsed_device_time = 0.0;
  elapsed_relativistic_time = 0.0;
  time_difference = 0.0;
  
  // Reset message - minimal
  Serial.println("Time measurement reset");
}

// Reset display cache to force redraw
void resetDisplayCache() {
  // Safer method to force a complete redraw of the screen
  
  // Static variables cannot be directly reset, so
  // clear the screen and modify values to trigger a redraw
  
  // Clear the screen
  M5.Display.fillScreen(BLACK);
  
  // Modify values to trigger a redraw
  current_speed += 0.2;  // Modify speed to trigger redraw
  time_dilation += 0.00002;  // Modify time dilation to trigger redraw
  
  // Adjust time values to maintain consistency
  double old_device_time = elapsed_device_time;
  double old_relativistic_time = elapsed_relativistic_time;
  
  elapsed_device_time += 0.2;  // Modify elapsed device time
  elapsed_relativistic_time = old_relativistic_time + 
                             (elapsed_device_time - old_device_time) * time_dilation;  // Modify relativistic time
  
  // Recalculate time difference
  time_difference = elapsed_device_time - elapsed_relativistic_time;
  
  // Modify last GPS data time to trigger redraw
  lastGpsDataTime = millis() - 1000;
}

// Time display helper function
void formatTimeHMS(float seconds, char* buffer) {
  int hours = (int)(seconds / 3600);
  seconds -= hours * 3600;
  int minutes = (int)(seconds / 60);
  seconds -= minutes * 60;
  
  sprintf(buffer, "%02d:%02d:%05.2f", hours, minutes, seconds);
}

// Force complete redraw of all elements
void forceCompleteRedraw() {
  // Clear the screen first to prevent display corruption
  M5.Display.fillScreen(BLACK);
  
  // Only for normal display mode
  if (display_mode == 0) {
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // Redraw title bar
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
    
    // Redraw light speed information
    if (isSmallDisplay) {
      // For small displays, more compact information
      M5.Display.setCursor(50, 22);  // Moved right to avoid overlap with GPS status
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792");
      
      M5.Display.setCursor(50, 30);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.03");
      
      // Draw another line
      M5.Display.drawLine(0, 38, displayWidth, 38, CYAN);
    } else {
      // For larger displays, more spaced information
      M5.Display.setCursor(5, 35);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("REAL C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("299,792 km/s");
      
      M5.Display.setCursor(5, 45);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT C: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("0.03 km/s");
      
      // Draw another line
      M5.Display.drawLine(0, 55, displayWidth, 55, CYAN);
    }
    
    // Redraw GPS status
    int current_gps_status = 0;
    static int prev_gps_status = -1;
    static double prev_speed = -1.0;
    static double prev_time_dilation = -1.0;
    static double prev_time_diff = -1.0;
    
    if (!gpsDataReceived) {
      current_gps_status = 0; // No connection
    } else if (millis() - lastGpsDataTime > 5000) {
      current_gps_status = 1; // No signal (timeout)
    } else if (!gps.location.isValid() || !gps.speed.isValid()) {
      current_gps_status = 2; // Acquiring
    } else {
      current_gps_status = 3; // Connected with valid data
    }
    
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
    
    // Redraw speed information
    int yPos = isSmallDisplay ? 40 : 60;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 16, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPD");
      
      // IMU is being used for speed measurement
      if (usingImuForSpeed) {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print("(IMU)");
      }
      
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print(":");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(2, yPos+8);
      M5.Display.printf("    %.4f km/s", current_speed / 3600.0);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 20, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPEED");
      
      // IMU is being used for speed measurement
      if (usingImuForSpeed) {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print(" (IMU)");
      }
      
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print(": ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(5, yPos+10);
      M5.Display.printf("       %.5f km/s", current_speed / 3600.0);
    }
    
    // Redraw time dilation
    yPos = isSmallDisplay ? 58 : 85;
    
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
    
    // Draw separator line
    int lineYPos = isSmallDisplay ? 68 : 100;
    M5.Display.drawLine(0, lineYPos, displayWidth, lineYPos, CYAN);
    
    // Redraw device time
    yPos = isSmallDisplay ? 70 : 105;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEV:");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_device_time, timeStr);
      M5.Display.print(timeStr);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEVICE TIME: ");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_device_time, timeStr);
      M5.Display.print(timeStr);
    }
    
    yPos = isSmallDisplay ? 78 : 115;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL:");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 10, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("REL TIME: ");
      M5.Display.setTextColor(WHITE);
      
      // Display time in HH:MM:SS format
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
    }
    
    // Redraw time difference
    yPos = isSmallDisplay ? 90 : 130;
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
    
    // Very small values (close to 0) are treated as 0
    if (absDiff < 0.05) {
      // Special display for values close to 0
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, GREEN);
      M5.Display.setTextColor(GREEN);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:0.0s");
      } else {
        M5.Display.printf("TIME DIFF: 0.0 s");
      }
    } else if (time_difference > 0) {
      // Relativistic time is slower than normal time (expected behavior)
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, YELLOW);
      M5.Display.setTextColor(YELLOW);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:-%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: -%.1f s", absDiff);
      }
    } else {
      // Relativistic time is faster than normal time (unexpected behavior)
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
  }
}

// Initialize IMU
void initializeImu() {
  // Initialize the IMU
  if (M5.Imu.begin()) {
    imuInitialized = true;
    Serial.println("IMU initialized successfully");
    
    // Start IMU calibration
    calibrateImu();
  } else {
    Serial.println("Failed to initialize IMU");
    imuInitialized = false;
  }
}

// Calibrate IMU
void calibrateImu() {
  if (!imuInitialized) return;
  
  // Check if we have saved calibration data
  if (preferences.isKey("imu_cal_x") && 
      preferences.isKey("imu_cal_y") && 
      preferences.isKey("imu_cal_z")) {
    
    // Load calibration data from persistent storage
    imuAccelOffset[0] = preferences.getFloat("imu_cal_x", 0.0);
    imuAccelOffset[1] = preferences.getFloat("imu_cal_y", 0.0);
    imuAccelOffset[2] = preferences.getFloat("imu_cal_z", 0.0);
    
    Serial.printf("Loaded IMU calibration: X=%.4f, Y=%.4f, Z=%.4f\n", 
                 imuAccelOffset[0], imuAccelOffset[1], imuAccelOffset[2]);
    
    // Display message about loaded calibration
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    int messageY = isSmallDisplay ? (displayHeight - 20) : (displayHeight - 30);
    M5.Display.fillRect(0, messageY, displayWidth, 20, DARKGREEN);
    M5.Display.setTextColor(WHITE, DARKGREEN);
    M5.Display.setCursor(5, messageY + 5);
    M5.Display.print("IMU Calibration Loaded");
    delay(1000);
    M5.Display.fillRect(0, messageY, displayWidth, 20, BLACK);
    
    imuCalibrated = true;
    return;
  }
  
  Serial.println("Starting IMU calibration...");
  Serial.println("Keep the device still for 5 seconds");
  
  // Display calibration start message
  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
  
  // Display calibration message at the bottom of the screen
  int messageY = isSmallDisplay ? (displayHeight - 20) : (displayHeight - 30);
  M5.Display.fillRect(0, messageY, displayWidth, 20, BLUE);
  M5.Display.setTextColor(WHITE, BLUE);
  M5.Display.setCursor(5, messageY + 5);
  M5.Display.print("IMU Calibrating...");
  
  // Variables for offset calculation
  float accelSumX = 0.0f, accelSumY = 0.0f, accelSumZ = 0.0f;
  int sampleCount = 0;
  
  // Collect acceleration data during calibration time
  unsigned long startTime = millis();
  while (millis() - startTime < IMU_CALIBRATION_TIME) {
    float accelX, accelY, accelZ;
    if (M5.Imu.getAccel(&accelX, &accelY, &accelZ)) {
      accelSumX += accelX;
      accelSumY += accelY;
      accelSumZ += accelZ;
      sampleCount++;
    }
    delay(10);
    
    // Show progress bar
    int progress = ((millis() - startTime) * 100) / IMU_CALIBRATION_TIME;
    int barWidth = (displayWidth - 20) * progress / 100;
    M5.Display.fillRect(10, messageY + 15, displayWidth - 20, 3, DARKGREY);
    M5.Display.fillRect(10, messageY + 15, barWidth, 3, GREEN);
  }
  
  // Calculate average values and set as offset
  if (sampleCount > 0) {
    imuAccelOffset[0] = accelSumX / sampleCount;
    imuAccelOffset[1] = accelSumY / sampleCount;
    imuAccelOffset[2] = accelSumZ / sampleCount;
    
    // Adjust for gravity acceleration (Z-axis has ~1G of gravity when stationary)
    imuAccelOffset[2] -= 1.0f;
    
    // Save calibration data to persistent storage
    preferences.putFloat("imu_cal_x", imuAccelOffset[0]);
    preferences.putFloat("imu_cal_y", imuAccelOffset[1]);
    preferences.putFloat("imu_cal_z", imuAccelOffset[2]);
    
    imuCalibrated = true;
    Serial.printf("IMU calibration complete. Offsets: X=%.3f Y=%.3f Z=%.3f\n", 
                 imuAccelOffset[0], imuAccelOffset[1], imuAccelOffset[2]);
    
    // Display success message
    M5.Display.fillRect(0, messageY, displayWidth, 20, DARKGREEN);
    M5.Display.setTextColor(WHITE, DARKGREEN);
    M5.Display.setCursor(5, messageY + 5);
    M5.Display.print("IMU Calibration Saved");
    delay(1000);
  } else {
    Serial.println("IMU calibration failed - no samples collected");
    
    // Display error message
    M5.Display.fillRect(0, messageY, displayWidth, 20, RED);
    M5.Display.setTextColor(WHITE, RED);
    M5.Display.setCursor(5, messageY + 5);
    M5.Display.print("IMU Calibration Failed");
    delay(1000);
  }
  
  // Clear calibration message after completion
  M5.Display.fillRect(0, messageY, displayWidth, 20, BLACK);
}

// Update IMU data
void updateImuData() {
  if (!imuInitialized || !imuCalibrated) return;
  
  float accelX, accelY, accelZ;
  if (M5.Imu.getAccel(&accelX, &accelY, &accelZ)) {
    // Save current acceleration values (for display)
    currentAccelX = accelX;
    currentAccelY = accelY;
    currentAccelZ = accelZ;
    
    // Apply offsets
    accelX -= imuAccelOffset[0];
    accelY -= imuAccelOffset[1];
    accelZ -= imuAccelOffset[2];
    
    // Apply low-pass filter to reduce noise
    filteredAccel[0] = filteredAccel[0] * 0.8f + accelX * 0.2f;
    filteredAccel[1] = filteredAccel[1] * 0.8f + accelY * 0.2f;
    filteredAccel[2] = filteredAccel[2] * 0.8f + accelZ * 0.2f;
    
    // Estimate velocity from acceleration
    unsigned long currentTime = millis();
    if (lastImuUpdate > 0) {
      float deltaT = (currentTime - lastImuUpdate) / 1000.0f; // in seconds
      
      // Check if device is at rest (very low acceleration)
      float accelMagnitude = sqrt(
        filteredAccel[0] * filteredAccel[0] + 
        filteredAccel[1] * filteredAccel[1] + 
        filteredAccel[2] * filteredAccel[2]
      );
      
      // Detect rest state
      if (accelMagnitude < IMU_REST_THRESHOLD) {
        if (!isAtRest) {
          restStartTime = currentTime;
          isAtRest = true;
        } else if (currentTime - restStartTime > IMU_REST_DURATION) {
          // Device has been at rest for sufficient time, reset velocity drift
          imuVelocity[0] *= 0.5f;
          imuVelocity[1] *= 0.5f;
          imuVelocity[2] *= 0.5f;
          
          // If velocity is very small, reset it completely
          if (abs(imuVelocity[0]) < 0.1f) imuVelocity[0] = 0;
          if (abs(imuVelocity[1]) < 0.1f) imuVelocity[1] = 0;
          if (abs(imuVelocity[2]) < 0.1f) imuVelocity[2] = 0;
        }
      } else {
        isAtRest = false;
      }
      
      // Calculate velocity from acceleration (integration)
      // Ignore acceleration below threshold as noise
      if (abs(filteredAccel[0]) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[0] += filteredAccel[0] * deltaT * 9.81f; // Convert to m/s^2
      } else {
        // Apply drift correction when acceleration is low
        imuVelocity[0] *= (1.0f - IMU_DRIFT_CORRECTION);
      }
      
      if (abs(filteredAccel[1]) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[1] += filteredAccel[1] * deltaT * 9.81f;
      } else {
        imuVelocity[1] *= (1.0f - IMU_DRIFT_CORRECTION);
      }
      
      if (abs(filteredAccel[2]) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[2] += filteredAccel[2] * deltaT * 9.81f;
      } else {
        imuVelocity[2] *= (1.0f - IMU_DRIFT_CORRECTION);
      }
      
      // Apply decay to velocity (simulate friction and air resistance)
      imuVelocity[0] *= IMU_SPEED_DECAY;
      imuVelocity[1] *= IMU_SPEED_DECAY;
      imuVelocity[2] *= IMU_SPEED_DECAY;
      
      // Calculate magnitude of 3D velocity vector
      imuSpeed = sqrt(imuVelocity[0]*imuVelocity[0] + 
                      imuVelocity[1]*imuVelocity[1] + 
                      imuVelocity[2]*imuVelocity[2]);
      
      // Convert from m/s to km/h
      imuSpeed *= 3.6f;
      
      // Apply GPS calibration if available
      if (hasValidGpsSpeed && (currentTime - lastGpsDataTime < GPS_VALID_TIMEOUT)) {
        // Calculate the difference between GPS and IMU speeds
        float speedDiff = lastValidGpsSpeed - imuSpeed;
        
        // If the difference is significant, adjust IMU velocity components proportionally
        if (abs(speedDiff) > 1.0f) {  // More than 1 km/h difference
          float adjustmentFactor = 1.0f + (speedDiff / imuSpeed) * GPS_IMU_FUSION_WEIGHT;
          
          // Prevent negative or extreme adjustments
          if (adjustmentFactor < 0.5f) adjustmentFactor = 0.5f;
          if (adjustmentFactor > 2.0f) adjustmentFactor = 2.0f;
          
          // Adjust velocity components
          imuVelocity[0] *= adjustmentFactor;
          imuVelocity[1] *= adjustmentFactor;
          imuVelocity[2] *= adjustmentFactor;
          
          // Recalculate speed after adjustment
          imuSpeed = sqrt(imuVelocity[0]*imuVelocity[0] + 
                          imuVelocity[1]*imuVelocity[1] + 
                          imuVelocity[2]*imuVelocity[2]) * 3.6f;
        }
      }
    }
    
    lastImuUpdate = currentTime;
  }
  
  // Get gyroscope data (for display)
  float gyroX, gyroY, gyroZ;
  if (M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ)) {
    currentGyroX = gyroX;
    currentGyroY = gyroY;
    currentGyroZ = gyroZ;
  }
}

// Update speed based on GPS or IMU
void updateSpeed() {
  unsigned long currentTime = millis();
  bool gpsValid = gps.speed.isValid() && (currentTime - lastGpsDataTime < GPS_VALID_TIMEOUT);
  
  // Update GPS speed reference if valid
  if (gpsValid) {
    float gpsSpeed = gps.speed.kmph();
    lastValidGpsSpeed = gpsSpeed;
    hasValidGpsSpeed = true;
    
    // Save valid GPS speed to persistent storage
    preferences.putFloat("last_speed", gpsSpeed);
    
    // Calculate fused speed with weighted average
    if (imuInitialized && imuCalibrated) {
      // Gradually transition from GPS to IMU or vice versa
      gpsImuFusedSpeed = gpsSpeed * GPS_IMU_FUSION_WEIGHT + imuSpeed * (1.0f - GPS_IMU_FUSION_WEIGHT);
      
      // Use fused speed
      current_speed = gpsImuFusedSpeed;
      usingImuForSpeed = false;
    } else {
      // If IMU not available, use GPS directly
      current_speed = gpsSpeed;
      usingImuForSpeed = false;
    }
  } 
  // Use IMU if GPS is unavailable but we have IMU
  else if (USE_IMU_WHEN_GPS_LOST && imuInitialized && imuCalibrated) {
    current_speed = imuSpeed;
    usingImuForSpeed = true;
  } 
  // If neither is available
  else {
    // Maintain speed or gradually decay it
    current_speed *= 0.95; // Gradual decay
    usingImuForSpeed = false;
  }
  
  // Debug output
  if (currentTime % 1000 < 50) {  // Approximately once per second
    Serial.print("Speed: ");
    Serial.print(current_speed);
    if (usingImuForSpeed) {
      Serial.println(" km/h (IMU)");
    } else if (gpsValid) {
      Serial.print(" km/h (GPS");
      if (imuInitialized && imuCalibrated) {
        Serial.println("+IMU fusion)");
      } else {
        Serial.println(")");
      }
    } else {
      Serial.println(" km/h (estimated)");
    }
  }
}

// Update distance calculation
void updateDistance() {
  unsigned long currentTime = millis();
  
  // Calculate time elapsed since last update in hours (for distance = speed * time)
  float deltaTimeHours = (float)(currentTime - last_distance_update) / 3600000.0f;
  
  // Only update if we have a reasonable time delta
  if (deltaTimeHours > 0 && deltaTimeHours < 0.1) {  // Less than 6 minutes
    // Calculate distance traveled in this interval (km)
    float distance_increment = current_speed * deltaTimeHours;
    
    // Only add if it's a reasonable value (not too large, which could indicate an error)
    if (distance_increment >= 0 && distance_increment < 10.0) {
      total_distance += distance_increment;
      
      // Save total distance to persistent storage every 100 meters or 5 minutes
      static unsigned long last_save_time = 0;
      
      bool shouldSave = false;
      
      // Save if we've traveled more than 100 meters since last save
      if (total_distance >= 0.1) {
        shouldSave = true;
      }
      
      // Or save if it's been more than 5 minutes since last save
      if (currentTime - last_save_time >= 300000) {
        shouldSave = true;
      }
      
      if (shouldSave) {
        preferences.putDouble("total_dist", total_distance);
        last_save_time = currentTime;
        
        // Log to serial occasionally
        if (currentTime % 60000 < 100) {  // Log approximately every minute
          Serial.printf("Total distance saved: %.2f km\n", total_distance);
        }
      }
    }
  }
  
  last_distance_update = currentTime;
}

// Save data to persistent storage
void saveDataToPersistentStorage() {
  // Save total distance
  preferences.putDouble("total_dist", total_distance);
  
  // Save last valid GPS speed
  if (hasValidGpsSpeed) {
    preferences.putFloat("last_speed", lastValidGpsSpeed);
  }
  
  // Save IMU calibration data (if calibrated)
  if (imuCalibrated) {
    preferences.putFloat("imu_cal_x", imuAccelOffset[0]);
    preferences.putFloat("imu_cal_y", imuAccelOffset[1]);
    preferences.putFloat("imu_cal_z", imuAccelOffset[2]);
  }
  
  Serial.println("Data saved to persistent storage");
}

// Load saved data from persistent storage
void loadSavedData() {
  // Load total distance
  total_distance = preferences.getDouble("total_dist", 0.0);
  
  // Load last valid GPS speed
  lastValidGpsSpeed = preferences.getFloat("last_speed", 0.0);
  hasValidGpsSpeed = preferences.isKey("last_speed");
  
  // Load last GPS position (for display purposes)
  bool hasLastPosition = preferences.isKey("last_lat") && 
                         preferences.isKey("last_lng") && 
                         preferences.isKey("last_gps_time");
  
  if (hasLastPosition) {
    double lastLat = preferences.getDouble("last_lat", 0.0);
    double lastLng = preferences.getDouble("last_lng", 0.0);
    float lastAlt = preferences.getFloat("last_alt", 0.0);
    unsigned long lastTime = preferences.getUInt("last_gps_time", 0);
    
    // Calculate how old the data is
    unsigned long dataAge = millis() - lastTime;
    
    // Only log if the data isn't too old (device just started)
    if (dataAge < 3600000) { // 1 hour
      Serial.printf("Loaded last GPS position: %.6f, %.6f, Alt: %.1f m (%.1f minutes ago)\n", 
                   lastLat, lastLng, lastAlt, dataAge / 60000.0);
    }
  }
  
  // IMU calibration data is loaded in calibrateImu()
  
  Serial.printf("Loaded from persistent storage: Distance=%.2f km, Last Speed=%.1f km/h\n", 
               total_distance, lastValidGpsSpeed);
}

// Initialize SD card
void initSDCard() {
  Serial.println("Initializing SD card...");
  
  // Initialize SD card with explicit SPI settings
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    sdCardInitialized = false;
  } else {
    Serial.println("SD card initialized successfully.");
    sdCardInitialized = true;
    
    // Get SD card type and size
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      sdCardInitialized = false;
      return;
    }
    
    // Print card type
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }
    
    // Print card size
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    
    // Create log file with headers if it doesn't exist
    if (!SD.exists(SD_LOG_FILENAME)) {
      logFile = SD.open(SD_LOG_FILENAME, FILE_WRITE);
      if (logFile) {
        logFile.println("timestamp,device_time,relativistic_time,time_difference,speed,latitude,longitude");
        logFile.close();
        Serial.println("Created new log file with headers");
      } else {
        Serial.println("Failed to create log file");
      }
    } else {
      // Add a separator line to indicate a new session
      logFile = SD.open(SD_LOG_FILENAME, FILE_APPEND);
      if (logFile) {
        logFile.println("--- NEW SESSION ---");
        logFile.close();
        Serial.println("Added session separator to log file");
      } else {
        Serial.println("Failed to open log file for writing");
      }
    }
  }
}

// Log data to SD card
void logDataToSD() {
  if (!sdCardInitialized) return;
  
  // Open log file for appending
  logFile = SD.open(SD_LOG_FILENAME, FILE_APPEND);
  if (logFile) {
    // Format: timestamp,device_time,relativistic_time,time_difference,speed,latitude,longitude
    logFile.print(millis());
    logFile.print(",");
    logFile.print(elapsed_device_time);
    logFile.print(",");
    logFile.print(elapsed_relativistic_time);
    logFile.print(",");
    logFile.print(time_difference);
    logFile.print(",");
    logFile.print(current_speed);
    logFile.print(",");
    
    // Add GPS data if available
    if (gpsDataReceived && gps.location.isValid()) {
      logFile.print(gps.location.lat(), 6);
      logFile.print(",");
      logFile.println(gps.location.lng(), 6);
    } else {
      logFile.println(",,");  // Empty fields for lat/lng
    }
    
    logFile.close();
    Serial.println("Data logged to SD card");
  } else {
    Serial.println("Failed to open log file for writing");
  }
}