#include <M5Unified.h>    // Unified library for M5Stack devices
#include <TinyGPS++.h>    // GPS parsing library
#include <SPI.h>          // SPI library

// Constants
const double ORIGINAL_LIGHT_SPEED = 299792.458;  // Actual speed of light (km/s)
const double MODIFIED_LIGHT_SPEED = 0.3;       // Modified speed of light (km/s)

// GPS connection pin settings
#define GPS_RX_PIN 5  // G5 pin - Receives data from GPS
#define GPS_TX_PIN -1  // TX pin is not connected

// Global variables
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Serial port for GPS
unsigned long lastGpsDataTime = 0;
boolean gpsDataReceived = false;
double current_speed = 0.0;    // Current speed (km/h)
double time_dilation = 0.0;    // Time dilation factor
double elapsed_device_time = 0.0;  // Elapsed device time (seconds)
double elapsed_relativistic_time = 0.0;  // Relativistically elapsed time (seconds)
double time_difference = 0.0;  // Time difference (seconds)
unsigned long last_update = 0;  // Last update time

void setup() {
  // For debugging - initialize Serial first
  Serial.begin(115200);
  delay(500); // Give serial time to start
  Serial.println("\n\n=== Urashima Effect Device starting... ===");
  
  // Initialize M5Unified with auto-detection
  M5.begin();
  Serial.println("M5 initialized");
  
  // Print device information
  Serial.println("Device information:");
  Serial.printf("Display width: %d, height: %d\n", M5.Display.width(), M5.Display.height());
  
  // Initialize display
  // Set initial display content
  if (M5.Display.width() > 0) {  // Check if display is available
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
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
      M5.Display.println("299,792");
      
      M5.Display.setCursor(50, 30);
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("VIRT:");
      M5.Display.setTextColor(WHITE);
      M5.Display.println("300");
      
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
      M5.Display.println("300 km/s");
      
      // Draw another line
      M5.Display.drawLine(0, 55, displayWidth, 55, CYAN);
    }
    
    updateDisplay(); // Initial display update
    Serial.println("Display initialized");
  } else {
    Serial.println("No display detected");
  }
  
  // Initialize GPS
  Serial.println("Initializing GPS");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  Serial.println("Waiting for GPS data...");
  
  last_update = millis();
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
      }
      
      // Log speed when updated
      if (gps.speed.isUpdated()) {
        Serial.printf("GPS Speed: %.2f km/h\n", gps.speed.kmph());
      }
    }
  }
  
  // Update screen periodically
  if (millis() - last_update > 500) {  // Reduce update frequency to 500ms
    unsigned long current_millis = millis();
    double delta_t = (current_millis - last_update) / 1000.0;  // Time elapsed since last update (seconds)
    
    // Update device time
    elapsed_device_time += delta_t;
    
    // Update speed if GPS is valid
    if (gps.speed.isValid()) {
      current_speed = gps.speed.kmph();  // Get speed in km/h
    }
    
    // Calculate relativistic effect
    calculateRelativisticEffect(delta_t);
    
    // Update display
    updateDisplay();
    
    last_update = current_millis;
  }
  
  // Reset with button pressed - using M5Unified's BtnA
  if (M5.BtnA.wasPressed()) {
    resetTimeCalculation();
    
    // Visual feedback for button press
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // Display button press notification
    M5.Display.fillRect(0, isSmallDisplay ? 40 : 60, displayWidth, 10, RED);
    M5.Display.setTextColor(WHITE, RED);
    M5.Display.setCursor(5, isSmallDisplay ? 40 : 60);
    M5.Display.print("BUTTON PRESSED - RESET");
    
    // Log button press
    Serial.println("Button A pressed - Time measurement reset");
    
    // Clear notification after 1 second
    delay(1000);
    updateDisplay();
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
  if (M5.Display.width() <= 0) return;  // Skip if no display available
  
  // Check if we're using a small display
  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
  
  // Use static variables to track previous values and reduce flicker
  static double prev_speed = -1;
  static double prev_dilation = -1;
  static double prev_device_time = -1;
  static double prev_relative_time = -1;
  static double prev_difference = -1;
  static int prev_gps_status = -1;
  
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
      M5.Display.printf("    %.4f km/s", current_speed / 3600.0);
    } else {
      // Standard layout for larger displays
      M5.Display.fillRect(5, yPos, displayWidth-10, 20, BLACK);
      M5.Display.setCursor(5, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPEED: ");
      M5.Display.setTextColor(WHITE);
      M5.Display.printf("%.1f km/h", current_speed);
      M5.Display.setCursor(5, yPos+10);
      M5.Display.printf("       %.5f km/s", current_speed / 3600.0);
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
    String btnText = isSmallDisplay ? "BTN:RESET" : "PRESS BUTTON TO RESET";
    int textWidth = btnText.length() * 6; // Approximate width
    int textX = (displayWidth - textWidth) / 2;
    
    M5.Display.setCursor(textX > 0 ? textX : 0, btnY + (isSmallDisplay ? 4 : 6));
    M5.Display.setTextColor(WHITE);
    M5.Display.print(btnText);
    
    reset_shown = true;
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