#include <M5Unified.h>    // Unified library for M5Stack devices
#include <TinyGPS++.h>    // GPS parsing library
#include <SPI.h>          // SPI library

// Constants
const double ORIGINAL_LIGHT_SPEED = 299792.458;  // Actual speed of light (km/s)
const double MODIFIED_LIGHT_SPEED = 0.3;       // Modified speed of light (km/s)

// GPS connection pin settings
#define GPS_RX_PIN 1      // ATOMS3R GPIO1 for GPS RX
#define GPS_TX_PIN 2      // ATOMS3R GPIO2 for GPS TX

// IMU関連の定数
#define USE_IMU_WHEN_GPS_LOST true  // GPSが捕捉できない場合にIMUを使用するかどうか
#define IMU_ACCEL_THRESHOLD 0.1f    // 加速度閾値（これ以上の加速度を検出した場合に移動と判断）
#define IMU_CALIBRATION_TIME 5000   // IMUキャリブレーション時間（ミリ秒）
#define IMU_SPEED_DECAY 0.95f       // 速度減衰係数（摩擦や空気抵抗をシミュレート）

// GPS関連の定数
#define GPS_RX_PIN 1      // ATOMS3R GPIO1 for GPS RX
#define GPS_TX_PIN 2      // ATOMS3R GPIO2 for GPS TX

// TinyGPS++オブジェクト
TinyGPSPlus gps;

// ソフトウェアシリアル
HardwareSerial GPSSerial(1);  // UART1を使用

// グローバル変数
double current_speed = 0.0;  // 現在の速度 (km/h)
double time_dilation = 1.0;  // 時間膨張率
double elapsed_device_time = 0.0;  // 経過したデバイス時間（秒）
double elapsed_relativistic_time = 0.0;  // 経過した相対論的時間（秒）
double time_difference = 0.0;  // 時間差（秒）
unsigned long last_update = 0;  // 最後の更新時間
bool gpsDataReceived = false;  // GPS信号を受信したかどうか
unsigned long lastGpsDataTime = 0;  // 最後にGPSデータを受信した時間
bool show_raw_gps = false;  // 生のGPSデータを表示するかどうか

// IMU関連の変数
bool imuInitialized = false;       // IMUが初期化されたかどうか
bool imuCalibrated = false;        // IMUがキャリブレーションされたかどうか
bool usingImuForSpeed = false;     // IMUを速度計測に使用しているかどうか
float imuAccelOffset[3] = {0, 0, 0}; // 加速度オフセット値
float imuVelocity[3] = {0, 0, 0};  // IMUから推定した速度ベクトル
float imuSpeed = 0.0f;             // IMUから推定した速度スカラー値
unsigned long lastImuUpdate = 0;   // 最後のIMU更新時間

void setup() {
  // Initialize M5 device
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Set up serial communication
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
      M5.Display.println("0.3");
      
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
      M5.Display.println("0.3 km/s");
      
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
  
  // Initialize IMU
  initializeImu();
  
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
  
  // Update IMU data if initialized
  if (imuInitialized) {
    updateImuData();
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
    
    // Update display
    updateDisplay();
    
    last_update = current_millis;
  }
  
  // Reset with button pressed - using M5Unified's BtnA
  if (M5.BtnA.wasPressed()) {
    // Toggle between normal display and raw GPS data display
    show_raw_gps = !show_raw_gps;
    
    // Visual feedback for button press
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // Display button press notification
    M5.Display.fillRect(0, isSmallDisplay ? 40 : 60, displayWidth, 10, BLUE);
    M5.Display.setTextColor(WHITE, BLUE);
    M5.Display.setCursor(5, isSmallDisplay ? 40 : 60);
    if (show_raw_gps) {
      M5.Display.print("GPS RAW DATA: ON");
      Serial.println("Button A pressed - GPS raw data display ON");
    } else {
      M5.Display.print("GPS RAW DATA: OFF");
      Serial.println("Button A pressed - GPS raw data display OFF");
    }
    
    // Clear notification after 500ms (shorter delay)
    delay(500);
    
    // Clear screen completely
    M5.Display.fillScreen(BLACK);
    
    // Redraw title and header
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
    
    // Reset static variables to force redraw
    resetDisplayCache();
    
    // Force immediate complete redraw
    forceCompleteRedraw();
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
  static bool line_drawn = false;
  static bool reset_shown = false;
  
  int current_gps_status = 0;
  if (!gpsDataReceived) current_gps_status = 0;  // Not Connected
  else if (millis() - lastGpsDataTime > 5000) current_gps_status = 1;  // No Signal
  else if (gps.location.isValid()) current_gps_status = 3;  // Connected
  else current_gps_status = 2;  // Acquiring

  // If showing raw GPS data, display that instead of the normal UI
  if (show_raw_gps) {
    // Clear the main content area (preserve the title bar)
    int startY = isSmallDisplay ? 21 : 31;
    M5.Display.fillRect(0, startY, displayWidth, displayHeight - startY, BLACK);
    
    // Display GPS raw data
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(2, startY + 2);
    M5.Display.print("GPS RAW DATA");
    
    M5.Display.setTextColor(WHITE);
    int y = startY + 12;
    
    // GPS Status - more compact for small screens
    M5.Display.setCursor(2, y);
    M5.Display.print("Status:");
    switch(current_gps_status) {
      case 0: M5.Display.setTextColor(RED); M5.Display.print("NO CONN"); break;
      case 1: M5.Display.setTextColor(YELLOW); M5.Display.print("NO SIG"); break;
      case 2: M5.Display.setTextColor(BLUE); M5.Display.print("ACQUIR"); break;
      case 3: M5.Display.setTextColor(GREEN); M5.Display.print("CONN"); break;
    }
    M5.Display.setTextColor(WHITE);
    y += 8; // Reduced spacing
    
    // Satellites
    M5.Display.setCursor(2, y);
    M5.Display.print("Sats:");
    if (gps.satellites.isValid()) {
      M5.Display.print(gps.satellites.value());
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    // Location - more compact
    M5.Display.setCursor(2, y);
    M5.Display.print("Lat:");
    if (gps.location.isValid()) {
      M5.Display.print(gps.location.lat(), 6);
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    M5.Display.setCursor(2, y);
    M5.Display.print("Lng:");
    if (gps.location.isValid()) {
      M5.Display.print(gps.location.lng(), 6);
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    // Altitude
    M5.Display.setCursor(2, y);
    M5.Display.print("Alt:");
    if (gps.altitude.isValid()) {
      M5.Display.print(gps.altitude.meters(), 1);
      M5.Display.print("m");
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    // Speed
    M5.Display.setCursor(2, y);
    M5.Display.print("Spd:");
    if (gps.speed.isValid()) {
      M5.Display.print(gps.speed.kmph(), 1);
      M5.Display.print("km/h");
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    // Course
    M5.Display.setCursor(2, y);
    M5.Display.print("Crs:");
    if (gps.course.isValid()) {
      M5.Display.print(gps.course.deg(), 1);
      M5.Display.print("°");
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
    // Date & Time - combined to save space
    M5.Display.setCursor(2, y);
    M5.Display.print("Date/Time:");
    y += 8; // Reduced spacing
    
    M5.Display.setCursor(2, y);
    if (gps.date.isValid() && gps.time.isValid()) {
      char dateTimeStr[20];
      sprintf(dateTimeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
              gps.date.year(), gps.date.month(), gps.date.day(),
              gps.time.hour(), gps.time.minute(), gps.time.second());
      M5.Display.print(dateTimeStr);
    } else {
      M5.Display.print("--");
    }
    y += 8; // Reduced spacing
    
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
      
      // IMUを使用している場合は表示
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
      
      // IMUを使用している場合は表示
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
  if (!line_drawn) {
    int yPos = isSmallDisplay ? 68 : 100;
    M5.Display.drawLine(0, yPos, displayWidth, yPos, CYAN);
    line_drawn = true;
  }
  
  // Only update device time if changed significantly
  if (abs(prev_device_time - elapsed_device_time) > 0.1) {
    int yPos = isSmallDisplay ? 70 : 105;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEV:");
      M5.Display.setTextColor(WHITE);
      
      // 時:分:秒形式で表示
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
      
      // 時:分:秒形式で表示
      char timeStr[12];
      formatTimeHMS(elapsed_device_time, timeStr);
      M5.Display.print(timeStr);
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
      
      // 時:分:秒形式で表示
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
      
      // 時:分:秒形式で表示
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
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
    
    // 非常に小さい値（0に近い値）の場合は0として扱う
    if (absDiff < 0.05) {
      // ゼロに近い値の場合は特別な表示
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, GREEN);
      M5.Display.setTextColor(GREEN);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:0.0s");
      } else {
        M5.Display.printf("TIME DIFF: 0.0 s");
      }
    } else if (time_difference > 0) {
      // 相対論的時間が通常時間より遅れている（期待される動作）
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, YELLOW);
      M5.Display.setTextColor(YELLOW);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:-%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: -%.1f s", absDiff);
      }
    } else {
      // 相対論的時間が通常時間より進んでいる（通常は発生しないはず）
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
    
    // リセットボタンの表示エリアを描画
    /*
    int btnY = isSmallDisplay ? 110 : (displayHeight - 20);
    if (btnY < (isSmallDisplay ? 110 : 150)) btnY = isSmallDisplay ? 110 : 150; // Ensure button is visible
    
    M5.Display.fillRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, NAVY);
    M5.Display.drawRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, CYAN);
    */
    
    prev_difference = time_difference;
  }
  
  // Always show reset button with nice styling
  if (!reset_shown) {
    int btnY = isSmallDisplay ? 110 : (displayHeight - 20);
    if (btnY < (isSmallDisplay ? 110 : 150)) btnY = isSmallDisplay ? 110 : 150; // Ensure button is visible
    
    //M5.Display.fillRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, NAVY);
    //M5.Display.drawRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, CYAN);
    
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

// Reset display cache to force redraw
void resetDisplayCache() {
  // より安全な方法で画面を完全に再描画するための関数
  
  // 静的変数を直接リセットすることはできないため、
  // 画面を一度クリアして、updateDisplay関数内で
  // 値が変更されたと判断されるようにする
  
  // 画面をクリア
  M5.Display.fillScreen(BLACK);
  
  // 次回のupdateDisplay呼び出しで全ての要素が再描画されるように
  // グローバル変数を少し変更する
  current_speed += 0.2;  // 速度を少し変更して再描画を強制
  time_dilation += 0.00002;  // 時間膨張を少し変更
  
  // 時間の値を変更する際に、差が小さくなりすぎないように調整
  double old_device_time = elapsed_device_time;
  double old_relativistic_time = elapsed_relativistic_time;
  
  elapsed_device_time += 0.2;  // 経過時間を少し変更
  elapsed_relativistic_time = old_relativistic_time + 
                             (elapsed_device_time - old_device_time) * time_dilation;  // 相対論的時間も適切に変更
  
  // 時間差を再計算（非常に小さな値になるのを防ぐ）
  time_difference = elapsed_device_time - elapsed_relativistic_time;
  
  // GPSステータスも更新されるように、最終GPS受信時間を変更
  lastGpsDataTime = millis() - 1000;
}

// 時間表示のヘルパー関数
void formatTimeHMS(float seconds, char* buffer) {
  int totalSeconds = (int)seconds;
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int secs = totalSeconds % 60;
  int millisecs = (int)((seconds - totalSeconds) * 10); // 小数点以下1桁まで
  
  sprintf(buffer, "%02d:%02d:%02d.%01d", hours, minutes, secs, millisecs);
}

// 全ての要素を強制的に再描画する
void forceCompleteRedraw() {
  // 通常表示モードの場合のみ実行
  if (!show_raw_gps) {
    int displayWidth = M5.Display.width();
    int displayHeight = M5.Display.height();
    bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
    
    // タイトルバーを再描画
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
    
    // 光速情報を再描画
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
      M5.Display.println("0.3");
      
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
      M5.Display.println("0.3 km/s");
      
      // Draw another line
      M5.Display.drawLine(0, 55, displayWidth, 55, CYAN);
    }
    
    // GPS状態を強制的に再描画
    int current_gps_status = 0;
    if (!gpsDataReceived) current_gps_status = 0;  // Not Connected
    else if (millis() - lastGpsDataTime > 5000) current_gps_status = 1;  // No Signal
    else if (gps.location.isValid()) current_gps_status = 3;  // Connected
    else current_gps_status = 2;  // Acquiring
    
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
    
    // 速度情報を強制的に再描画
    int yPos = isSmallDisplay ? 40 : 60;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 16, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("SPD");
      
      // IMUを使用している場合は表示
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
      
      // IMUを使用している場合は表示
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
    
    // 時間膨張を強制的に再描画
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
    
    // セパレータラインを描画
    int lineYPos = isSmallDisplay ? 68 : 100;
    M5.Display.drawLine(0, lineYPos, displayWidth, lineYPos, CYAN);
    
    // 時間表示を強制的に再描画
    yPos = isSmallDisplay ? 70 : 105;
    
    if (isSmallDisplay) {
      // Compact layout for small displays
      M5.Display.fillRect(2, yPos, displayWidth-4, 8, BLACK);
      M5.Display.setCursor(2, yPos);
      M5.Display.setTextColor(MAGENTA);
      M5.Display.print("DEV:");
      M5.Display.setTextColor(WHITE);
      
      // 時:分:秒形式で表示
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
      
      // 時:分:秒形式で表示
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
      
      // 時:分:秒形式で表示
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
      
      // 時:分:秒形式で表示
      char timeStr[12];
      formatTimeHMS(elapsed_relativistic_time, timeStr);
      M5.Display.print(timeStr);
    }
    
    // 時間差を強制的に再描画
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
    
    // 非常に小さい値（0に近い値）の場合は0として扱う
    if (absDiff < 0.05) {
      // ゼロに近い値の場合は特別な表示
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, GREEN);
      M5.Display.setTextColor(GREEN);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:0.0s");
      } else {
        M5.Display.printf("TIME DIFF: 0.0 s");
      }
    } else if (time_difference > 0) {
      // 相対論的時間が通常時間より遅れている（期待される動作）
      M5.Display.drawRect(boxX, boxY, boxWidth, boxHeight, YELLOW);
      M5.Display.setTextColor(YELLOW);
      M5.Display.setCursor(boxX + 3, boxY+3);
      if (isSmallDisplay) {
        M5.Display.printf("DIFF:-%.1fs", absDiff);
      } else {
        M5.Display.printf("TIME DIFF: -%.1f s", absDiff);
      }
    } else {
      // 相対論的時間が通常時間より進んでいる（通常は発生しないはず）
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
    
    // リセットボタンの表示エリアを描画
    /*
    int btnY = isSmallDisplay ? 110 : (displayHeight - 20);
    if (btnY < (isSmallDisplay ? 110 : 150)) btnY = isSmallDisplay ? 110 : 150; // Ensure button is visible
    
    M5.Display.fillRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, NAVY);
    M5.Display.drawRect(0, btnY, displayWidth, isSmallDisplay ? 15 : 20, CYAN);
    */
  }
}

// Initialize IMU
void initializeImu() {
  // IMUの初期化
  if (M5.Imu.begin()) {
    imuInitialized = true;
    Serial.println("IMU initialized successfully");
    
    // IMUのキャリブレーションを開始
    calibrateImu();
  } else {
    Serial.println("Failed to initialize IMU");
    imuInitialized = false;
  }
}

// IMUのキャリブレーション
void calibrateImu() {
  if (!imuInitialized) return;
  
  Serial.println("Starting IMU calibration...");
  Serial.println("Keep the device still for 5 seconds");
  
  // キャリブレーション開始を表示
  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  bool isSmallDisplay = (displayWidth <= 128 && displayHeight <= 128);
  
  M5.Display.fillRect(0, isSmallDisplay ? 40 : 60, displayWidth, 20, BLUE);
  M5.Display.setTextColor(WHITE, BLUE);
  M5.Display.setCursor(5, isSmallDisplay ? 40 : 60);
  M5.Display.print("IMU Calibrating...");
  
  // オフセット計算用の変数
  float accelSumX = 0.0f, accelSumY = 0.0f, accelSumZ = 0.0f;
  int sampleCount = 0;
  
  // キャリブレーション時間の間、加速度データを収集
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
  }
  
  // 平均値を計算してオフセットとして設定
  if (sampleCount > 0) {
    imuAccelOffset[0] = accelSumX / sampleCount;
    imuAccelOffset[1] = accelSumY / sampleCount;
    imuAccelOffset[2] = accelSumZ / sampleCount;
    
    // 重力加速度（Z軸）は補正する（静止時は約1Gの重力がかかっている）
    imuAccelOffset[2] -= 1.0f;
    
    imuCalibrated = true;
    Serial.printf("IMU calibration complete. Offsets: X=%.3f Y=%.3f Z=%.3f\n", 
                 imuAccelOffset[0], imuAccelOffset[1], imuAccelOffset[2]);
  } else {
    Serial.println("IMU calibration failed - no samples collected");
  }
  
  // キャリブレーション終了を表示
  M5.Display.fillRect(0, isSmallDisplay ? 40 : 60, displayWidth, 20, BLACK);
}

// IMUデータの更新
void updateImuData() {
  if (!imuInitialized || !imuCalibrated) return;
  
  float accelX, accelY, accelZ;
  if (M5.Imu.getAccel(&accelX, &accelY, &accelZ)) {
    // オフセットを適用
    accelX -= imuAccelOffset[0];
    accelY -= imuAccelOffset[1];
    accelZ -= imuAccelOffset[2];
    
    // 加速度から速度を推定
    unsigned long currentTime = millis();
    if (lastImuUpdate > 0) {
      float deltaT = (currentTime - lastImuUpdate) / 1000.0f; // 秒単位
      
      // 加速度から速度を計算（積分）
      // 閾値以下の加速度はノイズとして無視
      if (abs(accelX) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[0] += accelX * deltaT * 9.81f; // 加速度をm/s^2に変換
      }
      if (abs(accelY) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[1] += accelY * deltaT * 9.81f;
      }
      if (abs(accelZ) > IMU_ACCEL_THRESHOLD) {
        imuVelocity[2] += accelZ * deltaT * 9.81f;
      }
      
      // 速度の減衰（摩擦や空気抵抗をシミュレート）
      imuVelocity[0] *= IMU_SPEED_DECAY;
      imuVelocity[1] *= IMU_SPEED_DECAY;
      imuVelocity[2] *= IMU_SPEED_DECAY;
      
      // 3次元速度ベクトルの大きさを計算
      imuSpeed = sqrt(imuVelocity[0]*imuVelocity[0] + 
                      imuVelocity[1]*imuVelocity[1] + 
                      imuVelocity[2]*imuVelocity[2]);
      
      // m/sからkm/hに変換
      imuSpeed *= 3.6f;
    }
    
    lastImuUpdate = currentTime;
  }
}

// GPSまたはIMUに基づいて速度を更新
void updateSpeed() {
  // GPSが有効な場合はGPSの速度を使用
  if (gps.speed.isValid() && (millis() - lastGpsDataTime < 5000)) {
    current_speed = gps.speed.kmph();
    usingImuForSpeed = false;
  } 
  // GPSが無効でIMUが利用可能な場合
  else if (USE_IMU_WHEN_GPS_LOST && imuInitialized && imuCalibrated) {
    current_speed = imuSpeed;
    usingImuForSpeed = true;
  } 
  // どちらも利用できない場合
  else {
    // 速度を維持するか、徐々に減衰させる
    current_speed *= 0.95; // 徐々に減衰
    usingImuForSpeed = false;
  }
}