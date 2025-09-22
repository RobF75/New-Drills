#define SDA_PIN 42
#define SCL_PIN 41
#define DRILL_RELAY_BIT 0
#include <Arduino.h>

#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Update.h>
#include <webpage.h>
#include <version.h>

// Pin Definitions 
#define z_axis_step 5
#define z_axis_dir 6
#define z_axis_home 7


#include <Wire.h>
#define TCA9554_ADDRESS 0x38
#define TCA9554_INPUT_REG    0x00
#define TCA9554_OUTPUT_REG   0x01
#define TCA9554_POLARITY_REG 0x02
#define TCA9554_CONFIG_REG   0x03

uint8_t tca9554_address = TCA9554_ADDRESS;

void scanI2C() {
  for (uint8_t i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(i, HEX);
    }
  }
}

uint8_t findTCA9554() {
  for (uint8_t addr = 0x20; addr <= 0x3F; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Wire.beginTransmission(addr);
      Wire.write(TCA9554_INPUT_REG);
      if (Wire.endTransmission() == 0) {
        return addr;
      }
    }
  }
  return 0;
}

uint8_t relayState = 0xFF; // All relays off (active low)

void setRelay(uint8_t relay, bool on) {
  if (on) relayState |= (1 << relay); // 1=on
  else relayState &= ~(1 << relay);  // 0=off
  Wire.beginTransmission(tca9554_address);
  Wire.write(TCA9554_OUTPUT_REG);
  Wire.write(relayState);
  Wire.endTransmission();
}

#define x_axis_step 9
#define x_axis_dir 10
#define x_axis_home 11

#define start_button 4


AccelStepper zAxisStepper(AccelStepper::DRIVER, z_axis_step, z_axis_dir);
AccelStepper xAxisStepper(AccelStepper::DRIVER, x_axis_step, x_axis_dir);

const int waitTime = 100;
String startButtonStatus = "<span style='color:#888;'>Not pressed</span>"; // Initial status of the start button

// Z axis parameters
int zAxisMoveDownDistance = 5000; // How far the Z axis moves down while drilling 
const int zAxisSpeedDown = 2000; // Speed of Z axis while drilling
const int zAxisSpeedUp = 2000; // Speed of Z axis while moving up
const int zAxisAccelerationDown = 5000; // Acceleration of Z axis while drilling
const int zAxisAccelerationUp = 5000; // Acceleration of Z axis while moving up

// // X axis parameters
const int xAxisSpeedDrilling = 2000; // Speed of X axis while drilling
const int xAxisSpeedHoming = 4000; // Speed of X axis while moving
const int xAxisAccelerationDrilling = 2000; // Acceleration of X axis while drilling
const int xAxisAccelerationHoming = 2000; // Acceleration of X axis while moving
int xAxisMoveDistance = -2820; // How far the X axis moves while drilling

// WiFi credentials (try two networks)
const char* ssid1 = "2400Wireless";
const char* password1 = "lindafleming";
const char* ssid2 = "FactreeOfficeV2";
const char* password2 = "Factree3782";

WebServer server(80);

// Web start flag
volatile bool webStartFlag = false;



// HTML page from include/webpage.h and style.h
String getHTML() {
  String html(DRILL_CONTROL_HTML);
  html.replace("%ZAXIS%", String(zAxisMoveDownDistance));
  html.replace("%XAXIS%", String(xAxisMoveDistance));
  html.replace("%STYLE%", String(DRILL_CONTROL_STYLE));
  html.replace("%VERSION%", String(CURRENT_FIRMWARE_VERSION));
  html.replace("%STARTBTN%", startButtonStatus);
  return html;
}
// OTA update handler
void handleOTA() {
  String versionUrl = "https://raw.githubusercontent.com/RobF75/New-Drills/main/version.json";
  HTTPClient http;
  Serial.println("Checking for firmware update...");
  http.begin(versionUrl);
  int httpCode = http.GET();
  Serial.print("HTTP code for version.json: ");
  Serial.println(httpCode);
  if (httpCode == 200) {
    String payload = http.getString();
    Serial.println("version.json: " + payload);
    int vIdx = payload.indexOf("\"version\":");
    int vStart = payload.indexOf('"', vIdx + 10) + 1;
    int vEnd = payload.indexOf('"', vStart);
    String remoteVersion = payload.substring(vStart, vEnd);
    Serial.println("Remote version: " + remoteVersion);
    if (String(CURRENT_FIRMWARE_VERSION) == remoteVersion) {
      server.send(200, "text/plain", "Firmware is up to date.");
      http.end();
      return;
    }
    int urlIdx = payload.indexOf("\"firmware_url\":");
    int urlStart = payload.indexOf('"', urlIdx + 15) + 1;
    int urlEnd = payload.indexOf('"', urlStart);
    String firmwareUrl = payload.substring(urlStart, urlEnd);
    Serial.println("Firmware URL: " + firmwareUrl);
    http.end();
    // Download and update firmware
    WiFiClient client;
    http.begin(firmwareUrl);
    int fwCode = http.GET();
    if (fwCode == 200) {
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
      int contentLength = http.getSize();
      bool canBegin = Update.begin(contentLength);
      if (canBegin) {
        Serial.println("Begin OTA update...");
        size_t written = Update.writeStream(http.getStream());
        if (written == contentLength) {
          Serial.println("Written : " + String(written) + " successfully");
        } else {
          Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
        }
        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("Update successfully completed. Rebooting.");
            String redirectHtml = "<html><head><meta http-equiv='refresh' content='1;url=/'></head><body>Update successful. Rebooting...</body></html>";
            server.send(200, "text/html", redirectHtml);
            delay(1000);
            ESP.restart();
          } else {
            Serial.println("Update not finished. Something went wrong.");
            server.send(200, "text/plain", "Update not finished. Something went wrong.");
          }
        } else {
          Serial.println("Error Occurred. Error #: " + String(Update.getError()));
          server.send(200, "text/plain", "Update failed. Error #: " + String(Update.getError()));
        }
      } else {
        Serial.println("Not enough space to begin OTA");
        server.send(200, "text/plain", "Not enough space to begin OTA");
      }
    } else {
      Serial.println("Could not download firmware");
      server.send(200, "text/plain", "Could not download firmware");
    }
    http.end();
  } else {
    Serial.println("Could not fetch version.json");
    server.send(200, "text/plain", "Could not fetch version.json");
    http.end();
  }
}

void handleRoot() {
  server.send(200, "text/html", getHTML());
}

void handleUpdate() {
  if (server.hasArg("z")) {
    zAxisMoveDownDistance = server.arg("z").toInt();
  }
  if (server.hasArg("x")) {
    xAxisMoveDistance = server.arg("x").toInt();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStart() {
  webStartFlag = true;
  server.sendHeader("Location", "/");
  server.send(303);
}

// Row counter
int rowCounter = 0;

// Button debounce parameters
const unsigned long debounceDelay = 50;    // Debounce time in milliseconds
unsigned long lastDebounceTime = 0;        // Last time the button state changed
int lastButtonState = HIGH;                // Previous reading from the input pin
int buttonState = HIGH;                    // Current stable state of the button

// // Limit switch debounce parameters
// const unsigned long limitSwitchDebounceDelay = 200;  // Debounce time for limit switches
// unsigned long lastZLimitDebounceTime = 0;
// unsigned long lastXLimitDebounceTime = 0;
// int lastZLimitState = HIGH;
// int lastXLimitState = HIGH;
// int zLimitState = HIGH;
// int xLimitState = HIGH;

// Function to check if button is pressed with debounce
bool isStartButtonPressed() {
  // Read the current state of the button
  int reading = digitalRead(start_button);
  static unsigned long lastPressTime = 0;
  // If the button state changed, reset the debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  // Check if enough time has passed since the last change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed, update buttonState
    if (reading != buttonState) {
      buttonState = reading;
      
      // Return true only on a new LOW (pressed) state
      if (buttonState == LOW) {
        lastPressTime = millis();
        startButtonStatus = "<span style='color:green; background-color:yellow;'>Pressed</span>";
        return true;
      }
    }
  }
  // Show "Pressed" for 1 second after press
  if (millis() - lastPressTime > 1000) {
    startButtonStatus = "<span style='color:#888;'>Not pressed</span>";
  }
  // Save the current reading for the next loop
  lastButtonState = reading;
  return false;
}

// Simple check for Z axis limit switch
bool isZLimitTriggered() {
  return digitalRead(z_axis_home) == LOW;
}

// Simple check for X axis limit switch
bool isXLimitTriggered() {
  return digitalRead(x_axis_home) == LOW;
}

// void z_axis_homing() {
//   Serial.println("Z homing started");

//   zAxisStepper.setMaxSpeed(zAxisSpeedUp);
//   zAxisStepper.setAcceleration(zAxisAccelerationUp);
//   zAxisStepper.moveTo(-1000000);  // Move up toward limit switch

//   while (!isZLimitTriggered()) {
//     zAxisStepper.run();
//   }

//   zAxisStepper.stop();
//   zAxisStepper.setCurrentPosition(0);
//   Serial.println("Z axis homed");
// }

// void x_axis_homing() {
//   Serial.println("X homing started");

//   xAxisStepper.setMaxSpeed(xAxisSpeedHoming);
//   xAxisStepper.setAcceleration(xAxisAccelerationHoming);
//   xAxisStepper.moveTo(100000);  // Move toward limit switch

//   while (!isXLimitTriggered()) {
//     xAxisStepper.run();
//   }

//   xAxisStepper.stop();
//   xAxisStepper.setCurrentPosition(0);
//   Serial.println("X axis homed");
// }


void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(start_button, INPUT_PULLUP);

  Serial.println("Initializing I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  Serial.println("Scanning I2C devices...");
  scanI2C();
  tca9554_address = findTCA9554();
  if (tca9554_address == 0) {
    Serial.println("ERROR: TCA9554 not found at any expected address!");
    Serial.println("Please check I2C connections and device address.");
    return;
  }
  Serial.print("TCA9554 found at address: 0x");
  Serial.println(tca9554_address, HEX);

  // Configure TCA9554: all pins as outputs
  Wire.beginTransmission(tca9554_address);
  Wire.write(TCA9554_CONFIG_REG);
  Wire.write(0x00); // All pins as outputs
  uint8_t result1 = Wire.endTransmission();
  if (result1 == 0) {
    Serial.println("TCA9554 configuration successful");
  } else {
    Serial.print("TCA9554 configuration failed with error: ");
    Serial.println(result1);
  }
  // Set all relays OFF
  relayState = 0x00;
  Wire.beginTransmission(tca9554_address);
  Wire.write(TCA9554_OUTPUT_REG);
  Wire.write(relayState); // All relays OFF
  uint8_t result2 = Wire.endTransmission();
  if (result2 == 0) {
    Serial.println("TCA9554 relay initialization successful");
  } else {
    Serial.print("TCA9554 relay initialization failed with error: ");
    Serial.println(result2);
  }

  // WiFi setup (try two networks)
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi (Network 1): ");
  WiFi.begin(ssid1, password1);
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 8000; // 8 seconds for first network
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to Network 1! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.print("Network 1 failed, trying Network 2: ");
    WiFi.begin(ssid2, password2);
    startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("Connected to Network 2! IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println();
      Serial.println("Failed to connect to both WiFi networks!");
    }
  }
  delay(100);
  // Homing after WiFi connection
  // z_axis_homing(); // Call the homing function to return to home position
  // delay(100);
  // x_axis_homing(); // Call the homing function to return to home position

  // Web server routes
  server.on("/", handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/ota", HTTP_POST, handleOTA);
  server.on("/start", HTTP_POST, handleStart); // <-- Add this line
  server.on("/status", []() {
    server.send(200, "text/plain", startButtonStatus);
  });
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("Web server started");
}

// --- Non-blocking drill and homing state machine ---
enum DrillState {
  DRILL_IDLE,
  DRILL_START,
  DRILL_Z_DOWN,
  DRILL_Z_RETRACT,
  DRILL_Z_HOME_START,
  DRILL_Z_HOME_RUN,
  DRILL_X_MOVE_START,
  DRILL_X_MOVE_RUN,
  DRILL_NEXT_ROW,
  DRILL_FINISH_START,
  DRILL_FINISH_X_MOVE,
  DRILL_FINISH_X_HOME,
  DRILL_DONE
};
DrillState drillState = DRILL_IDLE;
int drillRowCounter = 0;
unsigned long drillDelayStart = 0;
const int drillDelayTime = 200; // ms
bool zHomingActive = false;
bool xHomingActive = false;

void startZHoming() {
  zAxisStepper.setMaxSpeed(zAxisSpeedUp);
  zAxisStepper.setAcceleration(zAxisAccelerationUp);
  zAxisStepper.moveTo(-1000000); // Move down toward limit switch
  zHomingActive = true;
}
void runZHoming() {
  if (zHomingActive) {
    zAxisStepper.run();
    if (isZLimitTriggered()) {
      zAxisStepper.stop();
      zAxisStepper.setCurrentPosition(0);
      zHomingActive = false;
    }
  }
}
void startXHoming() {
  xAxisStepper.setMaxSpeed(xAxisSpeedHoming);
  xAxisStepper.setAcceleration(xAxisAccelerationHoming);
  xAxisStepper.moveTo(100000); // Move left toward limit switch
  xHomingActive = true;
}
void runXHoming() {
  if (xHomingActive) {
    xAxisStepper.run();
    if (isXLimitTriggered()) {
      xAxisStepper.stop();
      xAxisStepper.setCurrentPosition(0);
      xHomingActive = false;
    }
  }
}

void loop() {
  server.handleClient();

  // Start drilling if button pressed or web start
  if ((isStartButtonPressed() || webStartFlag) && drillState == DRILL_IDLE) {
    webStartFlag = false;
    drillRowCounter = 0;
    drillState = DRILL_START;
  }

  switch (drillState) {
    case DRILL_IDLE:
      // Do nothing
      break;

    case DRILL_START:
      Serial.println("Start");
  setRelay(DRILL_RELAY_BIT, true);
  Serial.println("Drills On (relay)");
      zAxisStepper.setMaxSpeed(zAxisSpeedDown);
      zAxisStepper.setAcceleration(zAxisAccelerationDown);
      zAxisStepper.moveTo(zAxisMoveDownDistance);
      drillState = DRILL_Z_DOWN;
      break;

    case DRILL_Z_DOWN:
      zAxisStepper.run();
      if (zAxisStepper.distanceToGo() == 0) {
        zAxisStepper.moveTo(zAxisStepper.currentPosition() - 4000); // retract
        drillState = DRILL_Z_RETRACT;
      }
      break;

    case DRILL_Z_RETRACT:
      zAxisStepper.run();
      if (zAxisStepper.distanceToGo() == 0) {
  setRelay(DRILL_RELAY_BIT, false);
  Serial.println("Drills Off (relay)");
        drillDelayStart = millis();
        drillState = DRILL_Z_HOME_START;
      }
      break;

    case DRILL_Z_HOME_START:
      if (millis() - drillDelayStart < drillDelayTime) break; // wait
      startZHoming();
      drillState = DRILL_Z_HOME_RUN;
      break;

    case DRILL_Z_HOME_RUN:
      runZHoming();
      if (!zHomingActive) {
        drillState = DRILL_X_MOVE_START;
      }
      break;

    case DRILL_X_MOVE_START:
      xAxisStepper.setMaxSpeed(xAxisSpeedDrilling);
      xAxisStepper.setAcceleration(xAxisAccelerationDrilling);
      xAxisStepper.moveTo((drillRowCounter + 1) * xAxisMoveDistance);
      drillState = DRILL_X_MOVE_RUN;
      break;

    case DRILL_X_MOVE_RUN:
      xAxisStepper.run();
      if (xAxisStepper.distanceToGo() == 0) {
        drillRowCounter++;
        if (drillRowCounter < 12) {
          drillState = DRILL_START;
        } else {
          drillState = DRILL_FINISH_START;
        }
      }
      break;

    case DRILL_FINISH_START:
      Serial.println("All rows drilled, returning to home position.");
      startZHoming();
      drillState = DRILL_FINISH_X_MOVE;
      break;

    case DRILL_FINISH_X_MOVE:
      runZHoming();
      if (!zHomingActive) {
        xAxisStepper.setMaxSpeed(xAxisSpeedHoming);
        xAxisStepper.setAcceleration(xAxisAccelerationHoming);
        xAxisStepper.moveTo(xAxisStepper.currentPosition() - 4000);
        drillState = DRILL_FINISH_X_HOME;
      }
      break;

    case DRILL_FINISH_X_HOME:
      xAxisStepper.run();
      if (xAxisStepper.distanceToGo() == 0) {
        startXHoming();
        drillState = DRILL_DONE;
      }
      break;

    case DRILL_DONE:
      runXHoming();
      if (!xHomingActive) {
        drillState = DRILL_IDLE;
        drillRowCounter = 0;
      }
      break;
  }
}