#include <Arduino.h>

#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Update.h>

#define z_axis_step 32
#define z_axis_dir 33
#define z_axis_home 5
#define drills 21

#define x_axis_step 25
#define x_axis_dir 26
#define x_axis_home 15

#define start_button 4


AccelStepper zAxisStepper(AccelStepper::DRIVER, z_axis_step, z_axis_dir);
AccelStepper xAxisStepper(AccelStepper::DRIVER, x_axis_step, x_axis_dir);

const int waitTime = 100;

// Z axis parameters
int zAxisMoveDownDistance = 5000; // How far the Z axis moves down while drilling 
const int zAxisSpeedDown = 2000; // Speed of Z axis while drilling
const int zAxisSpeedUp = 2000; // Speed of Z axis while moving up
const int zAxisAccelerationDown = 5000; // Acceleration of Z axis while drilling
const int zAxisAccelerationUp = 5000; // Acceleration of Z axis while moving up

// X axis parameters
const int xAxisSpeedDrilling = 2000; // Speed of X axis while drilling
const int xAxisSpeedHoming = 4000; // Speed of X axis while moving
const int xAxisAccelerationDrilling = 8000; // Acceleration of X axis while drilling
const int xAxisAccelerationHoming = 4000; // Acceleration of X axis while moving
int xAxisMoveDistance = -2600; // How far the X axis moves while drilling
// WiFi credentials
const char* ssid = "2400Wireless";
const char* password = "lindafleming";

WebServer server(80);

// Web start flag
volatile bool webStartFlag = false;

// Current firmware version
#define CURRENT_FIRMWARE_VERSION "1.0.1"

// HTML page
String getHTML() {
  String html = "<html><head><title>Drill Control</title></head><body>";
  html += "<h2>Drill Control Panel</h2>";
  html += "<form action='/update' method='POST'>";
  html += "Z Axis Move Down Distance: <input type='number' name='z' value='" + String(zAxisMoveDownDistance) + "'><br>";
  html += "X Axis Move Distance: <input type='number' name='x' value='" + String(xAxisMoveDistance) + "'><br>";
  html += "<input type='submit' value='Update Values'>";
  html += "</form>";
  html += "<form action='/start' method='POST' style='margin-top:20px;'>";
  html += "<input type='submit' value='Start Drilling'>";
  html += "</form>";
  html += "<form action='/ota' method='POST' style='margin-top:20px;'>";
  html += "<input type='submit' value='Update Firmware'>";
  html += "</form>";
  html += "</body></html>";
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
            server.send(200, "text/plain", "Update successful. Rebooting...");
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
        return true;
      }
    }
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

void z_axis_homing() {
  Serial.println("Z homing started");

  zAxisStepper.setMaxSpeed(zAxisSpeedUp);
  zAxisStepper.setAcceleration(zAxisAccelerationUp);
  zAxisStepper.moveTo(-1000000);  // Move down toward limit switch

  while (!isZLimitTriggered()) {
    zAxisStepper.run();
  }

  zAxisStepper.stop();
  zAxisStepper.setCurrentPosition(0);
  Serial.println("Z axis homed");
}

void x_axis_homing() {
  Serial.println("X homing started");

  xAxisStepper.setMaxSpeed(xAxisSpeedHoming);
  xAxisStepper.setAcceleration(xAxisAccelerationHoming);
  xAxisStepper.moveTo(100000);  // Move left toward limit switch

  while (!isXLimitTriggered()) {
    xAxisStepper.run();
  }

  xAxisStepper.stop();
  xAxisStepper.setCurrentPosition(0);
  Serial.println("X axis homed");
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(start_button, INPUT_PULLUP); // Set start button pin as input with pull-up resistor
  pinMode(z_axis_home, INPUT_PULLUP);
  pinMode(x_axis_home, INPUT_PULLUP);
  pinMode(drills, OUTPUT); // Set drill pin as output
  z_axis_homing(); // Call the homing function to return to home position
  x_axis_homing(); // Call the homing function to return to home position

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());


  // Web server routes
  server.on("/", handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/ota", HTTP_POST, handleOTA);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();

  // Check for start button press with debounce or web start
  bool shouldStart = isStartButtonPressed() || webStartFlag;
  if (shouldStart) {
    webStartFlag = false;
    while (rowCounter < 12) {
      Serial.println("Start");
      // Turn on drill
      digitalWrite(drills, HIGH);
      Serial.println("Drills On");
      // Configure Z axis for downward movement
      zAxisStepper.setMaxSpeed(zAxisSpeedDown);
      zAxisStepper.setAcceleration(zAxisAccelerationDown);
      zAxisStepper.moveTo(zAxisMoveDownDistance);
      // Execute downward movement
      while (zAxisStepper.distanceToGo() != 0) {
        zAxisStepper.run();
      }
      // Move Z axis back up a little bit before turning drills off
      const int zAxisRetractDistance = 4000; // Adjust this value as needed
      zAxisStepper.moveTo(zAxisStepper.currentPosition() - zAxisRetractDistance);
      while (zAxisStepper.distanceToGo() != 0) {
        zAxisStepper.run();
      }
      digitalWrite(drills, LOW); // Now turn off drills
      delay(200);
      z_axis_homing(); 
      // Turn off drill
      Serial.println("Drills Off");
      // Configure X axis for drilling movement
      xAxisStepper.setMaxSpeed(xAxisSpeedDrilling);
      xAxisStepper.setAcceleration(xAxisAccelerationDrilling);
      xAxisStepper.moveTo((rowCounter + 1) * xAxisMoveDistance);
      while (xAxisStepper.distanceToGo() != 0) {
        xAxisStepper.run(); // Execute drilling movement
      }
      // Increment row counter
      rowCounter++;
      Serial.print("Row counter: ");
      Serial.println(rowCounter);
      // Optional delay to debounce button
      delay(waitTime);
    }
    Serial.println("All rows drilled, returning to home position.");
    z_axis_homing(); // Call the homing function to return to home position
    const int x_axis_extra_move = -4000; // Move X axis a bit more to ensure it is clear of the drilling area
    xAxisStepper.setMaxSpeed(xAxisSpeedHoming);
    xAxisStepper.setAcceleration(xAxisAccelerationHoming);
    xAxisStepper.moveTo(xAxisStepper.currentPosition() + x_axis_extra_move); // Move X axis further
    while (xAxisStepper.distanceToGo() != 0) {
      xAxisStepper.run();
    }
    xAxisStepper.moveTo(xAxisStepper.currentPosition() + x_axis_extra_move); // Move X axis further
    x_axis_homing(); // Call the homing function to return to home position
    rowCounter = 0; // Reset row counter after all rows are drilled
  }
}