#include "WiFi.h"
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <U8g2lib.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

const char* ssid = "DormWifi";
const char* password =  "497addfd33";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C rightScreen(U8G2_R2);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C leftScreen(U8G2_R2);

//Your Domain name with URL path or IP address with path
String serverLocation = "http://192.168.0.100:5000";

unsigned long glassesRefresh = 1000;
const unsigned long rateRefresh = 60000;
const unsigned long angleRefresh = 2000;

unsigned long currentTime = 0;
unsigned long lastGlassesUpdate = 0;
unsigned long lastRateUpdate = 0;
unsigned long lastAngleUpdate = 0;

MPU6050 mpu;

uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_lastSent[3] = { -375, -375, -375};          // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
  leftScreen.setI2CAddress(0x3C * 2);
  leftScreen.begin();
  leftScreen.setFont(u8g2_font_profont12_mf);
  leftScreen.clearDisplay();
  rightScreen.setI2CAddress(0x3D * 2);
  rightScreen.begin();
  rightScreen.setFont(u8g2_font_profont12_mf);
  rightScreen.clearDisplay();
  
  Serial.begin(115200);
  Serial.println("Starting Connection Process");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverPath = serverLocation + "/glasses/refreshrate";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String payload = http.getString();

      JSONVar myObject = JSON.parse(payload);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      glassesRefresh = long(myObject["rate"]);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }

  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
}

void loop() {
  currentTime = millis();
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //    Serial.print("ypr\t");
    //    Serial.print(ypr[0] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.print(ypr[1] * 180 / M_PI);
    //    Serial.print("\t");
    //    Serial.println(ypr[2] * 180 / M_PI);

    if ((abs(ypr[0] - ypr_lastSent[0]) > 0.0348 * 2.5 || abs(ypr[2] - ypr_lastSent[2]) > 0.0348 * 2.5 ) && currentTime - lastAngleUpdate > angleRefresh) {
      ypr_lastSent[0] = ypr[0];
      ypr_lastSent[2] = ypr[2];

      WiFiClient client;
      HTTPClient http;

      String serverPath = serverLocation + "/glasses/updateAngle";

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      http.addHeader("Content-Type", "application/json");

      // Send HTTP GET request
      int httpResponseCode = http.POST("{\"yaw\": " + String(ypr[0]) + ", \"roll\": " + String(ypr[2]) + "}");

      http.end();
      lastAngleUpdate = millis();
      return;
    }
  }

  if (currentTime - lastGlassesUpdate > glassesRefresh  ) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;

      String serverPath = serverLocation + "/glasses";

      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String payload = http.getString();

        JSONVar myObject = JSON.parse(payload);

        leftScreen.clearBuffer();
        for (int i = 0; i < myObject["left"]["rect"].length(); i++) {
          leftScreen.drawRBox((int)myObject["left"]["rect"][i][0], (int)myObject["left"]["rect"][i][1], (int)myObject["left"]["rect"][i][2], (int)myObject["left"]["rect"][i][3], 2);
        }
        for (int i = 0; i < myObject["left"]["circ"].length(); i++) {
          leftScreen.drawCircle((int)myObject["left"]["circ"][i][0], (int)myObject["left"]["circ"][i][1], (int)myObject["left"]["circ"][i][2]);
        }
        for (int i = 0; i < myObject["left"]["text"].length(); i++) {
          leftScreen.drawStr((int)myObject["left"]["text"][i][0], (int)myObject["left"]["text"][i][1], (const char*)myObject["left"]["text"][i][2]);
        }
        leftScreen.sendBuffer();

        rightScreen.clearBuffer();
        for (int i = 0; i < myObject["right"]["rect"].length(); i++) {
          rightScreen.drawRBox((int)myObject["right"]["rect"][i][0], (int)myObject["right"]["rect"][i][1], (int)myObject["right"]["rect"][i][2], (int)myObject["right"]["rect"][i][3], 2);
        }
        for (int i = 0; i < myObject["right"]["circ"].length(); i++) {
          rightScreen.drawCircle((int)myObject["right"]["circ"][i][0], (int)myObject["right"]["circ"][i][1], (int)myObject["right"]["circ"][i][2]);
        }
        for (int i = 0; i < myObject["right"]["text"].length(); i++) {
          rightScreen.drawStr((int)myObject["right"]["text"][i][0], (int)myObject["right"]["text"][i][1], (const char*)myObject["right"]["text"][i][2]);
        }
        rightScreen.sendBuffer();

        //PUT DRAWING CODE HERE
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    lastGlassesUpdate = millis();
  }
}
