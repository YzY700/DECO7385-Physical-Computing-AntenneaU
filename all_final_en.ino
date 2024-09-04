#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <FastLED.h>

/*
The code references the following tutorials and examples:
    Accelerometer‘s gesture detection:
    https://github.com/ldab/lis3dh-motion-detection
    https://github.com/zvonler/CircuitPlaygroundGestures
    
    Servo control:
    https://www.makerguides.com/servo-arduino-tutorial/
    
    LED control:
    https://blog.csdn.net/zbp_12138/article/details/119299637

We also used ChatGPT to help debugging (help find errors and give suggestions), but did't use it to generate code.
*/

#define NUM_LEDS_1 10  // Number of LEDs in the 1st LED strip is 10
#define NUM_LEDS_2 10  // Number of LEDs in the 2nd LED strip is 10
#define DATA_PIN_1 3   // Data pin for the 1st LED strip
#define DATA_PIN_2 4   // Data pin for the 2nd LED strip

#define SOUND_SENSOR A2  // Sound sensor connected to pin A2

#define VIBRATION_MOTOR_PIN 7   // Vibration motor pin

CRGB leds_1[NUM_LEDS_1];  // LED array for the 1st LED strip
CRGB leds_2[NUM_LEDS_2];  // LED array for the 2nd LED strip

Adafruit_LIS3DH lis = Adafruit_LIS3DH();  // Using I2C communication

// First antenna's servo set
Servo myservo1;  // First servo control object
Servo myservo2;  // Second servo control object
Servo myservo5;  // Base servo control object 

// Second antenna's servo set
Servo myservo3;  // First servo control object
Servo myservo4;  // Second servo control object
Servo myservo6;  // Base servo control object

// <------- Accelerometer Setting ------>
const float DIRECTION_THRESHOLD = 10.0;  // Direction change threshold

const float SHAKE_THRESHOLD = 12.0;        // Shake detection threshold
const unsigned int SHAKE_COUNT_THRESHOLD = 3;  // Minimum shake count to trigger shake event
const unsigned long SHAKE_TIME_WINDOW = 800;  // Shake counter time window (ms)

const float Z_UP_THROW_THRESHOLD =  0.5; // Upward acceleration threshold for throw action
const float Z_DOWN_ZERO_THRESHOLD = -9.0; // Threshold for near 0 at the peak of the throw

unsigned int shakeCount = 0;
unsigned long lastShakeTime = 0;
bool shakeDetected = false;

int directionChangeCount = 0;
int accelerationEvents = 0; // Number of accelerometer events
const int accelerationThreshold = 2; // Threshold for number of accelerometer events
unsigned long lastDetectionTime = 0; // Last detection time

int lastPressureValue = 0; // Last pressure sensor value
unsigned int pressureEvents = 0; // Number of pressure events
const int pressureThreshold = 30; 

enum ThrowState { WAITING, THROWING_UP };
ThrowState throwState = WAITING;

// <------- Pressure Sensor Setting ------>
const int pressureSensorPin = A0;


// <------- Servo Motor Setting ------>
int servoDelay = 500; // Global delay time in milliseconds
int smallDelay = 100;

// Define servo rotation function
void rotate(Servo &servo, int angle) {
  int pos = servo.read();
  pos += angle;
  servo.write(constrain(pos, 0, 180));
  delay(smallDelay);
}

// Servo initialization
void setupServos() {
  myservo1.attach(8); 
  myservo2.attach(9); 
  myservo5.attach(10);  
  myservo3.attach(11); 
  myservo4.attach(12); 
  myservo6.attach(13); 

  myservo1.write(90);  // Move the first servo to the initial position
  myservo2.write(90);  // Move the second servo to the initial position
  myservo5.write(180);  // Move the base servo to the initial position
  myservo3.write(90);  // Move the first servo to the initial position
  myservo4.write(90);  // Move the second servo to the initial position
  myservo6.write(180);  // Move the base servo to the initial position
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    while (!Serial);  // Wait for serial connection

    // Initialize accelerometer
    if (!lis.begin(0x18)) {
        Serial.println("Could not find LIS3DH sensor, pls check wiring!"); // Debug info if not connected
        while (1);
    }
    lis.setRange(LIS3DH_RANGE_4_G);
    lis.setDataRate(LIS3DH_DATARATE_50_HZ);
    Serial.println("LIS3DH set.");

    // Initialize servos
    setupServos();

    // Initialize the 1st LED strip
    FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds_1, NUM_LEDS_1);
    // Initialize the 2nd LED strip
    FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds_2, NUM_LEDS_2);

    // Initialize vibration motor pin
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH); 
    //(we use the gap between 5v & signal pin to make the motor work, instead of the gap between signal pin & gnd)
}

// Define variable to store the last interaction state
String lastInteraction = "None";

// Servo Action in progress flag
bool servoActionInProgress = false;

void loop() {
    // Process sound sensor
    int soundValue = 0;
    // Read sound sensor value 32 times and calculate the average
    for (int i = 0; i < 32; i++) {
        soundValue += analogRead(SOUND_SENSOR);
    }
    soundValue >>= 5;  // get the average value

    // Map the sound value to 0-255, translate into the hue value (start from Aqua)
    int hue = (map(soundValue, 0, 200, 100, 375) % 256);

    // Set the colors of the LED strips based on the sound value
    fill_solid(leds_1, NUM_LEDS_1, CHSV(hue, 255, 255));
    //Serial.print("LED1 setted   |   ");

    fill_solid(leds_2, NUM_LEDS_2, CHSV(hue, 255, 255));
    //Serial.print("LED2 setted");

    // Display the color of the first LED strip
    FastLED.show();
    Serial.print("LED1 lighted   |   ");
    // Display the color of the second LED strip
    FastLED.show();
    Serial.print("LED2 lighted");

    // Print debug info - sound intensity and hue values on the serial monitor
    /*Serial.print("Sound Level: ");
    Serial.print(soundValue);
    Serial.print(", Hue: ");
    Serial.println(hue);*/
    
    // Pause event detection if servo action is in progress
    if (!servoActionInProgress) {
        // Process pressure sensor
        int sensorValue = analogRead(pressureSensorPin);

        // Process accelerometer
        sensors_event_t event;
        lis.getEvent(&event);

        String currentInteraction = "None";

        Serial.print("Ready to Move :D   ");
        Serial.print("|  Pressure:   ");
        Serial.print(abs(sensorValue - lastPressureValue));
        Serial.print(" | Acceleration Events:  ");
        Serial.print(accelerationEvents);
        Serial.print(" | Z-axis acceleration:  ");
        Serial.println(event.acceleration.z);

        // Determine the Interaction
        if (checkForShake(event) /*|| checkOrientationChange(event))*/) {
            currentInteraction = "Shake";
            accelerationEvents = 0;
        } else if (abs(sensorValue - lastPressureValue) > 10 && checkOrientationChange(event) && accelerationEvents >= accelerationThreshold) {
            currentInteraction = "Rotate on table";
            accelerationEvents = 0;
        } else if (abs(sensorValue - lastPressureValue) < 10 && checkOrientationChange(event) && accelerationEvents >= accelerationThreshold) {
            currentInteraction = "Rotate in Hand";
            accelerationEvents = 0;
        } else if (checkForThrow(event)) {
            currentInteraction = "Throw";
            accelerationEvents = 0;
        } else if (accelerationEvents < accelerationThreshold && abs(sensorValue - lastPressureValue) > pressureThreshold ) {
            currentInteraction = "Press";
        } else if (accelerationEvents == 0 && abs(sensorValue - lastPressureValue) < pressureThreshold ) {
            currentInteraction = "Stop";
        } else {
            currentInteraction = "Stop";
        }

        // Check if the Status has changed
        if (currentInteraction != lastInteraction) {
            Serial.println(currentInteraction);
            lastInteraction = currentInteraction;
            //Serial.println(accelerationEvents);

            // Call the output function based on the interaction
            if (currentInteraction == "Rotate on Table") {
                servoActionInProgress = true; // Set the flag to indicate the start of servo action
                rollOnTable();
                servoActionInProgress = false; // Reset the flag after the servo action is completed
            } else if  (currentInteraction == "Rotate in hand") {
                servoActionInProgress = true;
                rollInHand();
                servoActionInProgress = false;
            } else if  (currentInteraction == "Throw") {
                servoActionInProgress = true;
                throwOutput();
                servoActionInProgress = false;
            } else if (currentInteraction == "Shake") {
                servoActionInProgress = true;
                shakeOutput();
                servoActionInProgress = false;
            } else if (currentInteraction == "Press") {
                Serial.println(sensorValue);
                Serial.println(lastPressureValue);
                servoActionInProgress = true;
                pressOutput();
                servoActionInProgress = false;
            } else if (currentInteraction == "Stop") {
                servoActionInProgress = true;
                stopOutput();
                servoActionInProgress = false;
            }
        }

        // Update the last pressure sensor value
        lastPressureValue = sensorValue;
        //Serial.print(lastPressureValue);
        //accelerationEvents = 0;

        delay(400);
    }
}

// <-------- Input Detect ----------> 

// Throw Detect
bool checkForThrow(sensors_event_t &event) {
    static float lastZ = 0;  // last z-axis acceleration
    static unsigned long throwStartTime = 0;  // Throw start time
    float currentZ = event.acceleration.z;

    switch (throwState) {
        case WAITING:
            if (currentZ > Z_UP_THROW_THRESHOLD) {
                throwState = THROWING_UP;
                throwStartTime = millis();  // Record the throw start time
            }
            break;

        case THROWING_UP:
            // Check the duration of the throw to avoid false trigger
            if (millis() - throwStartTime > 200 && currentZ < lastZ && currentZ < Z_DOWN_ZERO_THRESHOLD) {
                //Serial.println("Top of the throw reached! Throw detected.");
                throwState = WAITING; // Reset for the next detection
                return true;
            }
            break;
    }

    lastZ = currentZ;  // Update last z-axis acceleration
    return false;
}

// Orientation Detect
bool checkOrientationChange(sensors_event_t &event) {
    static int lastX = 0, lastY = 0, lastZ = 0;  // last direction status
    int currentX = (event.acceleration.x > DIRECTION_THRESHOLD) ? 1 : (event.acceleration.x < -DIRECTION_THRESHOLD) ? -1 : 0;
    int currentY = (event.acceleration.y > DIRECTION_THRESHOLD) ? 1 : (event.acceleration.y < -DIRECTION_THRESHOLD) ? -1 : 0;
    int currentZ = ((event.acceleration.z - Z_DOWN_ZERO_THRESHOLD) > DIRECTION_THRESHOLD) ? 1 : ((event.acceleration.z - Z_DOWN_ZERO_THRESHOLD) < -DIRECTION_THRESHOLD) ? -1 : 0;

    if (currentX != lastX || currentY != lastY || currentZ != lastZ) {
        directionChangeCount++; // Accumulate direction change count
        lastX = currentX;
        lastY = currentY;
        lastZ = currentZ;

        if (directionChangeCount >= 3) {
            directionChangeCount = 0; // Reset direction change counter
            accelerationEvents++;
            Serial.println(event.acceleration.x - DIRECTION_THRESHOLD);
            return true;
        }
    }
    return false;
}

// Shake Detect
bool checkForShake(sensors_event_t &event) {
    static float lastX = 0, lastY = 0, lastZ = 9.8;  // last acceleration data
    float delta_x = abs(event.acceleration.x - lastX);
    float delta_y = abs(event.acceleration.y - lastY);
    float delta_z = abs(event.acceleration.z - lastZ);

    lastX = event.acceleration.x;
    lastY = event.acceleration.y;
    lastZ = event.acceleration.z;

    unsigned long currentTime = millis();

    if (delta_x > SHAKE_THRESHOLD || delta_y > SHAKE_THRESHOLD || delta_z > SHAKE_THRESHOLD) {
        if (currentTime - lastShakeTime < SHAKE_TIME_WINDOW) {
            shakeCount++;
        } else {
            shakeCount = 1;  // Reset shake count
        }
        lastShakeTime = currentTime;
    }

    if (shakeCount >= SHAKE_COUNT_THRESHOLD) {
        shakeCount = 0;  // Reset shake counter
        lastShakeTime = currentTime;  // Update time to avoid immediate reset
        accelerationEvents++;
        return true;
    }

    return false;
}

void detectEvents() {
    // Reset counters and time
    directionChangeCount = 0;
    shakeCount = 0;
    lastShakeTime = 0;
}

// <-------- Output Control ----------> 

// Define output functions

void rollOnTable() {
  rotate(myservo2, 90);
  rotate(myservo4, 90);
  delay(servoDelay);
  rotate(myservo2, -180);
  rotate(myservo4, -180);
  delay(servoDelay);
  rotate(myservo2, -90);
  rotate(myservo4, -90);
  delay(servoDelay);
  Serial.println("Servo: rolls on table");
}

void rollInHand() {
  rotate(myservo2, -90);
  rotate(myservo4, -90);
  delay(servoDelay);
  rotate(myservo1, 90);
  rotate(myservo3, 90);
  delay(servoDelay);
  rotate(myservo2, 180);
  rotate(myservo4, 180);
  delay(servoDelay);
  rotate(myservo1, -180);
  rotate(myservo3, -180);
  delay(servoDelay);
  
  Serial.println("Servo: rolls in hand");
}

void shakeOutput() {
    Serial.println("Start Vibrate");
    for (int i = 0; i < 2; i++) {
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);  // Turn on the vibration motor 
        delay(1000);  // Vibrate for one second
        digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn off the vibration motor
        delay(1000);  // Stop for one second
    }

  rotate(myservo2, 45);
  rotate(myservo4, -45);
  delay(servoDelay);
  rotate(myservo2, -90);
  rotate(myservo4, 90);
  delay(servoDelay);
  delay(servoDelay);
  rotate(myservo2, 90);
  rotate(myservo4, -90);
  delay(servoDelay);
  rotate(myservo2, -90);
  rotate(myservo4, 90);
  delay(servoDelay);
  rotate(myservo2, 45);
  rotate(myservo4, -45);
  delay(servoDelay);

  digitalWrite(VIBRATION_MOTOR_PIN, HIGH);

  Serial.println("Servo: Shake");
}

void pressOutput() {
  // Calculate the change in the pressure sensor
  int pressureChange = abs(analogRead(pressureSensorPin) - lastPressureValue);

  // Map the pressure change to the range of -90 to 90 degrees
  int mappedAngle = map(pressureChange, 30, 200, -90, 90);

  // Print the mapped angle for debugging
  Serial.print("Angle: ");
  Serial.println(mappedAngle);

  // Control the servo to rotate according to the mapped angle
  rotate(myservo2, -mappedAngle);
  rotate(myservo4, mappedAngle);
  delay(servoDelay);
  rotate(myservo1, mappedAngle);
  rotate(myservo3, -mappedAngle);
  delay(servoDelay);
  rotate(myservo2, 2 * mappedAngle);
  rotate(myservo4, -2 * mappedAngle);
  delay(servoDelay);
  delay(servoDelay);
  rotate(myservo1, -2 * mappedAngle);
  rotate(myservo3, 2 * mappedAngle);

  Serial.println("Servo: presses");
}

void throwOutput() {
  rotate(myservo5, -180);
  rotate(myservo6, -180);
  delay(servoDelay);
  delay(servoDelay);
  delay(servoDelay);

  rotate(myservo5, 180);
  rotate(myservo6, 180);
  delay(servoDelay);

  Serial.println("Throw test");
}

// Define the LED control function
void stopLedOutput() {
  // Blink red twice
  for (int i = 0; i < 2; i++) {
    fill_solid(leds_1, NUM_LEDS_1, CRGB::Red);
    fill_solid(leds_2, NUM_LEDS_2, CRGB::Red);
    FastLED.show();
    delay(250);  // Keep red for 250 ms
    fill_solid(leds_1, NUM_LEDS_1, CRGB::Black);
    fill_solid(leds_2, NUM_LEDS_2, CRGB::Black);
    FastLED.show();
    delay(250);  // Off for 250 ms
  }
}

void stopOutput() {
  myservo1.write(90);  // First servo returns to initial position
  myservo2.write(90);  // Second servo returns to initial position
  myservo5.write(180); // Base servo returns to initial position
  delay(servoDelay);
  myservo3.write(90);  
  myservo4.write(90);  
  myservo6.write(180); 
  Serial.println("Servo back");

  // LED turn into red and blink twice
  stopLedOutput();
  delay(400);
  stopLedOutput();
}