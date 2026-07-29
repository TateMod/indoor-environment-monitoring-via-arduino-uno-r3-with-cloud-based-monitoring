#include <dht_nonblocking.h>//Imported Librarys 
#include <LiquidCrystal_74HC595.h>
#include <EEPROM.h>
#include <Wire.h>

// Cloud integration variables - synchronize with IoT dashboard for remote monitoring
bool  cloud_fanState = false;      // Current fan operational state
float cloud_humidity = 0;          // Humidity percentage (0-100%)
int   cloud_lightLevel = 0;        // Ambient light sensor ADC value (0-1023)
float cloud_temperature = 0;       // Temperature in Celsius
bool  cloud_ledState = false;      // LED array activation status

// Sensor error detection - implements timeout-based fault detection
unsigned long lastSuccessfulDHTReading = 0;  // Timestamp of last valid DHT reading
unsigned long lastLightReading = 0;          // Timestamp of last light sensor check
const unsigned long DHT_TIMEOUT = 10000;     // 10s timeout for DHT sensor failure
const unsigned long LIGHT_CHECK_INTERVAL = 5000; // Check light sensor every 5s
bool dhtSensorError = false;                 // DHT sensor fault flag
bool lightSensorError = false;               // Photoresistor fault flag
int lastLightLevel = -1;                     // Previous light reading for stuck detection
int lightStuckCount = 0;                     // Counter for persistent zero readings

// System state variables
bool autoProcessingActive = false;  // Flag: true when AUTO mode actively processing
int offButton = A5;                 // Pin for system power control button
bool systemOn = false;              // System power state (false = standby)
bool firstReadingDone = false;      // Flag: ensures valid sensor data before operation
int systemState = 0;                // 0 = AUTO mode menu, 1 = MANUAL mode
int previousSystemState = -1;       // Previous state for change detection
int backlight = A4;                 // LCD backlight control pin
int lightPin = A0;                  // Photoresistor voltage divider input (0-5V analog)
int lightLevel = 0;                 // Current light sensor ADC value (0-1023)

// LED output pins - three independent LEDs for progressive illumination
int led1Pin = 6;                    // LED1 (brightest conditions) - PWM capable
int led2Pin = 9;                    // LED2 (medium conditions) - PWM capable
int led3Pin = 10;                   // LED3 (dimmest conditions) - PWM capable
int backButton = 2;                 // Back button for menu navigation

// Control state variables
bool fanState = false;              // Current fan operational state
bool ledState = false;              // Overall LED array state
float temperature;                  // Current temperature reading (°C)
float humidity;                     // Current humidity reading (% RH)

// Joystick pins - provides 2-axis analog input + button
int joyPin = A1;                    // X-axis for LED brightness control
int X_pin = A1;                     // X-axis (duplicate reference)
int Y_pin = A2;                     // Y-axis for menu navigation and fan speed
int SW_pin = 8;                     // Joystick button (active LOW with INPUT_PULLUP)

// Manual mode LED brightness control - three separate variables for independent control
int currentBrightness = 0;          // LED1 brightness (0-255 PWM)
int currentBrightness2 = 0;         // LED2 brightness (0-255 PWM)
int currentBrightness3 = 0;         // LED3 brightness (0-255 PWM)

// Motor control pins - L293D H-Bridge driver interface
int ENA = 3;                        // Enable/PWM speed control (0-255)
int IN1 = 4;                        // Direction control bit 1
int IN2 = 5;                        // Direction control bit 2

// Menu navigation
int currentSelection = 0;           // Selected menu option (0=AUTO, 1=FAN, 2=LED, 3=SETTINGS)
unsigned long lastMoveTime = 0;     // Timestamp for joystick debouncing
int moveCooldown = 300;             // Minimum time between menu movements (ms)
int tempThresh = 18;                // Fan activation temperature (°C) - default 18°C
const int EEPROM_ADDR = 0;          // EEPROM memory address for threshold storage

// Custom LCD character - settings gear icon (8x5 pixel pattern)
byte name1x14[] = { B00000, B10101, B01110, B11011, B01110, B10101, B00000, B00000 };

// LCD connected via 74HC595 shift register to reduce pin usage (7 pins → 3 pins)
LiquidCrystal_74HC595 lcd(11, 13, 12, 0, 1, 2, 3, 4, 5);

// DHT11 sensor configuration
#define DHT_SENSOR_TYPE DHT_TYPE_11  // DHT11 type identifier
static const int DHT_SENSOR_PIN = 7; // Digital pin for single-wire protocol
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  Serial.begin(9600);               // Initialize serial communication for debugging and IoT bridge
  pinMode(SW_pin, INPUT_PULLUP);    // Joystick button (active LOW)
  delay(1500);                      // 1.5s delay for LCD and sensor initialization
  lcd.begin(16, 2);                 // Initialize 16x2 LCD display
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
  pinMode(backButton, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(offButton, INPUT_PULLUP);
  pinMode(backlight, OUTPUT);
  loadThresholdFromEEPROM();        // Load saved temperature threshold from EEPROM
  lcd.createChar(0, name1x14);      // Create custom settings icon character
}

// Non-blocking temperature and humidity measurement
// DHT11 requires 250ms to complete reading - this prevents UI freezing
static bool measure_environment(float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > 3000ul) {
    if (dht_sensor.measure(temperature, humidity)) {
      measurement_timestamp = millis();
      return true;  // Valid reading obtained
    }
  }
  return false;     // Waiting for sensor
}

// Detects DHT11 sensor timeout and implements fail-safe shutdown
// If no valid reading within 10 seconds stops fan to prevent uncontrolled operation
void checkDHTSensor() {
  if (millis() - lastSuccessfulDHTReading > DHT_TIMEOUT && firstReadingDone) {
    if (!dhtSensorError) {
      dhtSensorError = true;
      if (fanState) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
        fanState = false;
        cloud_fanState = false;
      }
      Serial.println("ERROR: DHT11 sensor not responding");
    }
  }
}

// Detects photoresistor disconnection or failure
// Checks for persistently low readings (<10 ADC) indicating sensor fault
void checkLightSensor() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > LIGHT_CHECK_INTERVAL) {
    lastCheck = millis();
    if (lightLevel < 10) {
      lightStuckCount++;
      if (lightStuckCount >= 2) {
        if (!lightSensorError) {
          lightSensorError = true;
        }
      }
    } else {
      lightStuckCount = 0;
      lightSensorError = false;
    }
    lastLightLevel = lightLevel;
  }
}

// Processes joystick Y-axis input for menu selection with debouncing
// Joystick down (>550) moves selection right, up (<450) moves left
void handleSelection() {
  int yValue = analogRead(Y_pin);
  if (millis() - lastMoveTime < moveCooldown) return;
  if (yValue > 550) {
    if (currentSelection < 3) {
      currentSelection++;
      lastMoveTime = millis();
    }
  }
  else if (yValue < 450) {
    if (currentSelection > 0) {
      currentSelection--;
      lastMoveTime = millis();
    }
  }
}

// Renders menu options on LCD with selection cursor
// Layout: [AUTO < FAN   LED   setting]
void displaySelection() {
  lcd.setCursor(0, 1); lcd.print("AUTO");
  lcd.setCursor(6, 1); lcd.print("FAN");
  lcd.setCursor(10, 1); lcd.print("LED");
  lcd.setCursor(14, 1); lcd.write((uint8_t)0);
  lcd.setCursor(4, 1); lcd.print(" ");
  lcd.setCursor(9, 1); lcd.print(" ");
  lcd.setCursor(13, 1); lcd.print(" ");
  lcd.setCursor(15, 1); lcd.print(" ");
  switch(currentSelection) {
    case 0: lcd.setCursor(4, 1); lcd.print("<"); break;
    case 1: lcd.setCursor(9, 1); lcd.print("<"); break;
    case 2: lcd.setCursor(13, 1); lcd.print("<"); break;
    case 3: lcd.setCursor(15, 1); lcd.print("<"); break;
  }
}

// Manual LED brightness control via joystick X-axis
// Sequential control: LED3 -> LED2 -> LED1 for intuitive dimmer behavior
void ledManual() {
  int val = analogRead(joyPin);
  if (val > 600) {
    if (currentBrightness < 255) currentBrightness += 5;
    else if (currentBrightness2 < 255) currentBrightness2 += 5;
    else if (currentBrightness3 < 255) currentBrightness3 += 5;
  }
  else if (val < 400) {
    if (currentBrightness3 > 0) currentBrightness3 -= 5;
    else if (currentBrightness2 > 0) currentBrightness2 -= 5;
    else if (currentBrightness > 0) currentBrightness -= 5;
  }
  analogWrite(led1Pin, currentBrightness);
  analogWrite(led2Pin, currentBrightness2);
  analogWrite(led3Pin, currentBrightness3);
  cloud_ledState = (currentBrightness > 0 || currentBrightness2 > 0 || currentBrightness3 > 0);
  delay(10);
}

// Manual fan speed control via joystick Y-axis
// Persistent speed control - maintains setting when joystick returns to center
void fanManual() {
  int yValue = analogRead(Y_pin);
  static int savedSpeed = 0;
  if (yValue >= 700) {
    savedSpeed += 5;
    savedSpeed = constrain(savedSpeed, 0, 255);
  } 
  else if (yValue <= 300) {
    savedSpeed -= 5;
    savedSpeed = constrain(savedSpeed, 0, 255);
  }
  if (savedSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, savedSpeed);
    cloud_fanState = true;
    fanState = true;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    cloud_fanState = false;
    fanState = false;
  }
  delay(50);
}

// Executes action when joystick button pressed
// 0=AUTO mode, 1=FAN manual, 2=LED manual, 3=Settings menu
void executeSelection() {
  if (digitalRead(SW_pin) == LOW) {
    switch(currentSelection) {
      case 0: systemState = 0; autoProcessingActive = true; resetManualControls(); break;
      case 1: systemState = 1; autoProcessingActive = false; break;
      case 2: systemState = 1; autoProcessingActive = false; break;
      case 3: adjustThreshold(); break;
    }
    lcd.clear();
    delay(500);
  }
}

// Progressive LED illumination based on ambient light level
// 3-stage activation: LED3 (0-350) -> LED2 (351-700) -> LED1 (700-1023)
void lightprocessing() {
  int led1Brightness, led2Brightness, led3Brightness;
  int invertedLight = 1023 - lightLevel;
  if (invertedLight <= 350) led3Brightness = map(invertedLight, 0, 350, 0, 255);
  else led3Brightness = 255;
  if (invertedLight >= 351 && invertedLight <= 700) led2Brightness = map(invertedLight, 351, 700, 0, 255);
  else if (invertedLight > 682) led2Brightness = 255;
  else led2Brightness = 0;
  if (invertedLight >= 700) led1Brightness = map(invertedLight, 700, 1023, 0, 255);
  else led1Brightness = 0;
  analogWrite(led1Pin, constrain(led1Brightness, 0, 255));
  analogWrite(led2Pin, constrain(led2Brightness, 0, 255));
  analogWrite(led3Pin, constrain(led3Brightness, 0, 255));
  cloud_ledState = (led1Brightness > 0 || led2Brightness > 0 || led3Brightness > 0);
}

// Temperature-based fan control with 2°C hysteresis
// Fan ON when temp ≥ threshold, OFF when temp < (threshold - 2°C)
// Prevents rapid on/off cycling and extends motor life
void dhtProcessing() {
  if (dhtSensorError) {
    if (fanState) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      fanState = false;
      cloud_fanState = false;
    }
    return;
  }
  if (fanState == false) {
    if (temperature >= tempThresh) {
      fanState = true;
      cloud_fanState = true;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 200);
    }
  }
  else {
    if (temperature < tempThresh - 2) {
      fanState = false;
      cloud_fanState = false;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    }
  }
}

// Resets all manual control states when entering AUTO mode
void resetManualControls() {
  currentBrightness = 0;
  currentBrightness2 = 0;
  currentBrightness3 = 0;
  analogWrite(led1Pin, 0);
  analogWrite(led2Pin, 0);
  analogWrite(led3Pin, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  fanState = false;
  cloud_fanState = false;
}

// Loads saved temperature threshold from EEPROM at startup
// Valid range: 0-50°C, default: 18°C if EEPROM empty/corrupted
void loadThresholdFromEEPROM() {
  int storedValue = EEPROM.read(EEPROM_ADDR);
  if (storedValue != 255 && storedValue >= 0 && storedValue <= 50) {
    tempThresh = storedValue;
  }
  else {
    tempThresh = 18;
  }
}

// Interactive menu for adjusting temperature threshold
// Joystick UP/DOWN to adjust, button to save, back button to cancel
void adjustThreshold() {
  while (digitalRead(SW_pin) == LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Threshold:");// lcd layput
  while (true) {
    int yValue = analogRead(Y_pin);
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(tempThresh);
    lcd.print("C  ");
    if (yValue > 700 && tempThresh < 50) {//joystcik values
      tempThresh++;
      delay(200);
    }
    else if (yValue < 300 && tempThresh > 0) {//joystick values
      tempThresh--;
      delay(200);
    }
    if (digitalRead(backButton) == LOW) {//if change is canceled output
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Canceled");
      delay(1000);
      break;
    }
    if (digitalRead(SW_pin) == LOW) {//svade screen
      EEPROM.write(EEPROM_ADDR, tempThresh);// getting amouts from memory
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Saved!");
      delay(1000);
      break;
    }
  }
  lcd.clear();
}

void loop() {
  // Power button handler - toggles system on/off
  if (digitalRead(offButton) == LOW) {
    systemOn = !systemOn;
    if (!systemOn) {
      lcd.noDisplay();
      resetManualControls();
      digitalWrite(backlight, HIGH); // Backlight ON
    } else {
      lcd.display();
      lcd.clear();
      firstReadingDone = false;
      digitalWrite(backlight, LOW); // turns back light LOW (OFF)
    }
    delay(500);
  }
  
  if (!systemOn) return;
  
  // Startup sequence - wait for first valid sensor reading
  if (!firstReadingDone) {
    if (measure_environment(&temperature, &humidity)) {
      firstReadingDone = true;
      lastSuccessfulDHTReading = millis();
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Starting...     "); // starting screen
      return;
    }
  }
  
  checkDHTSensor();
  checkLightSensor();
  
  if (systemState != previousSystemState) {
    lcd.clear();
    previousSystemState = systemState;
  }
  
  // AUTO MODE
  if (systemState == 0) {
    if (measure_environment(&temperature, &humidity)) {
      lastSuccessfulDHTReading = millis();
      dhtSensorError = false;
      cloud_temperature = temperature;//cloud vars
      cloud_humidity = humidity;
    }
    handleSelection();
    executeSelection();
    lcd.setCursor(0, 0);
    lcd.print(temperature, 0);//LCD display
    lcd.print("C");
    lcd.print(dhtSensorError ? "!" : " ");
    lcd.setCursor(5, 0);
    lcd.print(humidity, 0);
    lcd.print("%");
    lcd.print(dhtSensorError ? "!" : " ");
    lightLevel = analogRead(lightPin);//light pin reading
    cloud_lightLevel = lightLevel;// store value for cloud
    lcd.setCursor(11, 0);
    if (lightSensorError) lcd.print("ERR!");// error output
    else lcd.print(lightLevel < 300 ? "dark" : "lit ");
    displaySelection();
    if (autoProcessingActive) {//run automatic processing if enabled
      static unsigned long lastLightUpdate = 0;// last time light processing ran
      if (millis() - lastLightUpdate >= 500) {
        lightprocessing();
        lastLightUpdate = millis();
      }
      dhtProcessing();// data processing
    }
  }
  
  // MANUAL MODE
  else if (systemState == 1) {
    lcd.setCursor(0, 0);
    lcd.print("MANUAL MODE     ");
    if (currentSelection == 1) fanManual();// if fan mode sected
    else if (currentSelection == 2) ledManual();// if led mode selected
    if (digitalRead(backButton) == LOW) {
      systemState = 0;
      lcd.clear();
      delay(500);
    }
  }

  // IoT cloud data transmission - CSV format for bridge applications
  Serial.print("DATA:"); 
  Serial.print(cloud_fanState);    Serial.print(",");
  Serial.print(cloud_temperature); Serial.print(",");
  Serial.print(cloud_humidity);    Serial.print(",");
  Serial.print(cloud_lightLevel);  Serial.print(",");
  Serial.println(cloud_ledState);
  
  delay(100);
}
