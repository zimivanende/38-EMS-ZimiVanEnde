/* ------------------ Globals ------------------ */

/* Include needed libs */
#include "adafruit_io_config.h" // Adafruit IO setup file
#include <DHT.h>                // Library for temperature and humidity sensor DHT11
#include <Wire.h>               // Library for I2C communication with CO2 sensor T6713
#include <TFT_eSPI.h>           // Hardware-specific library


/* Define needed constants
  (Easy changes without needing to change functions)
*/
#define OPTI_BUTTON_PIN 0                   // Pin for button for manual optimization request
#define DHT_PIN 27                          // Pin for DHT11 sensor
#define DHT_TYPE DHT11                      // DHT sensor type
#define RED_LED_PIN 26                      // Pin for red led
#define GREEN_LED_PIN 25                    // Pin for green led
#define BLUE_LED_PIN 33                     // Pin for blue led
#define ADDR_6713 0x15                      // I2C address for T6713
#define RUNNING_AVG_READINGS 10             // Running average accuracy
#define CLOUD_UPDATES_PER_MINUTE 1          // Posibility to decrease or increase cloud updates per minute (for example: 2 => every 30sec)
#define OPTIMIZATION_UPDATES_PER_MINUTE 1   // Posibility to decrease or increase cloud updates per minute (for example: 2 => every 30sec)


/* Enum for states */
enum States {
  OPTIMAL,    // Everything is perfect
  WARNING,    // Be aware => possibility to activate optimization
  DANGER      // Danger => optimization will be performed instandly
};


/* Create global variables */
TFT_eSPI tft = TFT_eSPI();                      // Constructor for the TFT
DHT dht = DHT(DHT_PIN, DHT_TYPE);               // Constructor for the DHT
float humidity_readings[RUNNING_AVG_READINGS];  // Array for running average of humidity
float temp_c_readings[RUNNING_AVG_READINGS];    // Array for running average of temperature (C)
float temp_f_readings[RUNNING_AVG_READINGS];    // Array for running average of temperature (F)
int co2_readings[RUNNING_AVG_READINGS];         // Array for running average of CO2

float last_humidity = 0;                        // To limit sensor reads keep last humidity value
float last_temp_c = 0;                          // To limit sensor reads keep last temperature celcius value
float last_temp_f = 0;                          // To limit sensor reads keep last temperature fahrenheit value
float last_hi_c = 0;                            // To limit sensor reads keep last heat index celcius value
float last_hi_f = 0;                            // To limit sensor reads keep last heat index fahrenheit value
int last_co2 = 0;                               // To limit sensor reads keep last co2 value

bool optimization_started = false;              // If in warning state we know if optiization is active and keep checking
bool optimization_requested = false;            // On optimization interupt this value will be set true

States current_state = OPTIMAL;                 // Keep current state

unsigned long last_cloud_update_time = 0;       // To limit our Adafruit IO pushes we keep track of last push time
unsigned long last_optimization_update_time = 0;// To limit our Adafruit IO pushes we keep track of last push time


/* Create Adafruit IO feeds */
AdafruitIO_Feed *humidity_feed = io.feed("feed_humidity");                                  // Humidity
AdafruitIO_Feed *temp_c_feed = io.feed("feed_temp_c");                                      // Temperature in Celcius
AdafruitIO_Feed *temp_f_feed = io.feed("feed_temp_f");                                      // Temperature in Fahrenheit
AdafruitIO_Feed *hi_c_feed = io.feed("feed_hi_c");                                          // Heat index in Celcius
AdafruitIO_Feed *hi_f_feed = io.feed("feed_hi_f");                                          // Heat index in Fahrenheit
AdafruitIO_Feed *co2_feed = io.feed("feed_co2");                                            // CO2
AdafruitIO_Feed *state_feed = io.feed("feed_state");                                        // Current state
AdafruitIO_Feed *humidity_optimization_feed = io.feed("feed_humidity_optimization");        // Optimize humidity => -1 = decrease, 0 = stop, 1 = increase
AdafruitIO_Feed *temperature_optimization_feed = io.feed("feed_temperature_optimization");  // Optimize temperature => -1 = decrease, 0 = stop, 1 = increase
AdafruitIO_Feed *co2_optimization_feed = io.feed("feed_co2_optimization");                  // Optimize co2 => -1 = decrease, 0 = stop, 1 = increase


/* ------------------ Arduino functions ------------------ */

/* Setup Arduino before looping */
void setup(void) {
  tft.init();
  tft.setRotation(1);         // Screen in landscape

  connectAdafruitIO();
  
  initReadings();             
  initOptimizationButton();
  initRGBLed();
  
  Serial.begin(9600);         // Start Serial
  Wire.begin ();              // Start I2C
  dht.begin();                // Start DHT
}


/* "Brain" aka loop */
void loop() {
  updateSensorData();

  stateChecker();

  if(optimization_requested){         // If optimization button is pressed and interupt is called => Optimize
    executeOptimizationMeasures();
    optimization_requested = false;   // Reset request
  }

  delay(1000);                        // Give time for other messages before updating outputs
  
  updateOutputs();

  delay(1000);                        // Wait a between measurements
}


/* ------------------ Setup functions ------------------ */

/* Connect to WiFi and Adafruit IO */
void connectAdafruitIO(){
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_BLACK);

  tft.print("Connecting to Adafruit IO");
  io.connect();

  while (io.status() < AIO_CONNECTED) {     // Do the following until wifi is connected
    delay(500);
    tft.print(".");                         // Show user it is loading
  }

  tft.fillScreen(TFT_WHITE);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_BLACK);

  tft.println(io.statusText());             // Show status message
}

/* Fill running average arrays with 0's */
void initReadings(){
  for (int reading = 0; reading < RUNNING_AVG_READINGS; reading++) {
    humidity_readings[reading] = 0;
    temp_c_readings[reading] = 0;
    temp_f_readings[reading] = 0;
    co2_readings[reading] = 0;
  }
}

/* Init optimization button pin */
void initOptimizationButton(){
  pinMode(OPTI_BUTTON_PIN, INPUT_PULLUP);
}

/* Init RGB led pins */
void initRGBLed(){
  // Assign RGB led pins to channels
  ledcAttachPin(RED_LED_PIN, 1);
  ledcAttachPin(GREEN_LED_PIN, 2);
  ledcAttachPin(BLUE_LED_PIN, 3);
  
  // Initialize channels
  ledcSetup(1, 12000, 8); // Channel 1, 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8); // Channel 2, 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 12000, 8); // Channel 3, 12 kHz PWM, 8-bit resolution
}


/* ------------------ Update/Alert functions ------------------ */

/* Fetch latest data from sensors */
void updateSensorData(){
  last_humidity = getHumidity();
  last_temp_c = getTemperature(false);
  last_temp_f = getTemperature(true);
  last_hi_c = getHeatIndex(last_temp_c, last_humidity, false);
  last_hi_f = getHeatIndex(last_temp_f, last_humidity, true);
  last_co2 = getCO2();
}

/* Print data to Serial */
void updateSerial(String current_state, float humidity, float temp_c, float temp_f, float hi_c, float hi_f, int co2){
  Serial.print("State: "); Serial.println(current_state);
  Serial.print("RH: ");  Serial.print(humidity, 1);  Serial.println("%");
  Serial.print("Temp: "); Serial.print(temp_c, 1); Serial.print("C° / "); Serial.print(temp_f, 1); Serial.println("F°");
  Serial.print("HI: "); Serial.print(hi_c, 1); Serial.print("C° / "); Serial.print(hi_f, 1); Serial.println("F°");
  Serial.print("CO2: "); Serial.print(co2); Serial.println("ppm");
}

/* Print data to Screen */
void updateScreenData(uint16_t color, String current_state, float humidity, float temp_c, float temp_f, float hi_c, float hi_f, int co2){
  tft.fillScreen(color);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_BLACK);
  
  tft.print("State: "); tft.println(current_state);
  tft.print("RH: ");  tft.print(humidity, 1);  tft.println("%");
  tft.print("Temp: "); tft.print(temp_c, 1); tft.print("C° / "); tft.print(temp_f, 1); tft.println("F°");
  tft.print("HI: "); tft.print(hi_c, 1); tft.print("C° / "); tft.print(hi_f, 1); tft.println("F°");
  tft.print("CO2: "); tft.print(co2); tft.println("ppm");
}

/* Send data to Adafruit IO feeds */
void updateCloudData(String current_state, float humidity, float temp_c, float temp_f, float hi_c, float hi_f, int co2){  
  if(last_cloud_update_time + (60000 / CLOUD_UPDATES_PER_MINUTE) < millis()){   // Check if alowed to send according to send frequency;
    if(io.status() >= AIO_NET_CONNECTED){                                       // If connected to Adafruit IO, push data
      state_feed->save(current_state);
      humidity_feed->save(humidity);
      temp_c_feed->save(temp_c);
      temp_f_feed->save(temp_f);
      hi_c_feed->save(hi_c);
      hi_f_feed->save(hi_f);
      co2_feed->save(co2);
      last_cloud_update_time = millis();                                        // Update last update time
    }
    else{
      connectAdafruitIO();                                                      // If not connected to Adafruit IO,  try to connect again
    }
  }
}

/* Update Serial, Screen and Adafruit IO */
void updateOutputs(){  
  String state = "";
  uint16_t color = 0;
  
  if(current_state == OPTIMAL){
    color = TFT_GREEN;
    state = "OPTIMAL";
  }
  else if(current_state == WARNING){
    color = TFT_ORANGE;
    state = "WARNING";
  }
  else if(current_state == DANGER){
    color = TFT_RED;
    state = "DANGER";
  }

  updateSerial(state, last_humidity, last_temp_c, last_temp_f, last_hi_c, last_hi_f, last_co2);
  updateScreenData(color, state, last_humidity, last_temp_c, last_temp_f, last_hi_c, last_hi_f, last_co2);
  updateCloudData(state, last_humidity, last_temp_c, last_temp_f, last_hi_c, last_hi_f, last_co2);
}

/* Quick function to print a message on the screen */
void printMessageOnScreen(String message){
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_BLACK);

  tft.println(message);             // Show status message
}

/* Change color of led =>RGB values from 0 to 255 */
void changeLedColor(int red, int green, int blue){
  ledcWrite(1, red);
  ledcWrite(2, green);
  ledcWrite(3, blue);
}


/* ------------------ State functions ------------------ */

/* Activates Optimal state
   & Updates led color to green 
   & Disable manual optimization button
   & Stops optimizing
*/
void activateOptimalState(){
  current_state = OPTIMAL;
  detachInterrupt(OPTI_BUTTON_PIN);
  changeLedColor(0, 255, 0);
  cancelOptimizationMeasures();
}

/* Activates Warning state 
   & Updates led color to orange 
   & Enable manual optimization button if optimization is not started already
   & If optimization is started already keep checking
*/
void activateWarningState(){
  current_state = WARNING;
  if(optimization_started){
    executeOptimizationMeasures();
  }
  else{
    attachInterrupt(OPTI_BUTTON_PIN, optimizationButtonHandler, FALLING);
  }
  changeLedColor(255, 80, 0);
}

/* Activates Danger state
   & Updates led color to red 
   & Disable manual optimization button
   & Start optimizing
*/
void activateDangerState(){
  current_state = DANGER;
  detachInterrupt(OPTI_BUTTON_PIN);
  changeLedColor(255, 0, 0);
  executeOptimizationMeasures();
}

/* Check which state must be activated and activate that state */
void stateChecker(){
  if((last_humidity < 30)||(last_humidity > 70)||(last_temp_c < 15)||(last_temp_c > 30)|| last_co2 > 2000){
    activateDangerState();
  }
  else if((last_humidity < 45)||(last_humidity > 55)||(last_temp_c < 20)||(last_temp_c > 22)||(last_co2 > 1000)){
    activateWarningState();
  }
  else{
    activateOptimalState();
  }
}


/* ------------------ Optimization functions ------------------ */

/* Cancels all optimizations */
void cancelOptimizationMeasures(){
  optimization_started = false;

  if(last_optimization_update_time + (60000 / OPTIMIZATION_UPDATES_PER_MINUTE) < millis()){   // Check if alowed to send according to send frequency;
    humidity_optimization_feed->save(0);
    temperature_optimization_feed->save(0);
    co2_optimization_feed->save(0);
    last_optimization_update_time = millis();                                                 // Update last update time
  }
}

/* Start/stop optimizations if needed */
void executeOptimizationMeasures(){
  String message = "OPTIMIZING";
  Serial.println(message);
  printMessageOnScreen(message);
  optimization_started = true;

  if(last_optimization_update_time + (60000 / OPTIMIZATION_UPDATES_PER_MINUTE) < millis()){   // Check if alowed to send according to send frequency;
    if(last_humidity < 45){
      humidity_optimization_feed->save(1);
    }
    if(last_humidity > 55){
      humidity_optimization_feed->save(-1);
    }
    if(last_humidity >= 45 && last_humidity <= 55){
      humidity_optimization_feed->save(0);
    }
  
    
    if(last_temp_c < 20){
      temperature_optimization_feed->save(1);
    }
    if(last_temp_c > 22){
      temperature_optimization_feed->save(-1);
    }
    if(last_temp_c >= 20 && last_temp_c <= 22){
      temperature_optimization_feed->save(0);
    }
    
    if(last_co2 < 250){
      co2_optimization_feed->save(1);
    }
    if(last_co2 > 1000){
      co2_optimization_feed->save(-1);
    }
    if(last_co2 >= 250 && last_co2 <= 1000){
      co2_optimization_feed->save(0);
    }
  
    
    last_optimization_update_time = millis();                                                 // Update last update time
  }
}

/* Request manual optimization */
void optimizationButtonHandler(){
  String message = "OPTIMIZATION REQUESTED";
  Serial.println(message);
  printMessageOnScreen(message);
  optimization_requested = true;
}


/* ------------------ Sensor data functions ------------------ */

/* Fetch latest sensor data
   && Update running average
*/
float getHumidity() {
  float humidity = dht.readHumidity();                                      // Fetch latest sensor data
  if(isnan(humidity)){                                                      // If sensor not working => alert user and return 0
    Serial.println("Humidity sensor malfunctioning");
    return 0;
  }
  
  for (int reading = 0; reading < RUNNING_AVG_READINGS - 1; reading++) {    // Shift values one place in running average array
    humidity_readings[reading] = humidity_readings[reading + 1];
  }

  humidity_readings[RUNNING_AVG_READINGS - 1] = humidity;                   // Insert new value

  int sum = 0;
  for (int reading = 0; reading < RUNNING_AVG_READINGS; reading++) {        // Calculate sum
    sum += humidity_readings[reading];
  }
  
  float average = sum / RUNNING_AVG_READINGS;                               // Calculate average
  
  return average;
}

/* Fetch latest sensor data
   && Update running average
   (Possibility to request fahrenheit)
*/
float getTemperature(bool inFahrenheit) {
  float temperature = dht.readTemperature(inFahrenheit);                    // Fetch latest sensor data
  
  if(isnan(temperature)){                                                   // If sensor not working => alert user and return 0
    Serial.println("Temperature sensor malfunctioning");
    return 0;
  }
  
  float average = 0;
  
  if(inFahrenheit){                                                         // If Fahrenheit is requested
    for (int reading = 0; reading < RUNNING_AVG_READINGS - 1; reading++) {  // Shift values one place in running average array
      temp_f_readings[reading] = temp_f_readings[reading + 1];
    }
  
    temp_f_readings[RUNNING_AVG_READINGS - 1] = temperature;                // Insert new value
  
    float sum = 0;
    for (int reading = 0; reading < RUNNING_AVG_READINGS; reading++) {      // Calculate sum
      sum += temp_f_readings[reading];
    }
    
    average = sum / RUNNING_AVG_READINGS;// Calculate average
  } 
  else{                                                                     // If Celcius is requested
    for (int reading = 0; reading < RUNNING_AVG_READINGS - 1; reading++) {  // Shift values one place in running average array
      temp_c_readings[reading] = temp_c_readings[reading + 1];
    }
  
    temp_c_readings[RUNNING_AVG_READINGS - 1] = temperature;                // Insert new value
  
    float sum = 0;
    for (int reading = 0; reading < RUNNING_AVG_READINGS; reading++) {      // Calculate sum
      sum += temp_c_readings[reading];
    }
    
    average = sum / RUNNING_AVG_READINGS;                                   // Calculate average
  }
  
  return average;
}

/* Uses humidity and temerature to calculate the heat index => also know as the real feel */
float getHeatIndex(float temperature, float humidity, bool inFahrenheit) {
  return dht.computeHeatIndex(temperature, humidity, inFahrenheit);
}

/* Fetch latest sensor data
   && Update running average
*/
int getCO2(){
  int co2 = readCO2();                                                      // Fetch latest sensor data
  if(isnan(co2)){                                                           // If sensor not working => alert user and return 0
    Serial.println("CO2 sensor malfunctioning");
    return 0;
  }
  for (int reading = 0; reading < RUNNING_AVG_READINGS - 1; reading++) {    // Shift values one place in running average array
    co2_readings[reading] = co2_readings[reading + 1];
  }

  co2_readings[RUNNING_AVG_READINGS - 1] = co2;                             // Insert new value

  int sum = 0;
  for (int reading = 0; reading < RUNNING_AVG_READINGS; reading++) {        // Calculate sum
    sum += co2_readings[reading];
  }
  
  int average = sum / RUNNING_AVG_READINGS;                                 // Calculate average
  
  return average;
}

/* I2C call to CO2 sensor T6713 */
int readCO2(){
  int data [4];
  
  Wire.beginTransmission(ADDR_6713);                                                          // Start I2C
  Wire.write(0x04); Wire.write(0x13); Wire.write(0x8B); Wire.write(0x00); Wire.write(0x01);   // Send data
  Wire.endTransmission();                                                                     // End transmission
  
  delay(2000);                                                                                // Wait 2s while sensor is preparing data
  
  Wire.requestFrom(ADDR_6713, 4);                                                             // Request 4 bytes from T6713
  data[0] = Wire.read();                                                                      // Read Data
  data[1] = Wire.read();                                                                      // Read Data
  data[2] = Wire.read();                                                                      // Read Data
  data[3] = Wire.read();                                                                      // Read Data
  
  return ((data[2] * 0xFF ) + data[3]);                                                       // Generate current gas measurement in ppm using the acquired data
}
