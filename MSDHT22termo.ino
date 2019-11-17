
// Enable debug prints
#define MY_DEBUG

#define MY_RADIO_RF24
#define CHILD_ID_HVAC 0
#define MY_NODE_ID 10

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MySensors.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Bounce2.h>
#include <DaikinHeatpumpIR.h>

//screen data
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 7
#define DHTTYPE DHT22

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Set this offset if the sensor has a permanent small offset to the real temperatures.
// In Celsius degrees (as measured by the device)
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
int UPDATE_INTERVAL = 1000;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_THERMO 2

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

float temperature = 0.0;
float humidity = 0.0;
float thermoval = 18.0;

const int buttonPlus = 4;
const int buttonMoins = 5;

unsigned int cpt = 0;
unsigned int cpt_screen = 0;
boolean screenOff = false;

boolean tictac = false;

// Instantiate a Bounce object
Bounce debouncerPlus = Bounce();
Bounce debouncerMoins = Bounce();

int valuePreviousPlus = LOW;
int valuePreviousMoins = LOW;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgTermo(CHILD_ID_THERMO, V_TEMP);

DHT_Unified dht(DHT_DATA_PIN, DHTTYPE);

sensors_event_t event;

//pomp à chaleur
//Some global variables to hold the states
int POWER_STATE;
float TEMP_STATE;
int FAN_STATE;
int MODE_STATE;
int VDIR_STATE;
int HDIR_STATE;

IRSenderPWM irSender(3); // IR led on Arduino digital pin 3, using Arduino PWM

//Change to your Heatpump
HeatpumpIR *heatpumpIR = new DaikinHeatpumpIR();

MyMessage msgHVACSetPointC(CHILD_ID_HVAC, V_HVAC_SETPOINT_COOL);
MyMessage msgHVACSpeed(CHILD_ID_HVAC, V_HVAC_SPEED);
MyMessage msgHVACFlowState(CHILD_ID_HVAC, V_HVAC_FLOW_STATE);

bool initialValueSent = false;

void presentation()
{
  // Send the sketch version information to the gateway
  sendSketchInfo("TemHumTerm", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_THERMO, S_TEMP);

  metric = getControllerConfig().isMetric;
}

void setup()
{
  Serial.begin(115200);

  dht.begin(); // set data pin of DHT sensor

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.display();

  // Setup the first button with an internal pull-up :
  pinMode(buttonPlus, INPUT);
  // After setting up the button, setup the Bounce instance :
  debouncerPlus.attach(buttonPlus);
  debouncerMoins.interval(5); // interval in ms

  // Setup the second button with an internal pull-up :
  pinMode(buttonMoins, INPUT);
  // After setting up the button, setup the Bounce instance :
  debouncerMoins.attach(buttonMoins);
  debouncerMoins.interval(5); // interval in ms
}

void loop()
{
  // Update the Bounce instances :
  debouncerPlus.update();
  debouncerMoins.update();

  // Get the updated value :
  int valuePlus = debouncerPlus.read();
  int valueMoins = debouncerMoins.read();

  if ((valuePlus == HIGH) && (valuePreviousPlus == LOW))
  {
    thermoval += .5;
    cpt_screen = 0;
    displayValues();
  }
  if ((valueMoins == HIGH) && (valuePreviousMoins == LOW))
  {
    thermoval -= .5;
    cpt_screen = 0;
    displayValues();
  }

  valuePreviousPlus = valuePlus;
  valuePreviousMoins = valueMoins;

  wait(5);
  cpt+=5;
  cpt_screen +=5;

  if ((cpt % 10000) == 0)
  {
    readDHT22();

    //sendValues();

    // Sleep for a while to save energy
    //sleep(UPDATE_INTERVAL);
    //dht.begin();
  }
  if(cpt_screen > 10000){
    if(!screenOff)
      switchOffscreen();
  }
}

void readDHT22()
{
  dht.temperature().getEvent(&event);

  // Get temperature from DHT library
  temperature = event.temperature;
  if (isnan(temperature))
  {
    Serial.println("Failed reading temperature from DHT!");
  }
  else
  {
    Serial.print("T: ");
    Serial.println(temperature);
  }

  // Get humidity from DHT library
  dht.humidity().getEvent(&event);

  humidity = event.relative_humidity;
  if (isnan(humidity))
  {
    Serial.println("Failed reading humidity from DHT");
  }
  else
  {
    Serial.print("H: ");
    Serial.println(humidity);
  }

  //thermostat
}

void sendValues()
{

  send(msgTemp.set(temperature, 1));
  send(msgHum.set(humidity, 1));
  send(msgTermo.set(thermoval, 1));
}

void displayValues(void)
{
  display.clearDisplay();

  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.println(F("Consigne :"));

  display.setTextSize(2); // Draw 2X-scale text
  display.print("    ");
  display.println(thermoval);

  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  if (tictac)
  {
    display.print(F("Mesure : "));
    tictac = false;
  }
  else
  {
    display.print(F("Mesure.: "));
    tictac = true;
  }
  display.println(temperature);

  display.display();
  
  screenOff= false;
  //delay(2000);
}

void switchOffscreen(){
  display.clearDisplay();
  display.display();
  screenOff= true;
}