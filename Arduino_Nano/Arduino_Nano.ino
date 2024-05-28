#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <SoftwareSerial.h> // Include the SoftwareSerial library

Adafruit_ADS1115 ads;
SoftwareSerial mySerial(10, 11); // RX, TX

#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

uint8_t sensor1[8] = { 0x28, 0x36, 0x2F, 0x06, 0x00, 0x00, 0x00, 0xDF };
uint8_t sensor2[8] = { 0x28, 0xAD, 0xA7, 0x07, 0x00, 0x00, 0x00, 0xEC };
uint8_t sensor3[8] = { 0x28, 0x7D, 0xA0, 0x06, 0x00, 0x00, 0x00, 0xF6 };

unsigned long period = 2000;
unsigned long last_time = 0;

float vout1 = 0.0;
float vout2 = 0.0;
float vin2 = 0.0;
float vin1 = 0.0;
float R1 = 10000.0;
float R2 = 1000.0;

int pins[5] = { 5, 8, 7, 9, 6 };

String sentString;
String tempString[2];
String voltage1;
String voltage2;

const float VCC = 5.1;  // supply voltage 5V or 3.3V. If using PCB, set to 5V only.
const int model = 4;    // enter the model (see below)

float cutOffLimit = 2.00;  // reading cutt off current. 1.00 is 1 Amper
float sensitivity[] = {
  40.0,  // for ACS758LCB-050B
  60.0,  // for ACS758LCB-050U
  20.0,  // for ACS758LCB-100B
  40.0,  // for ACS758LCB-100U
  13.3,  // for ACS758KCB-150B
  16.7,  // for ACS758KCB-150U
  10.0,  // for ACS758ECB-200B
  20.0,  // for ACS758ECB-200U
};

float quiescent_Output_voltage[] = {
  0.5,   // for ACS758LCB-050B
  0.12,  // for ACS758LCB-050U
  0.5,   // for ACS758LCB-100B
  0.12,  // for ACS758LCB-100U
  0.5,   // for ACS758KCB-150B
  0.12,  // for ACS758KCB-150U
  0.5,   // for ACS758ECB-200B
  0.12,  // for ACS758ECB-200U
};

const float FACTOR = sensitivity[model] / 1000;           // set sensitivity for selected model
const float QOV = quiescent_Output_voltage[model] * VCC;  // set quiescent Output voltage for selected model
float voltage;                                            // internal variable for voltage
float cutOff = FACTOR / cutOffLimit;


int fullfan = 0;
float tempAverage = 0.0;

const float maxVoltage = 29.4;
const float minVoltage = 21.0;

int percentage = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600); // Initialize software serial
  // ads.setGain(GAIN_ONE);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  for (int i = 0; i < 6; i++) {
    pinMode(pins[i], OUTPUT);
  }
  sensors.begin();
}

void loop() {
  // Serial Input Handling Task
  handleSerialInput();

  unsigned long current_time = millis();

  // Timing Task
  if (current_time - last_time > period) {
    last_time = current_time;
    readSensors();
    handleTemperatureControl();
  }
}

void readSensors() {
  sensors.requestTemperatures();
  float tempSum = 0.0;
  for (int i = 0; i < 3; i++) {
    tempString[i] = sensors.getTempCByIndex(i);
    tempSum += tempString[i].toFloat();  // Convert string to float before adding to sum
  }
  tempAverage = tempSum / 3.0;

  float vin_sum = 0.0;
  for (int i = 0; i < 10; i++) {
    float vin = (ads.computeVolts(ads.readADC_SingleEnded(2))) / (R2 / (R1 + R2));
    vin_sum += vin;
  }

  float averaged_vin = vin_sum / 10.0;

  averaged_vin = averaged_vin+1;

  float sum_voltage_raw = 0.0;
  for (int i = 0; i < 100; ++i) {
    sum_voltage_raw += ads.computeVolts(ads.readADC_SingleEnded(0));
  }
  // voltage = (sum_voltage_raw / 100.0) - QOV - 0.099;
  voltage = (sum_voltage_raw / 100.0) - QOV - 0.002;
  // if (averaged_vin > 28.5) {
    
  // } else {
  //   voltage = (sum_voltage_raw / 1000.0) - QOV - 0.007;
  // }


  float current = voltage / FACTOR;

  if (abs(voltage) < cutOff) {
    current = 0;
  }


  percentage = map(averaged_vin, minVoltage, maxVoltage, 0, 100);
  percentage = constrain(percentage, 0, 100);

  mySerial.print(tempAverage, 1);
  mySerial.print(",");
  mySerial.print(averaged_vin, 1);
  mySerial.print(",");
  mySerial.print(current);
  mySerial.print(",");
  mySerial.println(percentage);
  // mySerial.print(",");
  // mySerial.print(ads.computeVolts(ads.readADC_SingleEnded(1)));
  // mySerial.print(",");
  // mySerial.print(ads.readADC_SingleEnded(0));
  // mySerial.print(",");
  // mySerial.print(ads.readADC_SingleEnded(2));
  // mySerial.print(",");
  // mySerial.println(ads.computeVolts(ads.readADC_SingleEnded(2)));
  Serial.print("volt ");
  Serial.println(voltage,4);
  Serial.print("current ");
  Serial.println(ads.computeVolts(ads.readADC_SingleEnded(0)));
  Serial.print("volt2 ");
  Serial.println(ads.computeVolts(ads.readADC_SingleEnded(2)));
  Serial.println("-------------------------------------------------------");
}

void handleTemperatureControl() {
  if (fullfan == 0) {
    if (tempAverage < 35) {
      analogWrite(pins[4], LOW);
    }
    else{
      analogWrite(pins[4], HIGH);
    }
    
  }
}


void handleSerialInput() {
  while (mySerial.available()) { // Changed from Serial.available() to mySerial.available()
    char text = mySerial.read(); // Changed from Serial.read() to mySerial.read()
    switch (text) {
      case 'a':
        digitalWrite(pins[0], LOW);
        mySerial.write("turnoffd\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'b':
        digitalWrite(pins[0], HIGH);
        mySerial.write("turnond\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'e':
        digitalWrite(pins[1], LOW);
        mySerial.write("turnoffa\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'f':
        digitalWrite(pins[1], HIGH);
        mySerial.write("turnona\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'i':
        digitalWrite(pins[2], LOW);
        mySerial.write("turnoffc\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'j':
        digitalWrite(pins[2], HIGH);
        mySerial.write("turnonc\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'm':
        digitalWrite(pins[3], LOW);
        mySerial.write("turnoffi\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'n':
        digitalWrite(pins[3], HIGH);
        mySerial.write("turnoni\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'x':
        analogWrite(pins[4], LOW);
        fullfan = 0;
        mySerial.write("turnofff\n"); // Changed from Serial.write() to mySerial.write()
        break;
      case 'z':
        analogWrite(pins[4], HIGH);
        fullfan = 1;
        mySerial.write("turnonf\n"); // Changed from Serial.write() to mySerial.write()
        break;
      default:
        break;
    }
  }
}
