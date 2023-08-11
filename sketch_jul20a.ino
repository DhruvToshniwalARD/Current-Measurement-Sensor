#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

int sensorPin = A0; // Default to A0

bool go = false;
bool go1 = true;
bool gotDur = false;
bool gotRate = false;
bool isStop = false;
bool sendData = false;

int sensorValue;
String x;
String sampleRate = "100"; // sample rate in ms (default 1ms)
unsigned long sTime;
String duration="1000";

// Define a struct for the data
struct Data {
  float current;
  float voltage;
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // for debugging
  Serial.begin(115200); // initialize serial communication

  while (!Serial) { delay(10); } // Wait until serial port is opened

  Serial.println("Adafruit INA260 Test");
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
}

void loop() {
  if(Serial.available()){
    x = Serial.readString();
    Serial.println("Received string: " + x);  // Debugging print
    if(x.charAt(0) == 's'){ // sample rate prefix
      sampleRate = x.substring(1); // get the rate in ms
      Serial.println("Received sample rate: " + sampleRate); // Debugging print
      gotRate = true; // key
    }
    if(x.charAt(0) == 'd'){ // sample duration prefix
      duration = x.substring(1); // get the duration in ms
      Serial.println("Received duration: " + duration); // Debugging print
      gotDur = true; // key
    }
    if(x.charAt(0) == 'p'){ // pin selection prefix
      String pinName = x.substring(1); // get the pin name
      if(pinName == "A0") sensorPin = A0;
      else if(pinName == "A1") sensorPin = A1;
      // Add more else-if clauses here for more pins
      Serial.println("Received pin: " + pinName); // Debugging print
    }
    x.trim();  // This removes any leading or trailing whitespace
    if(x == "Start"){
      Serial.println("Sending data");  // Debugging print
      sendData = true;  // Start sending data
    }
    if(x == "Stop"){
      sendData = false;  // Stop sending data
    }
  }

  if(gotDur && gotRate){ // if both the sample rate, durating have been recieved, start loop
    go = true;
    Serial.println("Received Both");
  }

  //if the current time minus the start time is larger then the sample duration:
  if((millis() - sTime) > (unsigned long)(duration.toInt())){
    if(!(isStop)){
      digitalWrite(LED_BUILTIN, LOW); // debugging
      Serial.println("Stop Milli"); // send stop message over serial port so that python script knows when to stop parsing
    }
    go = true; // stop sample loop
    gotDur = false; // lock 
    gotRate = false; // lock
    isStop = true; // so that it sends 'Stop' only one time
  }

  if(go && sendData){
    digitalWrite(LED_BUILTIN, HIGH);  // Debug purposes
    sensorValue = analogRead(sensorPin); // read the sensor

    if(go1){
      Serial.print(sensorValue); // send sensor value
      Serial.println(" " + String(sensorPin)); // tag
    }

    // Create an instance of the struct
    Data data;

    // Populate the struct with the current and voltage
    data.current = ina260.readCurrent();
    data.voltage = ina260.readBusVoltage();

    Serial.print(data.current);
    Serial.print(",");
    Serial.println(data.voltage);

    delay(sampleRate.toInt()); // delay by sample rate
  }
}
