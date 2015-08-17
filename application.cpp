#include "application.h"
#include "OneWire.h"
#include "DS18B20.h"
#include "PID.h"

/* Function prototypes -------------------------------------------------------*/
int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);

SYSTEM_MODE(MANUAL);

TCPServer server = TCPServer(23);
TCPClient client;
UDP udp;

// Loop vars
unsigned long last_temp_time = 0;
unsigned long last_temp_request_time = 0;
bool requesting_temp = false;
unsigned long last_send_time = 0;
float temp;
String buffer;

DS18B20 ds18b20 = DS18B20(D6);

double Setpoint=152, Input, Output;
int Kp=2, Ki=5, Kd=1;

PID Pid1(&Input, &Output, &Setpoint, Kp, Ki, Kd, PID_DIRECT);

String hostIP = "192.168.1.106";

void ipArrayFromString(byte ipArray[], String ipString) {
  int dot1 = ipString.indexOf('.');
  ipArray[0] = ipString.substring(0, dot1).toInt();
  int dot2 = ipString.indexOf('.', dot1 + 1);
  ipArray[1] = ipString.substring(dot1 + 1, dot2).toInt();
  dot1 = ipString.indexOf('.', dot2 + 1);
  ipArray[2] = ipString.substring(dot2 + 1, dot1).toInt();
  ipArray[3] = ipString.substring(dot1 + 1).toInt();
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
    byte lowByte = ((p_value >> 0) & 0xFF);
    byte highByte = ((p_value >> 8) & 0xFF);

    EEPROM.write(p_address, lowByte);
    EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
    byte lowByte = EEPROM.read(p_address);
    byte highByte = EEPROM.read(p_address + 1);

    return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}


void loadValues() {
    Setpoint = EEPROMReadInt(0);
    Kp = EEPROMReadInt(2);
    Ki = EEPROMReadInt(4);
    Kd = EEPROMReadInt(6);
}

void writeValues() {
    EEPROMWriteInt(0, Setpoint);
    EEPROMWriteInt(2, Kp);
    EEPROMWriteInt(4, Ki);
    EEPROMWriteInt(6, Kd);
}


bool connectToMyServer(String ip) {
  byte serverAddress[4];
  ipArrayFromString(serverAddress, ip);

  client.connect(serverAddress, 7645);
  client.print("device:" + Spark.deviceID() + "\n");
}

bool search_for_ds_onewire() {
  if(!ds18b20.search()){
      Serial.println("No more addresses.");
      Serial.println();
      ds18b20.resetsearch();

      return false;
  } else {
    return true;
  }
}

float get_temp() {
  return ds18b20.getTemperature();

}

void setup() {
  // Spark.function("connect", connectToMyServer);
  Serial.begin(57600);
  Serial.println("Starting");
  WiFi.on();
  Serial.println("wifi on");
  Serial.println("connecting");
  WiFi.connect();
  while( !WiFi.ready() ) {
      Serial.println("Waiting for wifi to be ready");
      Serial.println(WiFi.hasCredentials());
      Serial.println(WiFi.connecting());
      delay(1000);
  }

  for (int pin = D0; pin <= D7; ++pin) {
    pinMode(pin, OUTPUT);
  }

  Serial.println("Searching for Dallas 1-wire probe");
  while(!search_for_ds_onewire()) {
    delay(250);
  }
  Serial.println("Probe found!");
  loadValues();
  Pid1.SetMode(PID_AUTOMATIC);

}

int rest_to_int(String line, int pos) {
    String rest = line.substring(pos);
    return rest.toInt();
}

void process_line(String line) {
   if (line.startsWith("set:pid_1:Kp:")) {
       Kp = rest_to_int(line, 13);
   } else if (line.startsWith("set:pid_1:Ki:")) {
       Ki = rest_to_int(line, 13);
   } else if(line.startsWith("set:pid_1:Kd:")) {
       Kd = rest_to_int(line, 13);
   } else if(line.startsWith("set:pid_1:Setpoint:")) {
       Setpoint = rest_to_int(line, 19);
   } else {
       Serial.println("Couldn't handle line:");
       Serial.println(line);
   }
   writeValues();
}


void loop() {
  unsigned long time = millis();
  //Serial.println("loop");
  if (requesting_temp) {
      if (time > last_temp_time +760) {
          temp = ds18b20.retrieveTemperature();
          if ( temp > 150 || temp < -50 ) {
              Serial.println("Weird temp");
              Serial.println(temp);
              return;
          }
          Input = temp;
          Pid1.Compute();
          Serial.println(temp);
          Serial.println(client.connected());
          last_temp_time = time;
          requesting_temp = false;

          if (client.connected()) {
            Serial.println("sending");
            client.print("settable:pid_1:Kp:");
            client.print(Kp);
            client.print("\nsettable:pid_1:Ki:");
            client.print(Ki);
            client.print("\nsettable:pid_1:Kd:");
            client.print(Kd);
            client.print("\nsettable:pid_1:Setpoint:");
            client.print(Setpoint);
            client.print("\ntemperature (c):thermoprobe:");
            client.print(temp);
            client.print("\npid pwm:pid1:");
            client.print(Output);
            client.print("\n");
          } else {
            connectToMyServer(hostIP);
          }

      }
    } else {
        ds18b20.requestTemperature();
        requesting_temp = true;
        last_temp_request_time = time;
  }
  if (client.available()) {
      char c = client.read();
      buffer += c;
      Serial.print(c);
      if (c == '\n') {
          process_line(buffer);
          buffer = "";
      Serial.print(buffer);
      }
  }
  analogWrite(D0, Output);
}



