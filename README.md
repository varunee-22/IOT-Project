# IOT-Project
#Transmitter Code
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

GPIO.setmode(GPIO.BCM)

pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
radio.setPayloadSize(32)
radio.setChannel(0x60)

radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

# radio.openReadingPipe(1, pipes[1])
radio.openWritingPipe(pipes[1])
radio.printDetails()

# radio.startListening()
# message = list(input("Enter a message to send: "))
while(1):
    message = list("Hello World is awesome")
    radio.write(message)
    print("We sent the message of {}".format(message))

    # Check if it returned ackPL
    if radio.isAckPayloadAvailable():
        returnedPL = []
        radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
    else:
        print("No payload received")
    time.sleep(1)
    
    
   #Reciever code
   import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

GPIO.setmode(GPIO.BCM)

pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
radio.setPayloadSize(32)
radio.setChannel(0x60)

radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(0, pipes[1])
radio.printDetails()

radio.startListening()

while(1):
    ackPL = [1]
    while not radio.available(0):
        time.sleep(1 / 100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())
    print("Received: {}".format(receivedMessage))

    print("Translating the receivedMessage into unicode characters")
    string = ""
    for n in receivedMessage:
        # Decode into standard unicode set
        if (n &gt;= 32 and n &lt;= 126):
            string += chr(n)
    print(string)
    radio.writeAckPayload(1, ackPL, len(ackPL))
    print("Loaded payload reply of {}".format(ackPL))
    
    
   #ESP8266 and GPS code
   #include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

static const int RXPin = 4, TXPin = 5;   // GPIO 4=D2(conneect Tx of GPS) and GPIO 5=D1(Connect Rx of GPS
static const uint32_t GPSBaud = 9600; //if Baud rate 9600 didn't work in your case then use 4800

TinyGPSPlus gps; // The TinyGPS++ object
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget

SoftwareSerial ss(RXPin, TXPin);  // The serial connection to the GPS device

SimpleTimer timer;

float spd;       //Variable  to store the speed
float sats;      //Variable to store no. of satellites response
String bearing;  //Variable to store orientation or direction of GPS

char auth[] = "BkR9rl1XVj-iO7PEAxv-gKkVeuwUtUfU";              //Your Project authentication key
char ssid[] = "acharya";                                       // Name of your network (HotSpot or Router name)
char pass[] = "Acharyaji";                                      // Corresponding Password

//unsigned int move_index;         // moving index, to be used later
unsigned int move_index = 1;       // fixed location for now
  

void setup()
{
  Serial.begin(115200);
  Serial.println();
  ss.begin(GPSBaud);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
}

void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
      Blynk.virtualWrite(V4, "GPS ERROR");  // Value Display widget  on V4 if GPS not detected
  }
}

void loop()
{
    while (ss.available() > 0) 
    {
      // sketch displays information every time a new sentence is correctly encoded.
      if (gps.encode(ss.read()))
        displayInfo();
  }
  Blynk.run();
  timer.run();
}

void displayInfo()
{ 
  if (gps.location.isValid() ) 
  {    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 
    float longitude = (gps.location.lng()); 
    
    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V1, String(latitude, 6));   
    Blynk.virtualWrite(V2, String(longitude, 6));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();               //get speed
       Blynk.virtualWrite(V3, spd);
       
       sats = gps.satellites.value();    //get number of satellites
       Blynk.virtualWrite(V4, sats);

       bearing = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
       Blynk.virtualWrite(V5, bearing);                   
  }
  
 Serial.println();
}
