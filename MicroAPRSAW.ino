/*
 http://www.aprs-dl.de/?Aktuell
 https://github.com/markqvist/MicroAPRS
 */

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "dht11.h"
dht11 DSKY;

TinyGPSPlus gps;
SoftwareSerial ssGPS(4, 10); // RX TX

char *icon ="s";
char logString[130];
char * packet_buffer  = "";
int a=0;
float flat_send=52.0;
float flon_send=13.0;
float dist_send=10.0;
long zeit_send,zeit_send_vergangen;
int humi=0;
int tempc=0;
boolean firstfix=false;
int battVolts,LiPoVolts;

void setup()
{
  char* msg="ArduinoMicroAPRS MicroAPRS-SerialMode v141113";
  delay(100);
  Serial.begin(9600);
  ssGPS.begin(9600);
  Serial.println(msg);
}

void loop()
{
  int chk=1;
  if(chk = DSKY.read(5) == DHTLIB_OK)
  {
    //    Serial.print(DSKY.humidity,1),Serial.print(" ");
    //    Serial.println(DSKY.temperature,1);
    humi=DSKY.humidity;
    tempc=DSKY.temperature;
  }
  else
  {
    humi=0;
    tempc=0;
  }
  battVolts=getBandgap();  //Determins what actual Vcc is, (X 100), based on known bandgap voltage
  //connect VCC<->VREF an VCC from LiPo to A1
  // attention VCC from LiPo must < VCC and max 5V
  LiPoVolts=(map(analogRead(1), 0, 1023, 0, battVolts));

  if(LiPoVolts < 307)
  {
    //buzzer alarm 
  }
  
  float flat=0, flon=0;
  smartdelay(5000);

  if ((!gps.location.isValid()))
  {
  }
  else
  {
    flat = gps.location.lat();
    flon = gps.location.lng();
    int gpsdeg = (int)flat;
    float gpsmind = ((flat - (gpsdeg*1.0L))*60.0L);
    int gpsmin = (int) gpsmind;
    int gpss =  (int) ((gpsmind-(gpsmin*1.0L))*100.0L);  

    int gpsldeg = (int)flon;
    float gpslmind = ((flon - (gpsldeg*1.0L))*60.0L);
    int gpslmin = (int) gpslmind;
    int gpsls = (int) ((gpslmind-(gpslmin*1.0L))*100.0L);  

    if(firstfix == false)
    {
      flat_send = flat;
      flon_send = flon;
      firstfix = true;
      zeit_send = gps.time.value();
      sprintf(logString,"Erster FIX:%03d Zeit bis TX:%ds",flat_send,flon_send,zeit_send);
      //      Serial.println(logString);      
      return;
    }

    dist_send=gps.distanceBetween(flat,flon,flat_send,flon_send);
    //Serial.println(dist_send,6);

    zeit_send_vergangen=(int)(gps.time.value()-zeit_send)/100;
    //distan > 1Km or 2min time elabsed
    if(((int)dist_send > 1000) || (zeit_send_vergangen > 120))
    {
      flat_send=flat;
      flon_send=flon;
      zeit_send=gps.time.value(); //ms Raw

      sprintf(logString,"!=%02d%02d.%02dN/%03d%02d.%02dE%s/%03d/%03d/A=%06d/C=%02d/H=%02d/VCC=%03d/LiPo=%03d"
        ,gpsdeg,gpsmin,gpss,gpsldeg,gpslmin,gpsls
        ,icon
        ,(int)gps.course.deg(),(int)gps.speed.kmph(),(int)gps.altitude.meters()
        ,tempc,humi
        ,battVolts
        ,LiPoVolts);

      for (int repeat=0;repeat < 3;repeat++)
      {
        Serial.println(logString);
        delay(2000);
      }      
      ssGPS.flush();
    }
    else
    {
    }
  }
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do{
    while (ssGPS.available())
      gps.encode(ssGPS.read());
  } 
  while (millis() - start < ms);
}

int getBandgap(void) // Returns actual value of Vcc (x 100)
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // For mega boards
  const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

#else
  // For 168/328 boards
  const long InternalReferenceVoltage = 1071L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

#endif
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion  
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
  // Scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value 
  return results;

}




