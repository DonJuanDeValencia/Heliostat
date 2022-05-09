// Der Schrittmotor macht 2048 Impulse / Umdrehung
// Zahnrad auf Schrittmotor 24 Zähne
// Zahnrad auf Dreh 64 Zähne
// Zahnrad auf Schwenk 72 oder 73

#include <AccelStepper.h>
#include "RTClib.h"
RTC_DS1307 rtc;
// Define a StepperDreh and the pins it will use
AccelStepper StepperDreh(AccelStepper::FULL4WIRE, 2, 4, 3, 5); // Defaults to AccelStepperDreh::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper StepperSchwenk(AccelStepper::FULL4WIRE, 6, 8, 7, 9); // Defaults to AccelStepperDreh::FULL4WIRE (4 pins) on 2, 3, 4, 5
 
const int RefPinDrehen = 11;     // the number of the RefSwitch pin   Links Rechts
const int RefPinSchwenken = 10;     // the number of the RefSwitch pin   Rauf Runter
const int PinSunSensor = A0;      // Sensor if the sun is Shining
const int StepperInc = 2048;
const int StepperZahnrad = 24;
const int DrehZahnRad = 64;
const int SchwenkZahnrad = 73;

const float LATITUDE = 52.399552;     // Breitengrad von Brandenburg Gate 52.399552
const float LONGITUDE = 13.048008;    // Längengrad von Brandenburg Gate 13.048008


const float NORDOFFSET = 151;      // 0° = Norden Abweichung Null Referenzschalter  zu Norden
const float WINKELOFFSET = 56;  // Abweichung Schwenken  90° soll Waagerecht nach oben sein ( Platte Waagerecht ) 57
const float MAXSCHWENKINC = 2600; // 2600;  // Maximale Inc fürs Schwenken

const float WINKELVERTICALSOLL = 10;  // Da wo der Lichtpunkt hinscheinen soll  166 um 16:00   171 um 10:00  170
const float WINKELVERTICALKORREKTUR = 6; // Nachmittags muß der VerticalWinkel um so viel kleiner sein, warum auch immer 

const float WINKELHORIZONTALSOLL = 18; // Da wo der Lichtpunkt hinscheinen soll 193

const float MAXVISIBLESUN = 278; // Max Azimut wo die Sonne noch zu sehen ist ( hintern Haus )
const float MINVISIBLESUN = 100; // Min Azimut ab der die Sonne zu sehen ist ( hintern Haus )
const float MINVISIBLESUNHOEHE = 3.85; // Min Hoehe ab der die Sonne zu sehen ist ( hintern Haus )

const float PARKAZIMUT = 5;  // Park Azimut wenn Sonne nicht sichtbar
const float PARKHOEHE = 0;  // Park Höhe wenn Sonne nicht sichtbar


bool RefDrehDone;
bool RefSchwenkDone;
bool PosDOk;
bool PosSOk;
bool FirstTime;
bool MotorNotOn;


int RefDrehStep;  
int RefSchwenkStep;
int MainStep;
int Year;
int Month;
int Day;  
int Stunde;
int Minute;
int Second;
int Secondalt;
int NoSun;
int Sun;

float Hoehe;
float Azimut;
float NewHoehe;
float NewAzimut;
  
void setup()
{  
  // Change these to suit your StepperDreh if you want
  Serial.begin(115200);
  StepperDreh.setMaxSpeed(100);
  StepperDreh.setSpeed(100);
  StepperDreh.setAcceleration(40);
  StepperSchwenk.setMaxSpeed(100);
  StepperSchwenk.setSpeed(100);
  StepperSchwenk.setAcceleration(40);

  
  pinMode(RefPinDrehen, INPUT);
  pinMode(RefPinSchwenken, INPUT);
  FirstTime = true;
  pinMode(LED_BUILTIN, OUTPUT); 
  if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
   }
  //SetTime();
}

void SetTime() {
    rtc.adjust(DateTime(2022,3,16,10,41,00));   // Set to Winter Time !!!!
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
} 

void ReferenceDreh() {

int RefState;
      RefState = digitalRead(RefPinDrehen); 
      switch (RefDrehStep) {
        case 0:
            if (RefState == HIGH) {           // Ref Schalter ist betätigt -> ertmal wegfahren
                StepperDreh.setSpeed(500);
            }  else {
              RefDrehStep = RefDrehStep + 1;
            }
            break;
        case 1:                             // Ref Schalter ist nicht betätigt -> zum RefSchalter fahren
             StepperDreh.setSpeed(-500);
             if (RefState == HIGH) {
              RefDrehStep = RefDrehStep +1;
             }
             break;
         case 2:                            // RefSchalter ist wieder betätigt -> Wegfahren
             StepperDreh.setSpeed(500);
              RefDrehStep = RefDrehStep +1;
              break;
         case 3:
             if (RefState == LOW) {
              StepperDreh.setSpeed(0);
              StepperDreh.setCurrentPosition(0);            
              RefDrehStep = RefDrehStep +1;
              RefDrehDone = true;
             }
             break;
          }
}

void ReferenceSchwenk() {
 int RefState;
      RefState = digitalRead(RefPinSchwenken); 
      switch (RefSchwenkStep) {
        case 0:
            if (RefState == HIGH) {           // Ref Schalter ist betätigt -> ertmal wegfahren
                StepperSchwenk.setSpeed(500);
            }  else {
              RefSchwenkStep = RefSchwenkStep + 1;
            }
            break;
        case 1:                             // Ref Schalter ist nicht betätigt -> zum RefSchalter fahren
             StepperSchwenk.setSpeed(-500);
             if (RefState == HIGH) {
              RefSchwenkStep = RefSchwenkStep +1;
             }
             break;
         case 2:                            // RefSchalter ist wieder betätigt -> Wegfahren
             StepperSchwenk.setSpeed(500);
             RefSchwenkStep = RefSchwenkStep +1;
             break;
        case 3 :
             if (RefState == LOW) {           // Refschalter wieder frei -> Stop und actualIncremente auf Null stetzen
              StepperSchwenk.setSpeed(0);
              delay(100);
              StepperSchwenk.setCurrentPosition(0);     
              RefSchwenkStep = RefSchwenkStep +1;
              RefSchwenkDone = true;
             }
             break;
          }
}

void moveDreh(float NewPos ) {

  NewPos = NewPos - NORDOFFSET ;

int  NewStep = NewPos/360*  StepperInc*DrehZahnRad/StepperZahnrad;
 
  if (StepperDreh.currentPosition() < NewStep) {
        StepperDreh.setSpeed(500);
        PosDOk = false; }
  else { 
        StepperDreh.setSpeed(-500);
        PosDOk = false; }
  if (StepperDreh.currentPosition() == NewStep ) { 
    StepperDreh.setSpeed(0);
    PosDOk = true;
  }
}

void moveSchwenk(float NewPos ) {
  NewPos = abs(NewPos - 90 - WINKELOFFSET);
int  NewStep = NewPos/360*  StepperInc*SchwenkZahnrad/StepperZahnrad;
  if (NewStep < 0 ) NewStep = 0;
  if (NewStep > MAXSCHWENKINC) NewStep = MAXSCHWENKINC;
  if (StepperSchwenk.currentPosition() < NewStep) { 
        StepperSchwenk.setSpeed(500);
        PosSOk = false; }
  else {  StepperSchwenk.setSpeed(-500);
            PosSOk = false; }
  if (StepperSchwenk.currentPosition() == NewStep ) { 
        StepperSchwenk.setSpeed(0);
        PosSOk = true;
  }
}

void Sonnenstand(float Monat, float Tag, float Stunde, float Minute, float &Hoehe, float &Azimut) {  // Quelle http://www.geoastro.de/SME/tk/index.htm
float Tageszahl;
float deklin;
float k = PI / 180;
float zeitgleichung;
float stundenwinkel;
float x;
float y;
float a;
float b;

  Tageszahl = (Monat-1)*30.3 + Tag;
  deklin = -23.45 * cos(k*360*(Tageszahl+10)/365);
  zeitgleichung = 60 * (-0.171*sin(0.0337*Tageszahl + 0.465 ) - 0.1299 * sin ( 0.01787 * Tageszahl - 0.168 ));
  stundenwinkel = 15*(Stunde + Minute/60 - (15.0-LONGITUDE)/15.0 - 12 + zeitgleichung/60 );
  x = sin(k*LATITUDE)*sin(k*deklin)+cos(k*LATITUDE)*cos(k*deklin)*cos(k*stundenwinkel);
  Hoehe = asin(x)/k;
  y = -(sin(k*LATITUDE)*sin(k*Hoehe)-sin(k*deklin))/(cos(k*LATITUDE)*sin(acos(sin(k*Hoehe))));
  a = Stunde + Minute/60;
  b = 12 + (15-LONGITUDE)/15 - zeitgleichung/60;
  if ( a<=b ) Azimut = acos(y)/k;
  else Azimut = 360 - acos(y)/k;
}

void getTime(){
      DateTime now = rtc.now();
//       Serial.print(now.year(), DEC);
//       Serial.print('/');
//       Serial.print(now.month(), DEC);
//       Serial.print('/');
//       Serial.print(now.day(), DEC);
//       Serial.print('/')<
//       Serial.print(now.hour(), DEC);
//       Serial.print(':');
//       Serial.print(now.minute(), DEC);
//       Serial.print(':');
//       Serial.print(now.second(), DEC);
//       Serial.println();
      Year = now.year();
      Month = now.month();
      Day = now.day();
      Stunde = now.hour();
      Minute = now.minute();
      Second = now.second();
       Serial.print(Year);
       Serial.print('/');  
       Serial.print(Month);
       Serial.print('/');
       Serial.print(Day);
       Serial.print('=');
       Serial.print(Stunde);
       Serial.print(':');
       Serial.print(Minute);
       Serial.print(':');
       Serial.print(Second);
       Serial.println();
}
void MotorOff() {
    StepperSchwenk.disableOutputs();
    StepperDreh.disableOutputs();
    MotorNotOn = true;
}

void MotorOn() {
    StepperSchwenk.enableOutputs();
    StepperDreh.enableOutputs();
    MotorNotOn = false;
}
void loop()
{
  
  float Zeitkorrektur;
  float WinkelVertiKorr;
  switch (MainStep) {
        case 0:
                if (!RefSchwenkDone)  ReferenceSchwenk();
                if (!RefDrehDone)  ReferenceDreh();
                StepperSchwenk.runSpeed();      
                StepperDreh.runSpeed();
                if ( RefSchwenkDone and RefDrehDone ) { 
                      MotorOff();
                      MainStep = MainStep + 1 ;
                      //MainStep = 99;
                }
                break;
         case 1:
                if (!FirstTime ) delay(60000);  // alle 1 min 1 * 60 * 1000 
                FirstTime = false;
                getTime();  
                Sonnenstand(Month,Day,Stunde,Minute,Hoehe,Azimut);     // Monat , Day, Stunde , Minute
                MainStep = MainStep + 1 ;
                Sun = analogRead(PinSunSensor);  // To see if the sun is shining it should only move when the sun is there, but it is ot working korrekt so at the moment it is 
               
                break;
         case 2:
                //Zeitkorrektur = ( float) ( Stunde - 10 ) / 6 ;  // 10h -> 0 16h -> 1
                //Zeitkorrektur = constrain(Zeitkorrektur,0,1);
                //WinkelVertiKorr = WINKELVERTICALSOLL - ( Zeitkorrektur * WINKELVERTICALKORREKTUR );
                FindHeliostatAltAndAz(Hoehe, Azimut, WINKELVERTICALSOLL, WINKELHORIZONTALSOLL, NewHoehe, NewAzimut);
                 
                Serial.print("HoeheSonne = "); Serial.println(Hoehe);
                Serial.print("AzimutSonne = "); Serial.println(Azimut);
                //Serial.print("Sun = "); Serial.print(Sun);
                //Serial.print("    Min without Sun = "); Serial.println(NoSun);
                Serial.print("AzimutSpiegel = "); Serial.println(NewAzimut);
                Serial.print("HoeheSpiegel = "); Serial.println(NewHoehe);
                MainStep = MainStep + 1;
                break;
          case 3:
                //Zeitkorrektur = ( float) ( Stunde - 10 ) / 6 ;  // 10h -> 0 16h -> 1
                //Zeitkorrektur = constrain(Zeitkorrektur,0,1);
                //WinkelVertiKorr = WINKELVERTICALSOLL - ( Zeitkorrektur * WINKELVERTICALKORREKTUR );
               //FindHeliostatAltAndAz(Hoehe, Azimut, WINKELVERTICALSOLL, WINKELHORIZONTALSOLL, NewHoehe, NewAzimut);
                    
                if ( ( Azimut < MAXVISIBLESUN ) and (Azimut > MINVISIBLESUN ) and ( Hoehe > MINVISIBLESUNHOEHE)  ) {
                          moveDreh ( NewAzimut  );  // 180° Süden 90° Osten 270° Westen
                          moveSchwenk ( NewHoehe)  ; // 90° Senkrecht nach oben  0° zum Horizont
                } else {
                           moveDreh ( PARKAZIMUT  );  // 180° Süden      90° Osten         270° Westen
                           moveSchwenk ( PARKHOEHE)  ; // 90° Senkrecht nach oben  0° zum Horizont
                }
                
                if (PosDOk and PosSOk) {
                        MainStep = 1 ; 
                        delay(1000);
                        MotorOff();     
                } else {
                        if (MotorNotOn) {
                             MotorOn();
                             delay(1000);
                        }
                        StepperSchwenk.runSpeed();      
                        StepperDreh.runSpeed();
                }
                break;
          case 99:
                moveDreh ( 180  );  // 180° Süden      90° Osten         270° Westen
                moveSchwenk ( 90)  ; // 90° Senkrecht nach oben  0° zum Horizont
                if (PosDOk and PosSOk) {
                       
                        delay(1000);
                        MotorOff();     
                } else {
                        if (MotorNotOn) {
                             MotorOn();
                             delay(1000);
                        }
                        StepperSchwenk.runSpeed();      
                        StepperDreh.runSpeed();
                }
                break;   
          }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This code calculates the angles for the heliostat (returnaltaz = 1 will return alt, 2 returns az)
// got from https://www.cerebralmeltdown.com/

void FindHeliostatAltAndAz(float SunsAltitude, float SunsAzimuth, float targetalt, float targetaz, float &machinealt, float &machineaz){

  float x,y,z,z1,z2,x1,x2,y1,y2,hyp,dist;
  
  z1 = sin(to_rad(SunsAltitude));
  hyp = cos(to_rad(SunsAltitude));
  x1 = hyp*cos(to_rad(SunsAzimuth));
  y1 = hyp*sin(to_rad(SunsAzimuth));

  z2 = sin(to_rad(targetalt));
  hyp = cos(to_rad(targetalt));
  x2 = hyp*cos(to_rad(targetaz));
  y2 = hyp*sin(to_rad(targetaz));  
  
  x=(x1-x2)/2+x2;
  y=(y1-y2)/2+y2;
  z=(z1-z2)/2+z2;

  
  
  dist=sqrt(x*x+y*y+z*z);
  if ((dist>-0.0001) && (dist <0.0001)){
  dist=0.0001;
  }
  Serial.print("x = "); Serial.println(x);
  Serial.print("y = "); Serial.println(y);
  Serial.print("z = "); Serial.println(z);
  Serial.print("dist = "); Serial.println(dist);
  
  machinealt=to_deg(asin(z/dist));
  float quotient = y / x;
  Serial.print("quotient = "); Serial.println(quotient);
  machineaz=to_deg(atan(quotient));
  Serial.print("az = "); Serial.println(machineaz);
  if ( machineaz < 0 )   machineaz = machineaz + 360;
Serial.print("az = "); Serial.println(machineaz);
}

float to_rad(float angle){
return angle*(PI/180);
}
float to_deg(float angle){
return angle*(180/PI);
}
