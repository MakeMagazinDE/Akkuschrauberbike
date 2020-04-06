// ----------------------------------------------------------------------------
//
//    E-Bike mit Akkuschrauber						V1.31
//
//    Copyright 2020 by Christian Koubek
//
// 	  This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// ----------------------------------------------------------------------------
//
// Hardware: 		Arduino Leonardo
//
// Peripherie:		PWM-Ausgang Akku-Schrauber mit Power-MOS-Fet
//					PWW-Ausgang Licht vorne über Transistor
//					PWM-Ausgang Licht hinten über Transistor
//
//					Analogeingang Akku-Spannung über Spannungsteiler
//					Analogeingang Motor-Strom über Shunt-Leitung
//					Analogeingang Helligkeit über LDR
//					Analogeingang Tasten (2 Stück, Erkennung über Spannungsteiler)
//
//					2x16 Zeichen LCD über DIOs
//
// Hinweis: In der Arduino IDE unter Ubuntu wird der Arduino Leonardo mit dem Port /dev/ttyACM0 angezeigt.
//

// include für das LCD
#include <LiquidCrystal.h>

//Verschiedene Defines zum Debugen und Testen der Software und des Rads
//#define  DEBUGSERIAL                      //Aktivierung der seriellen Schnittstelle
//#define DEBUGBRAKELIGHT                   //Ausgabe von Debug-Informationen zum Bremslicht
//#define HWDEBUG                           //Ausgabe der Laufzeit der control-loop mit der grünen LED
//#define POWERLIMIT                        //Ausgabe von Informationen zur Leistungsbeschränkung
//#define FUNNYLIGHT                        //kein Debug, aber Blinklicht beim Start der Elektronik

//Definition der Konstanten (Limits und technische Eigenschaften der Hardware)

//Limitierungen der Straßenverkehrsordnung für e-bikes
#define STVO_MaxSpeed                25      //km/h
#define STVO_MaxPowerMean            250     //W

//Limitierungen der Hardware, in diesem Fall die 30A Flachsteck-Sicherung aus dem KFZ-Bereich
#define MaxMotorCurrent              30      //A, bedingt durch die eingebaute Sicherung beim Akku
#define minBrakeSpeed                10      //km/h; unter diese Geschwindigkeit wird das Bremslicht nicht angesteuert, hier macht die Auswertung Schwierigkeiten

//Technische Eigenschaften des Fahrrads
#define wheelCircumreference         2.10    //26" Rad hat 2,10m Umfang
#define speedSensorMonoflopTime      200     //benötigt damit ein Mindesttempo von ca. 2km/h
#define rampIncrement                0.5     // 255/Wert * 10ms bis Vollausschlag

//Array-Größen für gleitenden Mittelwert
#define motorPowerArraySize         5       //Einheit 100ms; Buffer für gleitenden Mittelwert Leistung
#define motorCurrentArraySize       5       //Einheit 100ms; Buffer für gleitenden Mittelwert Strom
#define brightnessArraySize         5       //Einheit 100ms

//Einstellungen für das autmatische Licht
#define lightOnThreshold            600     //Einheit ADC counts (0-1023); Helligkeit unter der das Licht eingeschaltet wird
#define lightMinOnTime              90      //Einheit Sekunden, so lange leuchtet das Licht mindestens, auch wenn es wieder heller ist
#define brakeMonoflopTime           2       //Einheit Sekunden, so lange leuchtet bei einem Bremsvorgang das Bremslicht auf

//PWM Ausgabe-Wert für die verschiedenen Zustände des Lichts
#define lightDriving                255     //Ausgabe PWM
#define lightStanding               50      //Ausgabe PWM
#define lightBraking                30      //Ausgabe PWM
#define lightOff                    0       //Ausgabe PWM


//Timings und Vertzögerungen

#define keyAntibeatTime             25      //Einheit 10ms => 250ms Entprell-Zeit für die Tasten

#define timeStatusDisplay           10      //Einheit ist timeDispCycle; so lange wird der geänderte Zustand angezeigt

#define timeDispCycle               100     //Einheit Millisekunden, in diesem Zyklus wird das Display bedient


//Pinout zu Mapping siehe hier
//https://www.arduino.cc/en/Hacking/PinMapping32u4

// Timer 0 wird zur Erzeugung der internen Zeitbasis verwendet (delay-Funktion)

//LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 9, 4, 3, 2);

//Sonderzeichen Grad für die Temperaturanzeigen (Vorbereitung für die Zukunft)
byte grad[8] = {
  B01100,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000,
};

//Digitale IOs					Pin		Sonderfunktion			Timer-Funktion			Interrupt-Funktion
int wheelSensor = 				0;														//int 2   
int redLed = 					1;														//int 3
//2   D7 Display																		//int 1
//3   D6 Display                		PWM                     Timer0-B				//int 0
//4   D5 Display
int lightFront = 				5;           					//Timer3-A
int motor = 					6;                				//Timer4-D
int greenLed =					7;                              						//int 4
int enableCockpit = 			8;  
//9   D4 Display               			PWM						Timer1-A
int yellowLed = 				10;   //PWM       			    Timer1-B
// 11  ENABLE Display         		    PWM                     Timer0-A
// 12  RS Display
int lightRear = 13;           		  							//Timer4-A

//Analoge Eingänge
int throttlePin = 				0;
int currentPin = 				1;
int voltagePin = 				2;
int ldr =						3;
int switches =					4;
int temperaturePin =			5;


//Variablen Definition
int fullStopDelay,fullStopDelayMEM;

float magnetCycle, magnetCycleOld;
float cycleChange;
unsigned long magnetPulse, magnetPulseOld;

float magnetCycleMEM, magnetCycleOldMEM;
float cycleChangeMEM;
unsigned long magnetPulseMEM, magnetPulseOldMEM;
bool storeMEM, displayMEM;

bool minSpeed, maxSpeed, motorOn;
int speedMonoflop;
float pwmOut;
float voltage, current, power;

bool  toggle100ms;

float throttleValue;
unsigned long time2000ms, time1000ms, time100ms, time10ms, time1ms, lastTime1000ms, time500ms, lastTime500ms, currentTime;
unsigned long timeDispLoop;

int fcounter;
float frequency, wheelSpeed, oldWheelSpeed;
byte brakeMonoflop, toggleDelay;

float motorTemperature;

int analogLDR, analogSwitches;

int brightness[brightnessArraySize]; //array für gleitenden Mittelwert
int meanBrightness;
int brightnessSum;
byte brightnessArrayCounter;

float motorPower[motorPowerArraySize]; //array für gleitenden Mittelwert
float meanMotorPower;
float motorPowerSum;
byte motorPowerArrayCounter;

float motorCurrent[motorCurrentArraySize]; //array für gleitenden Mittelwert
float meanMotorCurrent;
float motorCurrentSum;
byte motorCurrentArrayCounter;

byte lightMinOn;
byte wheelPulseDelay;

bool powerLimitation;
bool currentLimitation;
bool powerLimitationIndication;
bool currentLimitationIndication;

volatile int timerIntCounter;

bool  key1, key2;               //direkte Signale der Taster
bool  key1Antibeat, key2Antibeat;   //entprellte Signale der Taster
bool  oldkey1Antibeat, oldkey2Antibeat;   //alte Werte für Flankenerkennung

byte  key1AntibeatCounter, key2AntibeatCounter; //Timer zum Entprellen

int i,j;
bool  toggle;
bool  toggle1;
bool  lightManual;
byte  displayStatusCounter;


/**********************************************************************************************************/
//  setup
// 
// Arduino Initialisierung
// 
// Diese Routine ist Teil der Arduino Umgebung, wird nach einem Reset einmalig aufgerufen.
// Darin werden die Variablen und die Hardware initialisiert. 
// 
/**********************************************************************************************************/
void setup() {                
 
// Hardware initialisieren
  pinMode(redLed, OUTPUT);     
  pinMode(greenLed, OUTPUT);     
  pinMode(yellowLed, OUTPUT);     
  pinMode(motor, OUTPUT);     
  pinMode(enableCockpit, OUTPUT);     
  pinMode(lightFront, OUTPUT);     
  pinMode(lightRear, OUTPUT);     
  pinMode(wheelSensor, INPUT);     
    
  //Variablen initialisieren
  motorTemperature = 23;
  storeMEM = true;
  displayMEM = false;
  magnetCycle = 0;
  magnetCycleOld = 0;
  cycleChange = 0;
  magnetPulse = 0;
  magnetPulseOld = 0;
  magnetPulseOld = 0;
  magnetPulse = 0;
  key1 = false;
  key1Antibeat = false;
  oldkey1Antibeat = false;
  key1AntibeatCounter = 0;
  key2 = false;
  key2Antibeat = false;
  oldkey2Antibeat = false;
  key2AntibeatCounter = 0;
  powerLimitationIndication = false;
  currentLimitationIndication = false;
  powerLimitation = false;
  currentLimitation = false;
  time1000ms = micros();
  lastTime1000ms = time1000ms;
  time100ms = micros();
  time10ms = micros();
  time1ms = micros();
  time2000ms = micros();
  timeDispLoop = millis();
  wheelPulseDelay = 0;
  speedMonoflop = 0;
  minSpeed = true;
  maxSpeed = false;
  pwmOut = 0;
  motorOn = true;
  fcounter = 0;
  meanBrightness = 0;
  brightnessSum = 0;
  brightnessArrayCounter = 0;
  for (i=0;i<brightnessArraySize;i++) {
    brightness[i] = 0;    
  }
  meanMotorPower = 0;
  motorPowerSum = 0;
  motorPowerArrayCounter = 0;
  for (i=0;i<motorPowerArraySize;i++) {
    motorPower[i] = 0;    
  }
  meanMotorCurrent = 0;
  motorCurrentSum = 0;
  motorCurrentArrayCounter = 0;
  for (i=0;i<motorCurrentArraySize;i++) {
    motorCurrent[i] = 0;    
  }
  lightMinOn = 0;
  toggleDelay= 0;
  timerIntCounter = 0;
  lightManual = false;    //Nach dem Start ist das Licht im Automatik-Modus

  //LCD einschalten und Begrüßung ausgeben
  digitalWrite(enableCockpit,HIGH);

  lcd.begin(16, 2);
  //Das Sonderzeichen erstellen
  lcd.createChar(0, grad);  
  lcd.home();
  //        "                "
  lcd.setCursor(0, 0);   
  lcd.print("    Starte      ");
  lcd.setCursor(0, 1);   
  lcd.print("    E-Bike      ");
  digitalWrite(redLed,HIGH);
  delay(200); 
  digitalWrite(redLed,LOW);
  digitalWrite(yellowLed,HIGH);
  delay(200); 
  digitalWrite(yellowLed,LOW);
  digitalWrite(greenLed,HIGH);
  delay(200); 
  digitalWrite(greenLed,LOW);
  delay(2000); 
  lcd.setCursor(0, 0);   
  lcd.print("          ");
  lcd.setCursor(0, 1);   
  lcd.print("   Gute Fahrt!   ");
  delay(2000); 
  lcd.clear();

#ifdef funnylight
//Falls man es spacig haben möchten, dann kann man diesen Teil hier aktivieren (Lichtblinken beim Hochfahren) 
for (j=0;j<3;j++) {
  for (i=0;i<255;i++) {
    analogWrite(lightRear,i);
    analogWrite(lightFront,i);
    delay (2);
  }
  delay (100);
  for (i=255;i>0;i--) {
    analogWrite(lightRear,i);
    analogWrite(lightFront,i);
    delay (2);
  }
  delay (100);
}
#endif
    
    //Interrupts generell stoppen
    cli();			

    //Timer1 Interrupt auf 10ms aufsetzen, dieser steuert alle internen Abläufe!
    TCCR1A = 0;			
    TCCR1B = 0;			
    TCNT1  = 0;			
    OCR1A = 20000;
    // CTC Modus einschalten
    TCCR1B |= (1 << WGM12);
    // CS11 8 prescaler
    TCCR1B |= (1 << CS11);  
    // Timer compare interrupt einschalten
    TIMSK1 |= (1 << OCIE1A);
    //Interrupts generell wieder freigeben
	sei();
    
   //den Hardware-Interrupt für den Geschwindigkeitssensor einhängen 
   attachInterrupt(2, speedSensor, FALLING);

#ifdef serialDebug
  //Bei Bedarf kann die seriellen DEBUG-Schnittstelle initialisiert werden    
  Serial.begin(9600);
#endif
} 

/**********************************************************************************************************/
// speedSensor
// 
// Das ist die Interrupt-Routine für den externen Hardware-Interrupt des Rad-Sensors (Reed-Kontakt mit 4 Magneten)
//
// Wird bei jeder negativen Flanke vom Reed-Kontakt ausgelöst
// 26" Rad, 4 Sensoren:
// Bei 50km/h (höchste Geschwindigkeit, nur bergab zu erreichen, also die Auslegungsgrenze für die Software)
// kommt alle 37,88ms ein Impuls eines Speichenmagneten. Entprellt wird mit 10-20ms, je nachdem wie der Interrupt 
// für den Entprell-Timer fällt. Die Impulsdauer ist unter 1ms (abhängig von dem Kreisdurchmesser der Sensorposition)
// 
/*********************************************************************************************************/
void speedSensor () {
  
  if (wheelPulseDelay == 0) {
    
    //erste Flanke von einem Magnet
    fcounter++;

    magnetPulse = millis();

    //Zeit seit dem letzten Magnet-Signal berechnen
    magnetCycle = magnetPulse - magnetPulseOld;
    
    if (magnetCycleOld != 0) {
    //falls der alte Zyklus 0 war dürfen wir das nicht berechnen (nach dem Anfahren)    
      
      cycleChange = ((float)magnetCycle/(float)magnetCycleOld) * 100.0;
      if ((cycleChange > 115) && (wheelSpeed > minBrakeSpeed)) {
          //Bremsvorgang erkannt
          brakeMonoflop = brakeMonoflopTime;
      }
    }

    //das Timeout für eine Blockierbremsung neu starten, dies ist abhängig von der zuletzt gefahrenen Geschwindigkeit
    fullStopDelay = (magnetCycle * 1.3 / 10) + 2;

#ifdef DEBUGBRAKELIGHT  
  if (storeMEM)  {
    magnetPulseMEM = magnetPulse;
    magnetPulseOldMEM = magnetPulseOld;
    magnetCycleMEM = magnetCycle;
    magnetCycleOldMEM = magnetCycleOld;
    cycleChangeMEM = cycleChange;
    fullStopDelayMEM = fullStopDelay;
    storeMEM = false;
    displayMEM = true;
  }
#endif
   
    magnetCycleOld = magnetCycle;
    magnetPulseOld = magnetPulse;
    
    //Entprellen starten
    wheelPulseDelay = 2;

    //Monoflop für die Anzeige starten
    speedMonoflop = speedSensorMonoflopTime;
    //die gelbe LED zeigt jeden Impuls eines Speichen-Magnets an
    digitalWrite(yellowLed, HIGH);
  }
}

/**********************************************************************************************************/
// ISR
// 
// Hardware Interrupt-Routine TIMER 1
// 
// Wird beim Ablauf des Timer 1 counters ausgelöst, ist eingestellt auf 10ms
// 
// 
/**********************************************************************************************************/
ISR(TIMER1_COMPA_vect){//timer1 interrupt
    
    //alle Control-Aufgaben durchführen
    controlLoop();

}


/**********************************************************************************************************/
//  loop
// 
// Arduino Hauptschleife
// 
// In der Hauptschleife wird nur das LCD bedient, dies dauert sehr lange. Alle anderen Aufgaben
// werden in den Interrupt gesteuerten Control-Routinen erledigt.
// 
/**********************************************************************************************************/
void loop() {
  
  if ((millis() - timeDispLoop) > timeDispCycle) {
    timeDispLoop = millis();
    updateDisplay();
  }

#ifdef DEBUGSERIAL
if (displayMEM) {
    Serial.print ("\nPulse ");
    Serial.print (magnetPulseMEM);
    Serial.print (" - PulseOld ");
    Serial.print (magnetPulseOldMEM);
    Serial.print (" = Cycle ");
    Serial.print (magnetCycleMEM);
    Serial.print ("; OldCycle:  ");
    Serial.print (magnetCycleOldMEM);
    Serial.print (" Change: ");
    Serial.print (cycleChangeMEM);
    Serial.print (" FullStopDelay: ");
    Serial.print (fullStopDelayMEM);
    Serial.print (" Brakemonoflop: ");
    Serial.print (brakeMonoflop);
    Serial.print (" Wheelspeed: ");
    Serial.print (wheelSpeed);
    storeMEM = true;
    displayMEM = false;
  } 
#endif
}

/**********************************************************************************************************/
// controlLoop
// 
// Wird alle 10ms aus dem Timer-Interrupt aufgerufen
// 
// Von hier aus werden die Control-Aufgaben in den entsprechenden Unterroutinen 
// aufgerufen.
// 
/**********************************************************************************************************/
void controlLoop(void) {

#ifdef HWDEBUG
  //debug Ausgabe
  digitalWrite(greenLed,HIGH);
#endif

  control10ms();
    
  if ((timerIntCounter % 100) == 0) {
    control100ms();
    control500ms();
    control1000ms();
  } else {
    if ((timerIntCounter % 50) == 0) {
      control100ms();
      control500ms();     
    }  else {
      if ((timerIntCounter % 10) == 0) {
        control100ms();
      }
    }  
  }  	

  timerIntCounter++;
  if (timerIntCounter == 65000) {
    timerIntCounter = 100;
  }

#ifdef HWDEBUG
  digitalWrite(greenLed,LOW);
#endif

} 

/**********************************************************************************************************/
// control10ms
// 
// Wird alle 10ms aus dem controlLoop aufgerufen
// 
// Behandelt folgenden Themen 
// - Entprellung der Reedkontakt-Impulse von den Speichenmagneten
// - Erkennung der Mindestgeschwindigkeit für die Aktivierung des Motors 
// - Einlesen und Umrechnen des Daumengas
// - Einlesen und Umrechnung von Batteriespanung und Strom, Berechnung der Leistung
// - Berechnung der Ausgangsrampe beim Beschleunigen
// - Ausgabe des Signals an den DAC für PWM-Ansteuerung des Akku-Schraubers
// - Tasten vom Cockpit einlesen und auswerten
// - Licht ansteuern (Standliche, Fahrlicht, Bremslicht)
/**********************************************************************************************************/
void control10ms (void) {
    
  //Radimpuls Monoflop bereits abgelaufen?
  //Der Interrupt läuft mit 10ms, das Delay muss also auf 2 Zyklen eingestellt werden, sonst könnte es passieren, dass der Sensor-Interrupt unmittelbar vor dem 10ms Interrupt kommt und somit eine 
  //Fehlmessung entsteht, da das Monoflop zu schnell abhläuft und somit das Prellen mitgemessen wird.
  if (wheelPulseDelay > 0) {
    wheelPulseDelay--;
  } else {
      digitalWrite(yellowLed, LOW);
  }

  //Timeout für eine Blockierbremsung. Nach spätestens 120ms wird auch bei fehlendem weiteren Impuls des Radsensors eine Blockierbremsung erkannt!
  if (fullStopDelay > 0) {
    fullStopDelay--;
    if ((fullStopDelay == 0) && (wheelSpeed > minBrakeSpeed)) {
    //Timeout abgelaufen, Blockierbremsung, Bremslicht auslösen
      brakeMonoflop = brakeMonoflopTime;
    }
  }
  
  if (speedMonoflop > 0) {
    speedMonoflop--;
    minSpeed = true; 
  } else {  
  minSpeed = false;
  }
  
  //Analogwerte einlesen
  //Maximales-Signal vom Daumengas ist nur 85, bedingt durch den Drehbereich des Daumengas
  //unter 5% läuft der Motor nicht, deshalb die 1-100% Eingangswert auf 5-100% mappen. 0% ist Motor aus.
    
  // 5% = 12,75
  // y = k*x +d
  // k = 255-12,75/85-0 = 2,85
  // d = y bei x=0  => d=5
  // y = 2,85*x + 12,75
  
  throttleValue = (analogRead(throttlePin)/4)*2.85+12.75;
  if (throttleValue < 14) {
    throttleValue = 0;
  }
 
  if (throttleValue > 255) {
    throttleValue = 255;
  }
  
  //Spannung und Strom einlesen, daraus die aktuelle Leistung berechnen
  
  //ADC Referenzspannung ist 5V, daher Faktor 5
  //wegen Spannungsteiler am Eingang des ADC wird nur 1/3 gemessen, daher faktor 3
  voltage = (((float)(analogRead(voltagePin)))/1024)*5*3;
  if (throttleValue > 0) {
  
  //ADC Referenzspannung ist 5V, daher Faktor 5
    current = (((float)(analogRead(currentPin)))/1024)*5*8*1.5;
  } else {
    current = 0;
  }
  
  //Aktuelle Leistung berechnen
  power = voltage * current;

  //Und eventuelle Überlast-Situation berechnen
  if ((meanMotorPower > STVO_MaxPowerMean))  {
    powerLimitation = true;
    powerLimitationIndication = true;
  } else {
    powerLimitation = false;
  }
    
  //Und eventuell Überstrom-Situation berechnen
  if (meanMotorCurrent > MaxMotorCurrent)  {
    currentLimitation = true;
    currentLimitationIndication = true;
  } else {
    currentLimitation = false;
  }
  
  //Bedinungen für die Motor-Freigabe
  //Vorbedingung ist eine Mindestgeschwindigkeit, damit nicht aus dem Stand angefahren werden kann

  if ((minSpeed) && (wheelSpeed < STVO_MaxSpeed)) {
       motorOn = true;    
     } else {
       //ohne Bewegung kein Signal ausgeben, dient zur Vermeidung von hohen Drehmomenten aus dem Stillstand heraus
        motorOn = false;
     }

  //Ansteuerung PWM-Stufe für Akkuschrauber
  
  //Startup-Rampe: Beim Einschalten steigt der Wert nur mit einer Rampe an, beim Abschalten gibt es keine Rampe
  if (motorOn) { 

    //Wenn das Limit unterschritten wurde, so startet wieder die normale Beschleuigungsrampe
    //Falls das Limit wieder überschritten wird, so beginnt das Spielchen von vorne
        
    //Leistungsbeschränkung bei Überschreitung der maximalen Leistung,
    //dann wird die elektrisch Leistung pro Zyklus um 5% zurückgenommen
    if ((powerLimitation || currentLimitation)) {
      //Überschreitung der maximalen Leistung
      //Ausgangssignal um 5% pro Zyklus reduzieren
      pwmOut = pwmOut*0.95;
    
    } else {  
      // Leistung unter dem Limit
      if (throttleValue != pwmOut) {
        if (throttleValue > pwmOut) {
          //ansteigend geht langsam
          pwmOut = pwmOut + rampIncrement;
        } else {
          //abfallend geht sofort
          pwmOut = throttleValue;
        }
      }
    }
 }
 
 // Ausgabe des Wertes an den DAC
  if (motorOn) {
  //Motorsignal ausgeben
  analogWrite(motor, (int)pwmOut);  
  }
  else {
  //Motor ist aus
    pwmOut = 0;  
    analogWrite(motor, 0);  
  }     

  //Einlesen des lichtabhängigen Widerstandes zur Erkennung der Helligkeit
  analogLDR = analogRead(ldr);
  
  //Tasten einlesen: Dabei sind zwei Tasten mit Spannungsteilern an einen Analog-Eingang angeschlossen
  //deshalb ist ein wenig Tricksen notwendig.
  analogSwitches=analogRead(switches);
  
  if (analogSwitches > 900) {
    key1 = false;
    key2 = false;
  } else {
    if (analogSwitches > 630) {
      key1 = false;
      key2 = true;
    } else {
      if (analogSwitches > 480) {
        key1 = true;
        key2 = false;
      } else {
        key1 = true;
        key2 = true;
      }
    }
  }

    if (toggleDelay == 3) {
      toggle100ms = !toggle100ms;
      toggleDelay = 0;  
    } else {
      toggleDelay++;
    }

  if ((lightMinOn != 0)) {
    //Licht einschalten
    
    if (minSpeed) {
      //Wir bewegen uns mit der Mindestgeschwindigkeit
      
      //Licht vorne DAC ansteuern
      analogWrite(lightFront, lightDriving);

      //Licht hinten
      if (brakeMonoflop > 0) {
        //es wurde gebremst
        if (toggle100ms) {
        analogWrite(lightRear, lightDriving);
        } else {
        analogWrite(lightRear, lightBraking);
        }
      
      } else {
        analogWrite(lightRear, lightDriving);
      }

    } else {
     //das Rad steht
      analogWrite(lightFront, lightStanding);
      analogWrite(lightRear, lightStanding);
    }

  } else {
    //eigentlich Licht aus, aber Mindestzeit an lassen
    if (lightMinOn == 0) {
      analogWrite(lightFront, lightOff);

      if (brakeMonoflop > 0) {
        //es wurde gebremst
        if (toggle100ms) {
        analogWrite(lightRear, lightDriving);
        } else {
          analogWrite(lightRear, lightOff);
        }
      } else {
          analogWrite(lightRear, lightOff);
      }
    }
  }
  
  if (powerLimitationIndication || currentLimitationIndication) {
   digitalWrite(redLed,HIGH);
  } else {
    digitalWrite(redLed,LOW);
  }

 
  //Behandlung der beiden Tasten zum Erzeugen von entprellten Signalen
  if ((key1) && (key1AntibeatCounter == 0)) {
    key1AntibeatCounter = keyAntibeatTime;
    key1Antibeat = true;
  }

  if (key1Antibeat) {
    key1AntibeatCounter--;
    if (key1AntibeatCounter == 0) {
      key1Antibeat = false;
    }
  }
  
  if ((key2) && (key2AntibeatCounter == 0)) {
    key2AntibeatCounter = keyAntibeatTime;
    key2Antibeat = true;
  }

  if (key2Antibeat) {
    key2AntibeatCounter--;
    if (key2AntibeatCounter == 0) {
      key2Antibeat = false;
    }
  }

  //Wenn Taste 1 gedrückt wurde, so wird der Licht-Modus von Automatik auf Manuell geändert bzw. umgekehrt
  if ((!oldkey1Antibeat) && (key1Antibeat) && !key2Antibeat) {
    lightManual = !lightManual;
    displayStatusCounter = timeStatusDisplay;
    if (!lightManual) {
      lightMinOn = 0;
    }
    
  }

  //alte Tastenwerte für Flankenerkennung speichern
  oldkey1Antibeat = key1Antibeat;
  oldkey2Antibeat = key2Antibeat;
   
}

/**********************************************************************************************************/
// control100ms
// 
// Wird alle 100ms aus dem controlLoop aufgerufen
// 
// Behandle folgende Themen:  
// - Einschalten des Lichts durch Drücken der Taste 2
// - Berechnung der mittleren Helligkeit (gleitender Mittelwert)
// 
// 
// 
// 
/**********************************************************************************************************/
void control100ms (void) {

  if (lightManual || (meanBrightness < lightOnThreshold)) { 
    lightMinOn = lightMinOnTime;
  } 

 
    //Helligkeit gleitender Mittelwert
    brightnessSum = brightnessSum - brightness[brightnessArrayCounter];
 
    brightness[brightnessArrayCounter] = analogRead(ldr);

    brightnessSum = brightnessSum + brightness[brightnessArrayCounter];

    if (brightnessArrayCounter < (brightnessArraySize -1)) {
      brightnessArrayCounter++;
    } else {
      brightnessArrayCounter = 0;
    }

    meanBrightness = brightnessSum/brightnessArraySize;

    //gleitender Mittelwert für die Motorleistung
    motorPowerSum = motorPowerSum - motorPower[motorPowerArrayCounter];
    motorPower[motorPowerArrayCounter] = power;
    motorPowerSum = motorPowerSum + motorPower[motorPowerArrayCounter];

    if (motorPowerArrayCounter < (motorPowerArraySize -1)) {
      motorPowerArrayCounter++;
    } else {
      motorPowerArrayCounter = 0;
    }
    meanMotorPower = motorPowerSum/motorPowerArraySize;
    if (meanMotorPower < 0.1) {
      meanMotorPower = 0.0;
    }

    //gleitender Mittelwert für den Motorstrom
    motorCurrentSum = motorCurrentSum - motorCurrent[motorCurrentArrayCounter];
    motorCurrent[motorCurrentArrayCounter] = current;
    motorCurrentSum = motorCurrentSum + motorCurrent[motorCurrentArrayCounter];

    if (motorCurrentArrayCounter < (motorCurrentArraySize -1)) {
      motorCurrentArrayCounter++;
    } else {
      motorCurrentArrayCounter = 0;
    }
    meanMotorCurrent = motorCurrentSum/motorCurrentArraySize;

#ifdef POWERLIMIT
    Serial.print ("\nCurrent: ");
    for (i=0;i<motorCurrentArraySize;i++) {
      Serial.print (" ");
      Serial.print (motorCurrent[i]);
    }
    Serial.print (" : ");
    Serial.print (motorCurrentSum);
    Serial.print (" : ");
    Serial.print (meanMotorCurrent);

    Serial.print ("\nPower: ");
    for (i=0;i<motorPowerArraySize;i++) {
      Serial.print (" ");
      Serial.print (motorPower[i]);
    }
    Serial.print (" : ");
    Serial.print (motorPowerSum);
    Serial.print (" : ");
    Serial.print (meanMotorPower);
#endif
}

/**********************************************************************************************************/
// control500ms
// 
// Wird alle 500ms aus dem controlLoop aufgerufen
// 
// Behandle werden folgenden Themen: 
//**********************************************************************************************************/
void control500ms (void) {

}

/**********************************************************************************************************/
// control1000ms
// 
// Wird alle 1000ms aus dem controlLoop aufgerufen
// 
// Behandelt folgende Themen:
// - Geschwindigkeitsmessung mit einer Torzeit von 1s (höhere Genauigkeit) 
// - Rote Blinkled als LIVE-Led
/**********************************************************************************************************/
void control1000ms (void) {  
  
#ifdef HWDEBUG

#else
    if (digitalRead(greenLed) == HIGH) {
      digitalWrite(greenLed, LOW);
    } else {
      digitalWrite (greenLed, HIGH);
    }
#endif

  if (brakeMonoflop > 0) {
    brakeMonoflop--;
  }
  
  if (lightMinOn > 0) {
    lightMinOn--;
  }

  lastTime1000ms = time1000ms;
  time1000ms = micros();
  //Die Berechnung der Frequenz ist an die tatsächlich abgelaufenen Zeit angepasst
  frequency = fcounter/((time1000ms - lastTime1000ms)/1000000.0);
  fcounter = 0;
  oldWheelSpeed = wheelSpeed;
  wheelSpeed = frequency * wheelCircumreference * 3.6/4;

 //Hier werden die Anzeige zurückgesetzt, damit diese am Display sichtbar werden
 powerLimitationIndication = false;
 currentLimitationIndication = false;

  //auslesen der Motortemperatur
  motorTemperature = (((float)(analogRead(temperaturePin)))/1024)*5;

}

/**********************************************************************************************************/
// updateDisplay
//
// wird zyklisch aus dem Haupt-Loop aufgerufen
// 
// Aktualisiert das LDC
// 
// 
/**********************************************************************************************************/
void updateDisplay(void) {
  
    //die Ausgabe auf das Display benötigt 12ms
    //das lcd.home braucht sehr lange, schneller ist ein Setcursor
    //ohne Ausgabe der Einheiten 8ms

float displayBuffer;

    
    lcd.clear();

    if (displayStatusCounter != 0) {
        lcd.print("  Licht Modus");
        displayStatusCounter--;
        lcd.setCursor(0, 1);   

      if (lightManual) {
        lcd.print("      EIN");
      } else {
        lcd.print("   AUTOMATIK");
      }

      if (displayStatusCounter == 0) {
        lcd.home();
      }
      
    } else {
      
      //if (motorOn) {
      //  lcd.print("Ein ");
      //} else {
      //  lcd.print("Aus ");
      // }
      //lcd.print(" ");

      
      if (motorOn) {
        displayBuffer = pwmOut/2.55; 
        if (displayBuffer < 100.0) lcd.print(" ");
        if (displayBuffer < 10.0) lcd.print(" ");
        lcd.print((int)(displayBuffer));
        lcd.print("%  ");
      } else {
        lcd.print("  ! ");
      }

      lcd.setCursor(6, 0);
      displayBuffer = motorTemperature;   
      if (displayBuffer < 10.0) lcd.print(" ");
      lcd.print(displayBuffer,0);
      lcd.write(byte(0));
 
      lcd.setCursor(11,0);
      displayBuffer = voltage;
      if (displayBuffer < 10.0) lcd.print(" ");
      lcd.print(displayBuffer,1);
      lcd.print("V");
        
      lcd.setCursor(1, 1);   
      displayBuffer = meanMotorCurrent;
      if (displayBuffer < 10.0) lcd.print(" ");
      lcd.print((int)displayBuffer);
      if (currentLimitationIndication) {
        lcd.print("A!");
      } else {
        lcd.print("A ");
      }
          
      lcd.setCursor(5, 1);   
      displayBuffer = meanMotorPower;
      if (displayBuffer < 100.0) lcd.print(" ");
      if (displayBuffer < 10.0) lcd.print(" ");
      lcd.print(displayBuffer,0);
  
      if (powerLimitationIndication) {
        lcd.print("W!  ");
      } else {
        lcd.print("W   ");
      }
   
      lcd.setCursor(11, 1);   
      if (wheelSpeed < 10) lcd.print(" ");
      lcd.print(wheelSpeed,0);
      lcd.print("kmh ");
    }
}
  
