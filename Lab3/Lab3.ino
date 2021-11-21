
#include <LiquidCrystal.h>
#include <Wire.h>
#include <RTClib.h>
#include <String.h>
//PHYSICAL DESIGN FOR CLOCK BELOW
/*
 * GND is ground.
 * VCC is 5 volts. 
 * SDA is connected to SDA...
 * SCL is also connected to SCL...
 * SQW is for a square wave, but were not going to be using this.
 */
//RTC Object
RTC_DS1307 rtc;
DateTime currentTime;
//Array of the days of the week.
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


//LCD Pins
#define rs 52
#define e 50
#define d7 47
#define d6 49
#define d5 51
#define d4 53

//Liquid Crystal Object
LiquidCrystal lcd(rs,e,d4,d5,d6,d7);

//PHYSICAL SETUP DESCRIPTION FOR LCD BELOW::
/*
 * Vss is connected to to ground
 * Vdd is connected to +5 volts
 * V0 pin is the contrast pin. Connect to a 10k potentiometer to control contrast
 * Rs pin is the register select pin. Selects whether we are sending data or commands to the LCD. 0 volts its commands. 5 volts for data.
 * R/W pin selects mode to read to write to the LCD.
 * E pin is the enable pin. Enables writting to the LCD's registers.
 * D0-D7 is the data pins for parralel communication of sending data from arduino to board.
 * A is the anode for the LCD backlight. NOTE YOU NEED A RESISTOR IN SERIES WITH A 5+ SUPPLY HERE!!!
 * K is the cathode for the LCD backlight.
 * 
 * Note: we use the LiquidCrystal Library.
 * We can use it in 4 bit or 8 bit mode.
 */

//DC motor global variables
String rotation = "CCW";
int fanPower = 100;
#define E1 2 // Enable pin for motor
#define I1 22 // Control pin for motor
#define I2 24 // second control pin for motor
const byte interruptPin = 3; 
volatile byte state = LOW;

boolean timer = false;

void setup() {
  //The code below sets up the button interrupt for the lab.
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 15624;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);
  sei();
  
  //Setting up button interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), changeDirection, RISING);

  //Sets up for DC motor
  pinMode(E1, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(3, INPUT);
  
  lcd.begin(16,2); //Begins communication with LCD.
  rtc.begin();    //Begins communication with RTC.
  stopRTC();  //Stops the RTC

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //Sets the RTC's registers to the current date and time

  startRTC();   //Starts the RTC
   
  Serial.begin(57600);
}

void loop() {   //Main loop, runs motor control function, then prints data to LCD.
  
  if(timer == true){
    timer = false;
    motorControl();
    lcdPrint();
    }
  
}

void stopRTC(){ //Stops the internal oscillator within the RTC to write data to it.
  Wire.begin();
  Wire.begin(0x68); //Start I2C protocol with DS1307 address
  Wire.write(0); //Send Register Address
  Wire.write(1); //Sets CH to 1, stopping the oscillator if it was running.
  Wire.endTransmission(true); //Ends Serial communication with RTC
  Wire.end();
  }

void startRTC(){ //Starts the internal oscillator of the RTC.
  Wire.begin();
  Wire.begin(0x68); //Start I2C protocol with DS1307 address
  Wire.write(0); //Send Register Address
  Wire.write(0); //Sets CH to 0, starting the oscillator.
  Wire.endTransmission(true); //Ends Serial communication with RTC
  Wire.end();
  
  }

void lcdPrint(){ //This function prints everything to the LCD display.
  lcd.clear();
  currentTime = rtc.now();
  lcd.setCursor(0,0);
  lcd.print(currentTime.hour(), DEC);
  lcd.print(':');
  lcd.print(currentTime.minute(), DEC);
  lcd.print(':');
  lcd.print(currentTime.second(), DEC);

  //Printing current motor speed:
  lcd.setCursor(0,1);
  lcd.print("Fan: "); lcd.print(fanPower); lcd.print("% "); lcd.print("("); lcd.print(rotation); lcd.print(")");
  }

void motorControl(){ //This function controls the Motor direction and if it is on or off.
  int power;
  currentTime = rtc.now();
  if(currentTime.second() > 29){ //If seconds is above 29, turn of the motor.
    fanPower = 0;
    digitalWrite(E1, LOW);
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    }
  else{                         //If below 29 seconds, turn on the motor.
    fanPower = 100;   
    power = ((255/100)*fanPower);
    if(rotation == "CCW"){      //If the rotation variable is "CCW" make the motor spin CCW
      digitalWrite(E1, power);
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
      }
    if(rotation == "CW"){       //If the rotation variable is "CCW" make the motor spin CCW
      digitalWrite(E1, power);
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);
      }
    }
  }

void changeDirection(){
  if(rotation == "CCW"){
      rotation = "CW";
    }
  else{
      rotation = "CCW";
    }
  }

  ISR(TIMER1_COMPA_vect){  //change the 0 to 1 for timer1 and 2 for timer2
   timer = true;
}
