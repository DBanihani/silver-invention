#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
//byte PS_16 = (1 << ADPS2);
//byte PS_32 = (1 << ADPS2) | (1 << ADPS0);
//byte PS_64 = (1 << ADPS2) | (1 << ADPS1);
//byte PS_128 =(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
//long starttime, interval;
float measuredCurrent = 0;
void setup() {
  // put your setup code here, to run once:
  TCCR1B = TCCR1B & B11111000 | B00000001;
  //ADCSRA &= ~PS_128;
  //ADCSRA |= PS_16;
  Serial.begin(9600);
  Serial.println(TCCR1B, BIN);
  pinMode(10,OUTPUT);
  pinMode(A0,INPUT);
  attachInterrupt(digitalPinToInterrupt(3),sampleCurrent,RISING);
  lcd.init();
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
  float pot = ((float)analogRead(A0)) * 255 / 1023;
  lcd.setCursor(0,0);
  lcd.print(pot);
  analogWrite(10, pot);
}

void sampleCurrent(){
  measuredCurrent = analogRead(/*SAMPLEPIN*/6);
}

 /*
//Including libraries
#include <LiquidCrystal_I2C.h>

//Prototyping fns
void sampleCurrent();

//Macros to make code easier to follow
#define CURRENTIN A0//Analog pin for pot to select current
#define RAMPTIN A1//Analog pin for pot for ramp time
#define WELDTIN A2//Analog pin for pot for weld time
#define SDA A4//LCD SDA pin
#define SCL A5//LCD SCL pin
#define DUTYPIN 3//output dutycycle NEEDS TO BE PWM OUTPUT PIN
#define SAMPLEPIN 7//input sample pin NEEDS TO BE PWM INPUT

float measuredCurrent = 0;//Create global variable bescause ISR needs to be void and take no paramenters
//Variables for PID control
int kp = 0;
int ki = 0;
int kd = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);//Create LCD object with register and resolution

void setup(){
  TCCR1B = TCCR1B & B11111000 | B00000001;// Sets timer prescaler to 1
  //defining I/O
  pinMode(RAMPTIN, INPUT);
  pinMode(CURRENTIN, INPUT);
  pinMode(WELDTIN, INPUT);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  pinMode(DUTYPIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3),sampleCurrent,RISING);//make sampling on same clock that PWM is?
  //Initializing LCD
  lcd.init();
  lcd.backlight();
}

void loop(){
  int rampTime = analogRead(RAMPTIN);
  float setCurrent = ((float)analogRead(CURRENTIN)) * 100 / 1023;
  int weldTime = analogRead(WELDTIN);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("I:");
  lcd.print(setCurrent);
  lcd.print(" R:");
  lcd.print(rampTime);
  lcd.setCursor(0,1);
  lcd.print("T:");
  lcd.print(weldTime);
 
 
  float D = 0.1;


}

void sampleCurrent(){
  measuredCurrent = analogRead(SAMPLEPIN);
}

/*float feedback(float measured,float set){
  float error = measured-set;
  float PIDerror = error*kp + (error-previousError)/
  return PIDerror



}*/
