#if defined (PARTICLE)
#else
#include <Wire.h>
#endif
#include <string>

#include "Grove_LCD_RGB_Backlight.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"

Adafruit_BMP280 bmp; // I2C

rgb_lcd lcd;

Servo myservo;
int servoPin = A4;
int servoPos = 0;

int ledR = D4;
int ledG = D5;
int ledB = D6;
int ledW = D7;

int hotTol = 27;
int chillTol = 15;
int coldTol = 5;

Timer timer(5000, checkEnvironment);

// Update to reflect current pressure at sea level
float seaLevelhPa = 1035.7;

int t = 20, tH = 20, tL = 20;
String todayCondition = "clear";
String condition = "clear";

int r = 255, g = 255, b = 255;

void setup() {
  Serial.begin(9600);

  Particle.publish("DEBUG", "starting...");

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledW, OUTPUT);

  myservo.attach(servoPin);
  myservo.write(0);

  lcd.begin(16, 2);

  if (bmp.begin()) {
    Particle.publish("DEBUG", "starting the environment timer...");
    /*timer.start();*/
  }
  else {
    Particle.publish("WARN", "Could not find a valid BMP280 sensor, check wiring!");
  }

  Particle.publish("DEBUG", "started!");

  Particle.subscribe("now", tempHandler);
  Particle.subscribe("rain", tempHandler);
  Particle.subscribe("snow", tempHandler);
  Particle.subscribe("cloudy", tempHandler);
  Particle.subscribe("clear", tempHandler);

  Time.zone(-4);
}

void loop() {
  Particle.syncTime();
  /*Serial.println(Time.timeStr());

  Serial.print("temperature is ");
  Serial.print(t);
  Serial.println("*C");
	delay(1000);
  Serial.println(String(r) + ", " + String(g) + ", " + String(b));*/
  lcd.setRGB(r, g, b);
  myservo.write(servoPos);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);
  digitalWrite(ledW, LOW);
  delay(15);

  if (t > hotTol) {
    /*Serial.println("hot");*/
    servoPos = 157.5;
    /*lcd.setRGB(255, 0, 0);*/
    digitalWrite(ledR, HIGH);
    delay(15);
  }

  else if (t <= hotTol && t > chillTol) {
    /*Serial.println("nice");*/
    servoPos = 112.5;
    /*lcd.setRGB(0, 255, 0);*/
    digitalWrite(ledG, HIGH);
    delay(15);
  }

  else if (t <= chillTol && t > coldTol) {
    /*Serial.println("chill");*/
    servoPos = 67.5;
    /*lcd.setRGB(0, 0, 255);*/
    digitalWrite(ledB, HIGH);
    delay(15);
  }

  else if (t <= coldTol) {
    /*Serial.println("cold");*/
    servoPos = 22.5;
    /*lcd.setRGB(255, 255, 255);*/
    digitalWrite(ledW, HIGH);
    delay(15);
  }

  printInit();
  delay(2000);

  printTemp();
  delay(2000);

  printCond();
  delay(2000);
}

void tempHandler(const char * name, const char * data) {
  /*Serial.println("inside handler.");*/
  /*Serial.println(name);*/
  if (strcmp(name, "now") == 0) {
    Serial.println("in now");
    Serial.println(data);
    String dataString = data;
    Serial.println(dataString);
    Serial.println(dataString.substring(dataString.lastIndexOf(".") + 2));
    String bothConditions = dataString.substring(dataString.lastIndexOf(".") + 2);
    Serial.println(bothConditions);
    todayCondition = bothConditions.substring(0, bothConditions.lastIndexOf(","));
    Serial.println(todayCondition);
    condition = bothConditions.substring(bothConditions.lastIndexOf(",") + 2);
    Serial.println(condition);
    t = dataString.toInt();
    Serial.println(t);
    tH = (dataString.substring(dataString.indexOf(",") + 2)).toInt();
    Serial.println(dataString.substring(dataString.indexOf(",") + 2));
    String tempHL = dataString.substring(dataString.indexOf(",") + 2);
    tL = (tempHL.substring(tempHL.indexOf(",") + 2)).toInt();
    Serial.println(tempHL.substring(tempHL.indexOf(",") + 2));
    Particle.publish("Now", String(t));
  }
  else if (strcmp(name, "rain") == 0) {
    /*Serial.println("in rain");*/
    r = 125;
    g = 0;
    b = 125;
    t = atoi(data);
    condition = "rain";
    Particle.publish("Rain", String(t));
  }
  else if (strcmp(name, "snow") == 0) {
    /*Serial.println("in snow");*/
    r = 0;
    g = 255;
    b = 255;
    t = atoi(data);
    condition = "snow";
    Particle.publish("Snow", String(t));
  }
  else if (strcmp(name, "cloudy") == 0) {
    /*Serial.println("in cloudy");*/
    r = 50;
    g = 50;
    b = 50;
    t = atoi(data);
    condition = "cloudy";
    Particle.publish("Cloudy", String(t));
  }
  else if (strcmp(name, "clear") == 0) {
    /*Serial.println("in clear");*/
    r = 255;
    g = 255;
    b = 255;
    t = atoi(data);
    condition = "clear";
    Particle.publish("Clear", String(t));
  }
}

void checkEnvironment() {
  Particle.publish("environment/temperature", String(cToF(bmp.readTemperature())));
  Particle.publish("environment/pressure", String(pToHg(bmp.readPressure())));
  Particle.publish("environment/altitude", String(bmp.readAltitude(seaLevelhPa)));
}

void printInit() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Weather Station");
  lcd.setCursor(0, 1);
  lcd.print(Time.timeStr());
}

void printTemp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("H/L : ");
  lcd.print(tH);
  lcd.print("C/");
  lcd.print(tL);
  lcd.print("C");
}

void printCond() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(todayCondition);
  lcd.setCursor(0 , 1);
  lcd.print(condition);
}

float cToF(float c) {
  return c * 9/5 + 32;
}

float pToHg(float p) {
  return p/3389.39;
}
