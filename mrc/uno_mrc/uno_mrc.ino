#include <LiquidCrystal.h>
#include "shared.h"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  Serial.begin(SERIAL_BAUD);
  lcd.begin(16, 2);
  lcd.print("Hello, World!");
  pinMode(A0, INPUT);
}
char old_button = 'x';
void loop()
{
  char button = getButtonValue();

  if (button != 'x')
    Serial.write(button);

  if (Serial.available())
  {
    char incoming = Serial.read();
    if (incoming == '.')
    {
      while(incoming != '\n' && incoming != '\r'){
        incoming = Serial.read();
      }
    }
    else if (incoming == '#')
    {
      //connected!
      lcd.clear();
      lcd.home();
    }
    else if (incoming == '!')
    {
      //http error string
      lcd.clear();
      lcd.home();
      while (Serial.available())
      {
        char x = Serial.read();
        if (x == '\n' || x == '\r')
          break;
        lcd.print(x);
      }
    }
    else if (incoming == '|')
    {
      //pose estimation
      printPoseInformation();
    }
    else
    {
      //skip, newline, return, etc.
    }
  }

  delay(50);
}

float getFloatValue()
{
  String tmp = "";
  char x = Serial.read();
  while (x != ',' && x != '\n' && x != '\r')
  {
    tmp += x;
    x = Serial.read();
  }
  return tmp.toFloat();
}

void printPoseInformation()
{
  //reading from serial, we have already received '|'
  float x = getFloatValue();
  float y = getFloatValue();
  float theta = getFloatValue();

  Serial.println("x,y,z:");
  Serial.println(x, 4);
  Serial.println(y, 4);
  Serial.println(theta, 4);
  drawInterface(x, y, theta);
}
void drawInterface(float x, float y, float theta)
{
  lcd.setCursor(0, 0);
  lcd.print("::ROS     ");

  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(x < 0 ? ""
                  : "+");
  lcd.print(x, 2);

  lcd.setCursor(9, 1);
  lcd.print("Y:");
  lcd.print(y < 0 ? ""
                  : "+");
  lcd.print(y, 2);

  lcd.setCursor(9, 0);
  lcd.write((char)0b11110111); //from datasheet
  lcd.print(":");
  lcd.print(theta < 0 ? ""
                      : "+");
  lcd.print(theta, 2);
}
char getButtonValue()
{
  float value = analogRead(A0) / 10.0f;
  unsigned char v = round(value);
  char button = 'x';

  if (v < 5)
  {
    button = 'r';
  }
  else if (v < 15)
  {
    button = 'u';
  }
  else if (v < 30)
  {
    button = 'd';
  }
  else if (v < 50)
  {
    button = 'l';
  }
  else if (v < 70)
  {
    button = 's';
  }

  return button;
}