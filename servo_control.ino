#include<Servo.h>
Servo ser1;
Servo ser2;
Servo ser3;

void setup() {
 ser1.attach(A0);//the first servo is attached to A0 which is the X axis servo
 ser2.attach(A1);//the second servo is attached to A1 which is the Y axis servo
 ser3.attach(A2);//the third servo is attached to A2 which is the trigger servo
 ser3.write(0);
 Serial.begin(250000);
}
String a,c; 
int b,d;
void loop() {
  if(Serial.available())
  {
    a=Serial.readStringUntil('\n'); //reads the angle values for both X and Y servos
    b=a.toInt();                    //converts the angles to integer
    while(!Serial.available()){}
    c=Serial.readStringUntil('\n'); // reads whether the value obtained is to be written on X Servo or Y servo
   // Serial.println(c);
    if(c=="a")
    ser1.write(b);            //if we obtain 'a' then the X servo moves
    else if(c=="b")
    ser2.write(b);            //if we obtain 'b' then the Y servo moves
    else if(c=="c")           //if we obtain 'c' then the trigger servo is pulled
    {
    ser3.write(60);
    delay(1000);
    ser3.write(0);
    }//Serial.println(a);
  }


}