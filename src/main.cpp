#include <Arduino.h>
#include <Servo.h>


Servo myservo;
int value = 0;

void servo_manipulation(){
	value +=10 ;
	
	if (value > 180){
		value = 0;
	}
	Serial.println(value);
	myservo.write(value); 
}

void setup() {
	Serial.begin(115200);
	pinMode (LED_BUILTIN, OUTPUT);
	myservo.attach(23);
}

void loop() {
	digitalWrite(LED_BUILTIN, HIGH);
	servo_manipulation();
	delay(1000);
	digitalWrite(LED_BUILTIN, LOW);
	servo_manipulation();
	delay(1000);
}