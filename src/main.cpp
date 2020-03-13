#include "defines.h"
#include <Arduino.h>
#include <Servo.h>
#include <thread>
#include <chrono>

Servo myservo;
int enc_value = 55;
std::chrono::system_clock::time_point now;

void read_speed(){
	//Speed sensor init
	int speed_read_value;
	bool was_covered = false;
	long speed_counter = 0;
	int time_passed;
	const auto speed_timer_sleep = std::chrono::microseconds(1000);
	std::chrono::system_clock::time_point speed_read_at_time = std::chrono::system_clock::now();
	while (1) {
		speed_read_value = analogRead(SPEED_READ_PIN);
		if (speed_read_value < 3400 && ! was_covered) {
			was_covered = true;
			speed_counter++;
		} else if (speed_read_value > 3400) {
			was_covered = false;
		}
		now = std::chrono::system_clock::now();
		time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(now - speed_read_at_time).count();
		if (time_passed >= 1000){
			Serial.print("Sensor speeed ::");
			Serial.println((speed_counter * 60000)/ time_passed);
			speed_read_at_time = std::chrono::system_clock::now();
			speed_counter = 0;
		}

		
		std::this_thread::sleep_for(speed_timer_sleep);
	}
}

void servo_manipulation(){
	enc_value +=1 ;
	
	if (enc_value > 65){
		enc_value = 55;
	}
	//Serial.println(enc_value);
	myservo.write(enc_value); 
}

	//Create a task for speed read
    std::thread read_speed_thread(read_speed);
void setup() {
	//Set the servo
	myservo.attach(SERVO_PIN);
	//Start the serial readout
	Serial.begin(115200);
	
	
	pinMode (LED_BUILTIN, OUTPUT);
}

void loop() {
	digitalWrite(LED_BUILTIN, HIGH);
	servo_manipulation();
	delay(3000);
	digitalWrite(LED_BUILTIN, LOW);
	servo_manipulation();
	delay(3000);
}