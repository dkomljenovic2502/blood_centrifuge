#include "defines.h"
#include <Arduino.h>
#include <SPI.h>
#include <thread>
#include <chrono>
#include "Servo.h"
#include "AiEsp32RotaryEncoder.h"
#include "Adafruit_ST7789.h"

#define DEBUG
#undef DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(t_input) print_to_screenln(t_input);
#else
#define DEBUG_PRINT(t_input);
#endif

Servo myservo;
int enc_value = 55;
std::chrono::system_clock::time_point now;
bool start_centrifuge = false;

//Rotary encoder
AiEsp32RotaryEncoder my_enc = AiEsp32RotaryEncoder(ENCODER_PIN_1, ENCODER_PIN_2, ENCODER_PIN_SW, ENCODER_PIN_VCC);
int32_t rpm_setp  = -999;

//Display
Adafruit_ST7789 my_display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// Print to screen
void clear_screen(){
	my_display.setCursor(0,0);
	my_display.fillScreen(ST77XX_BLACK);
}
void print_to_screenln(String t_inp_string){
	Serial.println(t_inp_string);
	my_display.println(t_inp_string);
}
void print_to_screen(String t_inp_string){
	Serial.print(t_inp_string);
	my_display.print(t_inp_string);
}
//end print to screen

//Read speed
int speed_read_value;
void read_speed(){
	//Speed sensor init
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
			print_to_screenln("Sensor spd::" + String((speed_counter * 60000)/ time_passed) + " rpm");
			speed_read_at_time = std::chrono::system_clock::now();
			speed_counter = 0;
		}
		delay(1);
	}
}
//Create a task for speed read
std::thread read_speed_thread(read_speed);
//END Read speed


void servo_manipulation(){
	
	if (enc_value > 65){
		enc_value = 55;
	}
	print_to_screenln("Enc value :: " + String(enc_value));
	myservo.write(enc_value); 
}


void setup() {
	//Start the serial readout
	Serial.begin(115200);
	//Start the spi
	SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
	//Start the display
	my_display.init(240, 240, SPI_MODE2);
	my_display.fillScreen(ST77XX_BLACK);
	my_display.setRotation(3);
	my_display.setTextSize(3);
	//Set the servo
	myservo.attach(SERVO_PIN);
	//we must initialize rorary encoder 
	my_enc.begin();
	my_enc.setup([]{my_enc.readEncoder_ISR();});
	//optionally we can set boundaries and if values should cycle or not
	my_enc.setBoundaries(0, 10, true); //minValue, maxValue, cycle values (when max go to min and vice versa)
		
	pinMode (LED_BUILTIN, OUTPUT);
	pinMode (POWER_SPEED_SENSOR, OUTPUT);
	digitalWrite(POWER_SPEED_SENSOR, HIGH);
}

void loop() {
	static int32_t my_enc_delta = my_enc.readEncoder();
	DEBUG_PRINT("Read data ::" + String (my_enc_delta));
  	if (my_enc.currentButtonState() == BUT_PUSHED) {
		print_to_screenln("My button BUT_PUSHED");
		start_centrifuge = !start_centrifuge;
		
		enc_value +=1 ; // delete me
	}

	if (start_centrifuge){
		servo_manipulation();
	} else {
		myservo.write(0);
	}

	if (my_enc_delta != 0) {
		rpm_setp += my_enc_delta;
		print_to_screenln("Speed is: " + String(rpm_setp) + "rpm");
	}

	delay(100);
	my_enc.enable();
	clear_screen();
}