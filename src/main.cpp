#include "defines.h"

#include "Adafruit_ST7789.h"
#include "AiEsp32RotaryEncoder.h"
#include "PID_v1.h"
#include "Servo.h"
#include <Arduino.h>
#include <SPI.h>
#include <chrono>
#include <thread>

#define DEBUG
#undef DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(t_input) print_to_screenln(t_input);
#else
#define DEBUG_PRINT(t_input) ;
#endif

Servo myservo;
std::chrono::system_clock::time_point now;
bool start_centrifuge = false;
int16_t my_enc_delta = 0;

// Display
Adafruit_ST7789 my_display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// Print to screen
void clear_screen() {
	my_display.setCursor(0, 0);
	my_display.fillScreen(ST77XX_BLACK);
}
void print_to_screenln(String t_inp_string) {
	Serial.println(t_inp_string);
	my_display.println(t_inp_string);
}
void print_to_screen(String t_inp_string) {
	Serial.print(t_inp_string);
	my_display.print(t_inp_string);
}
// end print to screen

// Rotary encoder
AiEsp32RotaryEncoder my_enc = AiEsp32RotaryEncoder(ENCODER_PIN_CLK, ENCODER_PIN_DT, ENCODER_PIN_SW, ENCODER_PIN_VCC);

// PID
double pid_setpoint = START_SETP_RPM, pid_input, pid_output;
double pid_kp = 0.005, pid_ki = 0.01, pid_kd = 0;
double pid_kp_low = 0.001, pid_ki_low = 0.003, pid_kd_low = 0;
PID my_pid(&pid_input, &pid_output, &pid_setpoint, pid_kp, pid_ki, pid_kd, DIRECT);
void pid_manipulation() {
	while (1) {
		if (start_centrifuge) {
			my_pid.Compute();
		}
		delay(500);
	}
}
// Create a task for pid read
std::thread pid_manipulation_thread(pid_manipulation);
// END PID

// Read speed
int speed_read_value;
void read_speed() {
	// Speed sensor init
	bool was_covered = false;
	long speed_counter = 0;
	int time_passed;
	const auto speed_timer_sleep = std::chrono::microseconds(1000);
	std::chrono::system_clock::time_point speed_read_at_time = std::chrono::system_clock::now();
	while (1) {
		speed_read_value = analogRead(SPEED_READ_PIN);
		if (speed_read_value < 3400 && !was_covered) {
			was_covered = true;
			speed_counter++;
		} else if (speed_read_value > 3400) {
			was_covered = false;
		}
		now = std::chrono::system_clock::now();
		time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(now - speed_read_at_time).count();
		if (time_passed >= 1000) {
			pid_input = (speed_counter * 60000) / time_passed;
			print_to_screenln("Sensor spd::" + String(pid_input) + " rpm");
			speed_read_at_time = std::chrono::system_clock::now();
			speed_counter = 0;
		}
		delay(1);
	}
}
// Create a task for speed read
std::thread read_speed_thread(read_speed);
// END Read speed

void setup() {
	// Start the serial readout
	Serial.begin(115200);
	// Start the spi
	SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
	// Start the display
	my_display.init(240, 240, SPI_MODE2);
	my_display.fillScreen(ST77XX_BLACK);
	my_display.setRotation(3);
	my_display.setTextSize(3);
	// Set the servo
	myservo.attach(ESC_PIN);
	// we must initialize rorary encoder
	my_enc.begin();
	my_enc.setup([] { my_enc.readEncoder_ISR(); });
	// optionally we can set boundaries and if values should cycle or not
	my_enc.setBoundaries(0, 100, false); // minValue, maxValue, cycle values (when max go to min and vice versa)

	// turn the PID on
	my_pid.SetOutputLimits(-10, 300);
	my_pid.SetMode(AUTOMATIC);
	pinMode(SPEED_READ_PIN, ANALOG);
}

void loop() {
	clear_screen();
	my_enc_delta = my_enc.encoderChanged();
	if (my_enc.currentButtonState() == BUT_PUSHED) {
		print_to_screenln("My button BUT_PUSHED");
		start_centrifuge = !start_centrifuge;
	}

	if (start_centrifuge) {
		myservo.writeMicroseconds(ESC_MINIMUM_MS + pid_output);
	} else {
		myservo.write(0);
	}

	if (my_enc_delta != 0) {
		pid_setpoint += 50 * double(my_enc_delta);
		print_to_screenln("Speed setp: " + String(pid_setpoint) + "rpm");
	}
	if ((abs(pid_setpoint - pid_input)) > 70) {
		my_pid.SetTunings(pid_kp, pid_ki, pid_kd);
	} else {
		my_pid.SetTunings(pid_kp_low, pid_ki_low, pid_kd_low);
	}
	DEBUG_PRINT("setp::" + String(pid_setpoint) + " outp::" + String(ESC_MINIMUM_MS + pid_output) + " inp::" + String(pid_input));
	delay(100);
}