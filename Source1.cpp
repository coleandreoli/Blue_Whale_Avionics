// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>


// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
//#define BNO08X_RESET -1

//Adafruit_BNO08x  bno08x(BNO08X_RESET);
Adafruit_BNO08x  bno08x;
sh2_SensorValue_t sensorValue;

//Setup struct for sensor data
struct sensor_data {
	float accelerometer[3] = { 0 };  // [ax, ay, az]
	float magnetometer[3] = { 0 };   // [mx, my, mz]
	float gyroscope[3] = { 0 };      // [gx, gy, gz]
	float acc_linear[3] = { 0 };
	float rotation_vector[4] = { 0 };  // [w, x, y, z]
};

void setup(void) {
	Serial.begin(115200);
	while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

	Serial.println("Adafruit BNO08x test!");
	//pinMode(18, INPUT_PULLUP);
	//pinMode(19, INPUT_PULLUP);

	// Try to initialize!
	if (!bno08x.begin_I2C()) {
		//if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
		//if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
		Serial.println("Failed to find BNO08x chip");
		while (1) { delay(10); }
	}
	Serial.println("BNO08x Found!");

	setReports();

	Serial.println("Reading events");
	delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
	Serial.println("Setting desired reports");
	if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
		Serial.println("Could not enable accelerometer");
	}
	if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
		Serial.println("Could not enable gyroscope");
	}
	if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
		Serial.println("Could not enable magnetic field calibrated");
	}
	if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
		Serial.println("Could not enable linear acceleration");
	}
	if (!bno08x.enableReport(SH2_GRAVITY)) {
		Serial.println("Could not enable gravity vector");
	}
	if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
		Serial.println("Could not enable rotation vector");
	}
	if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
		Serial.println("Could not enable step counter");
	}
	if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
		Serial.println("Could not enable stability classifier");
	}
	if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
		Serial.println("Could not enable raw accelerometer");
	}
	if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
		Serial.println("Could not enable raw gyroscope");
	}
	if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
		Serial.println("Could not enable raw magnetometer");
	}
}



//update sensor data
void update_sensor_data(struct sensor_data* data) {
	switch (sensorValue.sensorId) {
	case SH2_ACCELEROMETER:
		//update accelerometer data
		data->accelerometer[0] = sensorValue.un.accelerometer.x;
		data->accelerometer[1] = sensorValue.un.accelerometer.y;
		data->accelerometer[2] = sensorValue.un.accelerometer.z;
		break;
	case SH2_MAGNETIC_FIELD_CALIBRATED:
		//update magnetometer data
		data->magnetometer[0] = sensorValue.un.magnetometer.x;
		data->magnetometer[1] = sensorValue.un.magnetometer.y;
		data->magnetometer[2] = sensorValue.un.magnetometer.z;
		break;
	case SH2_GYROSCOPE_CALIBRATED:
		//update gyroscope data
		data->gyroscope[0] = sensorValue.un.gyroscope.x;
		data->gyroscope[1] = sensorValue.un.gyroscope.y;
		data->gyroscope[2] = sensorValue.un.gyroscope.z;
		break;
	case SH2_LINEAR_ACCELERATION:
		//update linear acceleration data
		data->acc_linear[0] = sensorValue.un.linear_acceleration.x;
		data->acc_linear[1] = sensorValue.un.linear_acceleration.y;
		data->acc_linear[2] = sensorValue.un.linear_acceleration.z;
		break;
	case SH2_ROTATION_VECTOR:
		//update rotation vector data
		data->rotation_vector[0] = sensorValue.un.rotation_vector.w;
		data->rotation_vector[1] = sensorValue.un.rotation_vector.x;
		data->rotation_vector[2] = sensorValue.un.rotation_vector.y;
		data->rotation_vector[3] = sensorValue.un.rotation_vector.z;
		break;
	}
}


void loop() {

	//State Machine
	switch (state) {
		case 0:
			//State 0: Idle
			//Check if the button is pressed
			if (1 == 1) {
				//Button is pressed, go to state 1
				state = 1;
			}
			break;
		case 1:
			//State 1: Armed
			//Check if the button is still pressed
			if (1 == 1) {
				//Button is still pressed, go to state 2
				state = 2;
				//Beep 3 times to indicate that the rocket is armed
				for (int i = 0; i < 3; i++) {
					digitalWrite(BUZZER_PIN, HIGH);
					delay(100);
					digitalWrite(BUZZER_PIN, LOW);
					delay(100);
				}
				//Start Data Logging
				//Start Timer
				int time_at_launch = milis();
			}
			else {
				//Button is not pressed anymore, go back to state 0
				state = 0;
				//One long Beep to indicate that the rocket is not armed
				digitalWrite(BUZZER_PIN, HIGH);
				delay(1000);
				digitalWrite(BUZZER_PIN, LOW);
			}
			break;
		case 2:
			//State 2: Launch Detect
			sum = ;
			if sum > 2 {
				//Launch Detected, go to state 3
				state = 3;
			}
			break;
		case 3:
			//State 3: Detecting Apogee
			//Check if 16 seconds have passed
			if (milis() - time_at_launch > 16000) {
				//16 seconds have passed, go to state 4
				state = 4;
				break;
			}
			switch (launch_state) {
				case 0:
					//Launch State 0: Detecting Apogee
					//Check if the acceleration is negative
					if (acc_linear[0] + acc_linear[1] + acc_linear[2] < 0) {
						//Acceleration is negative, go to Launch State 1
						launch_state = 1;
					}
					break;
				case 1:
					//Launch State 1: Detecting Apogee
					//Check if the acceleration reaches
					if (acc_linear[0] + acc_linear[1] + acc_linear[2] > 0) {
						//Acceleration is positive, go to Launch State 2
						launch_state = 2;
					}
					break;
				case 2:
					//Launch State 2: Detecting Apogee
					//If 3 second average acceleration is less than 2.5g, detected cruise
					if (three_sec_avg_acc < 2.5) {
						//Acceleration is less than 2.5g, go to Launch State 3
						launch_state = 3;
					}
					break;

			}
			
			if (digitalRead(BUTTON_PIN) == HIGH) {
				//Button is still pressed, go to state 4
				state = 4;
			}

			break;
		case 4:
			//State 4: Fire Ejection Charge 1
			
			if (digitalRead(BUTTON_PIN) == HIGH) {
				//Button is still pressed, go to state 5
				state = 5;
			}

			break;
		case 5:
			//State 5: Check if the rocket is descelerating
			
			if (digitalRead(BUTTON_PIN) == HIGH) {
				//Button is still pressed, go
			}
		case 6:
			//State 6: Fire Ejection Charge 2
			
		case 7:
			//State 7: Data Preservation + Landing Detect
			//Close file
			//Start new file
		case 8:
			//State 8: Beep for 10 minutes

	}
}

