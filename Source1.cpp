#include <Adafruit_BNO08x.h>
#include <SD.h>
#include <SPI.h>


#define BNO08X_RESET -1
#define PY1 2
#define PY2 3
#define PIN_SWITCH 4
#define BUZZER 7
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


struct {
    int time;
    float accelerometer[3];  // [ax, ay, az]
    float magnetometer[3];   // [mx, my, mz]
    float gyroscope[3];      // [gx, gy, gz]
    float acc_linear[3];     // [ax, ay, az]
    float rotation_vector[4];  // [real, i, j, k]
    float acc_linear_total; 	// sqrt(ax^2 + ay^2 + az^2)
    float acc_linear_samples;
    float acc_linear_total_avg_100; // average of the last 100 linear_total values
}sensor_data;

int state = 0;
File dataFile;

void setup() {
    Serial.begin(115200);
    //while (!Serial)
    //  delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit BNO08x test!");

    // Try to initialize!
    if (!bno08x.begin_I2C()) {
        // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
        // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("BNO08x Found!");

    setReports();

    Serial.println("Reading events");

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }

    // put your setup code here, to run once:
    pinMode(PY1, OUTPUT);
    pinMode(PY2, OUTPUT);
    pinMode(PIN_SWITCH, OUTPUT);
    pinMode(BUZZER, OUTPUT);

}

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
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("Could not enable rotation vector");
    }
}

void write_sensor_data() {
    Serial.println("write_sensor_data");
    if (dataFile) {
        // Append values to the 'out' string with commas
        String out = String(sensor_data.time) + ","
            + String(sensor_data.accelerometer[0]) + ","
            + String(sensor_data.accelerometer[1]) + ","
            + String(sensor_data.accelerometer[2]) + ","
            + String(sensor_data.magnetometer[0]) + ","
            + String(sensor_data.magnetometer[1]) + ","
            + String(sensor_data.magnetometer[2]) + ","
            + String(sensor_data.gyroscope[0]) + ","
            + String(sensor_data.gyroscope[1]) + ","
            + String(sensor_data.gyroscope[2]) + ","
            + String(sensor_data.acc_linear[0]) + ","
            + String(sensor_data.acc_linear[1]) + ","
            + String(sensor_data.acc_linear[2]) + ","
            + String(sensor_data.rotation_vector[0]) + ","
            + String(sensor_data.rotation_vector[1]) + ","
            + String(sensor_data.rotation_vector[2]) + ","
            + String(sensor_data.rotation_vector[3]);

        // Write the 'out' string to the file
        dataFile.println(out);
    }

}

//update sensor data
void update_sensor_data() {
    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }
    sensor_data.time = millis();
    switch (sensorValue.sensorId) {
    case SH2_ACCELEROMETER:
        //update accelerometer data
        sensor_data.accelerometer[0] = sensorValue.un.accelerometer.x;
        sensor_data.accelerometer[1] = sensorValue.un.accelerometer.y;
        sensor_data.accelerometer[2] = sensorValue.un.accelerometer.z;
        break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        //update magnetometer data
        sensor_data.magnetometer[0] = sensorValue.un.magneticField.x;
        sensor_data.magnetometer[1] = sensorValue.un.magneticField.y;
        sensor_data.magnetometer[2] = sensorValue.un.magneticField.z;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        //update gyroscope data
        sensor_data.gyroscope[0] = sensorValue.un.gyroscope.x;
        sensor_data.gyroscope[1] = sensorValue.un.gyroscope.y;
        sensor_data.gyroscope[2] = sensorValue.un.gyroscope.z;
        break;
    case SH2_LINEAR_ACCELERATION:
        //update linear acceleration data
        sensor_data.acc_linear[0] = sensorValue.un.linearAcceleration.x;
        sensor_data.acc_linear[1] = sensorValue.un.linearAcceleration.y;
        sensor_data.acc_linear[2] = sensorValue.un.linearAcceleration.z;
        break;
    case SH2_ROTATION_VECTOR:
        //update rotation vector data
        sensor_data.rotation_vector[0] = sensorValue.un.rotationVector.real;
        sensor_data.rotation_vector[1] = sensorValue.un.rotationVector.i;
        sensor_data.rotation_vector[2] = sensorValue.un.rotationVector.j;
        sensor_data.rotation_vector[3] = sensorValue.un.rotationVector.k;
        break;
    }

    //Calculate total linear acceleration
    sensor_data.linear_total = sqrt(pow(sensor_data.acc_linear[0], 2) + pow(sensor_data.acc_linear[1], 2) + pow(sensor_data.acc_linear[2], 2));
    //Calculate average of 100 samples of linear acceleration
    //if less than 100 samples, append to array
    //if 100 samples, delete first element and append to array
    //sizeof(sensor_data.acc_linear_samples) = length of sensor_data.linear

    int samples = sizeof(sensor_data.acc_linear_samples);
    if (sizeof(sensor_data.acc_linear_samples) < 100) {
        sensor_data.acc_linear_samples[samples] = sensor_data.linear_total;
    }
    else {
        for (int i = 0; i < sizeof(sensor_data.acc_linear_samples) - 1; i++) {
            sensor_data.acc_linear_samples[i] = sensor_data.acc_linear_samples[i + 1];
        }
        sensor_data.acc_linear_samples[sizeof(sensor_data.acc_linear_samples) - 1] = sensor_data.linear_total;
    }

}


void loop() {

	//State Machine
	switch (state) {
		case 0:
			//State 0: Idle
			//Check if moving (restarted in flight)

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

