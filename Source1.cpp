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
    float acc_linear_total_samples[100];  //array of 100 samples
    float acc_linear_total_avg; // average of the last 100 linear_total values
}sensor_data;

int state = 0;
int launch_state = 0;
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
    pinMode(PIN_SWITCH, INPUT);
    pinMode(BUZZER, OUTPUT);

    //Open a new file
    dataFile = SD.open("data.csv", FILE_WRITE);

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
    sensor_data.acc_linear_total = sqrt(pow(sensor_data.acc_linear[0], 2) + pow(sensor_data.acc_linear[1], 2) + pow(sensor_data.acc_linear[2], 2));

    //Calculate a moving average of the last 100 linear acceleration total values
    //shift all values in the array to the left
    for (int i = 0; i < 99; i++) {
        sensor_data.acc_linear_total_samples[i] = sensor_data.acc_linear_total_samples[i + 1];
    }
    //add the new value to the end of the array
    sensor_data.acc_linear_total_samples[99] = sensor_data.acc_linear_total;
    //calculate the average of the array
    sensor_data.acc_linear_total_avg = 0;
    for (int i = 0; i < 100; i++) {
        sensor_data.acc_linear_total_avg += sensor_data.acc_linear_total_samples[i];
    }
    sensor_data.acc_linear_total_avg = sensor_data.acc_linear_total_avg / 100;
}

void loop() {
    //Update sensor data
    update_sensor_data();

    //Write sensor data to SD card if state > 0 and != 9
    if (state > 0 && state != 9) {
		write_sensor_data();
	}
	//State Machine
	switch (state) {
		case 0:
			//State 0: Idle
			//Check if the button is pressed
			if (1 == 1) {
				//Button is pressed, go to state 1
                delay(1000);
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
                tone(BUZZER, 1000);
                delay(50);
                noTone(BUZZER);
                delay(50);
                tone(BUZZER, 1000);
                delay(50);
                noTone(BUZZER);
                delay(50);
                tone(BUZZER, 1000);
                delay(50);
                noTone(BUZZER);
                
				//Start Timer
				int time_at_launch = millis();
                break;
			}
			else {
				//Button is not pressed anymore, go back to state 0
				state = 0;
				//One long Beep to indicate that the rocket is not armed
                tone(BUZZER, 400);
				delay(1000);
                noTone(BUZZER);
			}
			break;
		case 2:
			//State 2: Launch Detect
			if sensor_data.acc_linear_total_avg > 20 {
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
					//Launch State 0: No control for 5 seconds
                    if (time_at_launch - sensor_data.time > 5000) {
                        launch_state = 1;
					}
                    break;
                case 1:
					//Launch State 1: Estimate Apogee
                    // Apogee at < 5 m/s^2
                    if (sensor_data.acc_linear_total_avg < 5) {
                        state = 4;
					}
					break;
			}
		case 4:
			//State 4: Fire Ejection Charge 1
            digitalWrite(PY1, HIGH);
			state = 5;
            int time_at_EjectionCharge1 = millis();
			break;
		case 5:
			//State 5: Check if the rocket is descelerating
            //Check if 2 second has passed since ejection charge 1
            if (millis() - time_at_EjectionCharge1 > 2000) {
                //2 seconds have passed, turn off ejection charge 1
                digitalWrite(PY1, LOW);
                //Check if the rocket is descelerating
                if (sensor_data.acc_linear_total_avg < 5) {
                    //Rocket is not descelerating, go to state 6
                    state = 6;
                }
                else
                {
                    //Rocket is descelerating, go to state 7
                    state = 7;
                }
				break;
			}
            break;
		case 6:
			//State 6: Fire Ejection Charge 2
            digitalWrite(PY2, HIGH);
            state = 7;
            int time_at_EjectionCharge2 = millis();
			break;
		case 7:
			//State 7: Data Preservation
			//Close file
            dataFile.close();
			//Start new file
            dataFile = SD.open("data_after.csv", FILE_WRITE);
            state = 8;
            break;
		case 8:
			//State 8: Landing Detect
            //Check if 1 second has passed since ejection charge 2
            if (millis() - time_at_EjectionCharge2 > 1000) {
				//1 second has passed, turn off ejection charge 2
				digitalWrite(PY2, LOW);
			}
            //Check if the acceleration < 1 m/s^2
            if (acc_linear_total_avg < 1) {
				//Landed, go to state 9
                int time_at_landing = millis();
				state = 9;
			}
            break;
        case 9:
			//State 9: Beep for 20 minutes
            //Close file
            dataFile.close();
            while (millis() - time_at_landing < 1200000) {
				tone(BUZZER, 1000);
				delay(50);
				noTone(BUZZER);
				delay(50);
			}
	}
}

