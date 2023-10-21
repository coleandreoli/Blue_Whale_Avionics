#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BNO08X_RESET -1
#define PY1 2
#define PY2 3
#define PIN_SWITCH 4
#define BUZZER 7
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
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
    float pressure;
    float temperature;
    float altitude;
}sensor_data;

int state = 0;
int launch_state = 0;
int record_state = 0;
int time_at_launch;
int time_at_apogee;
int time_at_landing;
int time_at_EjectionCharge1;
int time_at_EjectionCharge2;
File dataFile;

void setup() {
    Serial.begin(115200);
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


    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
        //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1);
    }
    Serial.println("BMP3 Found!");

    setReports();

    // Set up oversampling and filter initialization
    // Set for "Drone" settings, see datasheet for details
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);


    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD Found!");
    // put your setup code here, to run once:
    pinMode(PY1, OUTPUT);
    pinMode(PY2, OUTPUT);
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);

    //Open a new file
    dataFile = SD.open("data.csv", FILE_WRITE);
    Serial.println("Completed Setup");

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
    //Serial.println("write_sensor_data");
    String out;
    if (dataFile) {
        // Append values to the 'out' string with commas
        out = String(sensor_data.time) + ","
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
            + String(sensor_data.rotation_vector[3]) + ","
            + String(sensor_data.acc_linear_total) + ","
            + String(sensor_data.acc_linear_total_avg) + ","
            + String(state) + ","
            + String(launch_state);
    }
    //write BMP data if state = 1
    if (record_state == 1) {
        out = out + "," + String(sensor_data.pressure) + "," + String(sensor_data.temperature) + "," + String(sensor_data.altitude);
    }

    // Write the 'out' string to the file
    dataFile.println(out);
}

//update sensor data
void update_sensor_data() {
    if (bno08x.wasReset()) {
        Serial.println("sensor was reset ");
        setReports();
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }
    //Serial.println("update_sensor_data");
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
    ////update BMP data after recovery sequence
    if (record_state == 1) {
        
        //C
        sensor_data.temperature = bmp.temperature;
        //Pa
        sensor_data.pressure = bmp.pressure;
        //m
        sensor_data.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }
}
void loop() {
    //Update sensor data
    update_sensor_data();

    //Write sensor data to SD card if state > 0 and != 9
    if (state > 0 && state != 10) {
        write_sensor_data();
    }
    //State Machine
    switch (state) {
    case 0:
        //State 0: Idle
        //Check if the button is pressed
        if (digitalRead(PIN_SWITCH) == 0) {
            //Button is pressed, go to state 1
            delay(1000);
            state = 1;
            Serial.println("Idle > State 1");
        }
        break;
    case 1:
        //State 1: Armed
        //Check if the button is still pressed
        if (digitalRead(PIN_SWITCH) == 0) {
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

            //Button is still pressed, go to state 2
            state = 2;
            Serial.println("Armed > State 2");
        }else{
            //Button is not pressed anymore, go back to state 0
            state = 0;
            //Turn off BMP data recording
            record_state = 0;
            Serial.println("Unarmed > State 0");
            //Descending chirp to indicate that the rocket is not armed
            tone(BUZZER, 1200);
            delay(50);
            tone(BUZZER, 800);
            delay(50);
            tone(BUZZER, 600);
            delay(50);
            noTone(BUZZER);
        }

        break;
    case 2:
        //State 2: Launch Detect
        if (sensor_data.acc_linear_total_avg > 20) {
            //Launch Detected, go to state 3
            time_at_launch = millis();
            state = 3;
            //Turn on BMP data recording
            record_state = 1;
            Serial.println("Launch Detected > State 3");
            tone(BUZZER, 400);
        }

        if (digitalRead(PIN_SWITCH) == 1) {
            state = 1;
            //Turn off BMP data recording
            record_state = 0;
            Serial.println("Unarmed > State 1");
            //Descending chirp to indicate that the rocket is not armed
            tone(BUZZER, 1200);
            delay(50);
            tone(BUZZER, 800);
            delay(50);
            tone(BUZZER, 600);
            delay(50);
            noTone(BUZZER);
        }

        break;
    case 3:
        //State 3: Detecting Apogee
        //Check if 16 seconds have passed
        if (millis() - time_at_launch > 16000) {
            //16 seconds have passed, go to state 4
            Serial.println("16 seconds have passed > State 4");
            state = 4;
            break;
        }
        if (launch_state == 0) {
            //Launch State 0: No control for 5 seconds
            if (millis() - time_at_launch > 5000) {
                launch_state = 1;
                Serial.println("5 seconds after Launch > Launch State 1");
            }
        }
        else
        {
            //Launch State 1: Estimate Apogee
            // Apogee at < 5 m/s^2
            if (sensor_data.acc_linear_total_avg < 5) {
                Serial.println("Apogee detected > State 4");
                state = 4;
            }
        }
        break;
    case 4:
        //State 4: Fire Ejection Charge 1
        noTone(BUZZER);
        digitalWrite(PY1, HIGH);
        state = 5;
        Serial.println("Ejection Charge 1 fired > State 5");
        time_at_EjectionCharge1 = millis();
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
                Serial.println("Rocket is not descelerating > State 6");
            }
            else
            {
                //Rocket is descelerating, go to state 8
                state = 8;
                Serial.println("Rocket is descelerating > State 8");
            }
            break;
        }
        break;
    case 6:
        //State 6: Fire Ejection Charge 2
        digitalWrite(PY2, HIGH);
        state = 7;
        Serial.println("Ejection Charge 2 fired > State 7");
        time_at_EjectionCharge2 = millis();
        break;
    case 7:
        //State 7: 1 second after Ejection Charge 2
        //Check if 1 second has passed since ejection charge 2
        if (millis() - time_at_EjectionCharge2 > 1000) {
            //1 second has passed, turn off ejection charge 2
            digitalWrite(PY2, LOW);
            state = 8;
        }
        break;
    case 8:
        //State 8: Data Preservation
        //Close file
        dataFile.close();
        //Start new file
        dataFile = SD.open("data_after.csv", FILE_WRITE);
        state = 9;
        Serial.println("Data Preservation > State 8");
        break;
    case 9:
        //State 9: Detecting Landing
        //Check if the acceleration < 1 m/s^2
        if (sensor_data.acc_linear_total_avg < 1) {
            //Landed, go to state 9
            time_at_landing = millis();
            state = 10;
            Serial.println("Landed > State 10");
        }
        break;
    case 10:
        //State 9: Beep for 20 minutes
        //Close file
        dataFile.close();
        Serial.println("Beeping for 20 mins");
        while (millis() - time_at_landing < 1200000) {
            tone(BUZZER, 200);
            delay(500);
            noTone(BUZZER);
            delay(500);
        }
    }
}

