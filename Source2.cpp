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
    float acc_linear[3];
    float rotation_vector[4];  // [real, i, j, k]
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
}


void loop() {

    //3 mode state machine
    switch (state) {
    case 0:
        //Open a new file
        dataFile = SD.open("data.csv", FILE_WRITE);
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
        state = 1;
        break;
    case 1:
        //Write sensor data to file
        Serial.println("Updating Sensor data and writing");
        update_sensor_data();
        if (dataFile) {
            //Make compile data to string
            str out = sensor_data.time + "," 
                + sensor_data.accelerometer[0] + "," 
                + sensor_data.accelerometer[1] + "," 
                + sensor_data.accelerometer[2] + "," 
                + sensor_data.magnetometer[0] + "," 
                + sensor_data.magnetometer[1] + "," 
                + sensor_data.magnetometer[2] + "," 
                + sensor_data.gyroscope[0] + "," 
                + sensor_data.gyroscope[1] + "," 
                + sensor_data.gyroscope[2] + "," 
                + sensor_data.acc_linear[0] + "," 
                + sensor_data.acc_linear[1] + "," 
                + sensor_data.acc_linear[2] + "," 
                + sensor_data.rotation_vector[0] + "," 
                + sensor_data.rotation_vector[1] + "," 
                + sensor_data.rotation_vector[2] + "," 
                + sensor_data.rotation_vector[3];
            dataFile.println(out);
        }
        if (sensor_data.time > 60000) {
            state = 2;
        }

        break;
    case 2:
        //Close dataFile
        dataFile.close();
        state = 3;
        break;
    case 3:
        //Turn buzzer on
        tone(BUZZER, 400);
    }

}
