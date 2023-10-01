// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>


// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
//#define BNO08X_RESET -1

//Adafruit_BNO08x  bno08x(BNO08X_RESET);
Adafruit_BNO08x  bno08x;
sh2_SensorValue_t sensorValue;

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


void loop() {

    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }

    float accelerometer[3] = { 0 };  // [ax, ay, az]
    float magnetometer[3] = { 0 };   // [mx, my, mz]
    float gyroscope[3] = { 0 };      // [gx, gy, gz]
    float acc_linear[3] = { 0 };
    float rotation_vector[4] = { 0 };  // [w, x, y, z]
    float sum;

    switch (sensorValue.sensorId) {
    case SH2_ACCELEROMETER:
        accelerometer[0] = sensorValue.un.accelerometer.x;
        accelerometer[1] = sensorValue.un.accelerometer.y;
        accelerometer[2] = sensorValue.un.accelerometer.z;
        break;

    case SH2_GYROSCOPE_CALIBRATED:
        gyroscope[0] = sensorValue.un.gyroscope.x;
        gyroscope[1] = sensorValue.un.gyroscope.y;
        gyroscope[2] = sensorValue.un.gyroscope.z;
        break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
        magnetometer[0] = sensorValue.un.magneticField.x;
        magnetometer[1] = sensorValue.un.magneticField.y;
        magnetometer[2] = sensorValue.un.magneticField.z;
        break;

    case SH2_LINEAR_ACCELERATION:
        acc_linear[0] = sensorValue.un.linearAcceleration.x;
        acc_linear[1] = sensorValue.un.linearAcceleration.y;
        acc_linear[2] = sensorValue.un.linearAcceleration.z;
        break;

    case SH2_ROTATION_VECTOR:
        Serial.print("Rotation Vector - r: ");
        Serial.print(sensorValue.un.rotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.rotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.rotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.rotationVector.k);
        break;

        //Serial.print("x: ");
        //Serial.println(acc_linear[0]);
        //Serial.print("y: ");
        //Serial.println(acc_linear[1]);
        //Serial.print("z: ");
        //Serial.println(acc_linear[2]);
        //Serial.print("Sum: ");
        //sum = acc_linear[0] + acc_linear[1] + acc_linear[2];
        //if (sum > 0.2){
        //  Serial.println(sum);
        Serial.println("Quaternion: ");
        Serial.println("x" + String(rotation_vector[0]) + "y");
        Serial.println(rotation_vector[1]);
        Serial.println(rotation_vector[2]);
        Serial.println(rotation_vector[3]);
    }
    delay(200);
}


void loop() {
    delay(10);

    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }

    switch (sensorValue.sensorId) {

    case SH2_ACCELEROMETER:
        Serial.print("Accelerometer - x: ");
        Serial.print(sensorValue.un.accelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.accelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.accelerometer.z);
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        Serial.print("Gyro - x: ");
        Serial.print(sensorValue.un.gyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.gyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.gyroscope.z);
        break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        Serial.print("Magnetic Field - x: ");
        Serial.print(sensorValue.un.magneticField.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.magneticField.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.magneticField.z);
        break;
    case SH2_LINEAR_ACCELERATION:
        Serial.print("Linear Acceration - x: ");
        Serial.print(sensorValue.un.linearAcceleration.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.linearAcceleration.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.linearAcceleration.z);
        break;
    case SH2_GRAVITY:
        Serial.print("Gravity - x: ");
        Serial.print(sensorValue.un.gravity.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.gravity.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.gravity.z);
        break;
    case SH2_ROTATION_VECTOR:
        Serial.print("Rotation Vector - r: ");
        Serial.print(sensorValue.un.rotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.rotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.rotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.rotationVector.k);
        break;
    case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        Serial.print("Geo-Magnetic Rotation Vector - r: ");
        Serial.print(sensorValue.un.geoMagRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.geoMagRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.geoMagRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.geoMagRotationVector.k);
        break;

    case SH2_GAME_ROTATION_VECTOR:
        Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);
        break;

    case SH2_STEP_COUNTER:
        Serial.print("Step Counter - steps: ");
        Serial.print(sensorValue.un.stepCounter.steps);
        Serial.print(" latency: ");
        Serial.println(sensorValue.un.stepCounter.latency);
        break;

    case SH2_STABILITY_CLASSIFIER: {
        Serial.print("Stability Classification: ");
        sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
        switch (stability.classification) {
        case STABILITY_CLASSIFIER_UNKNOWN:
            Serial.println("Unknown");
            break;
        case STABILITY_CLASSIFIER_ON_TABLE:
            Serial.println("On Table");
            break;
        case STABILITY_CLASSIFIER_STATIONARY:
            Serial.println("Stationary");
            break;
        case STABILITY_CLASSIFIER_STABLE:
            Serial.println("Stable");
            break;
        case STABILITY_CLASSIFIER_MOTION:
            Serial.println("In Motion");
            break;
        }
        break;
    }

    case SH2_RAW_ACCELEROMETER:
        Serial.print("Raw Accelerometer - x: ");
        Serial.print(sensorValue.un.rawAccelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawAccelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawAccelerometer.z);
        break;
    case SH2_RAW_GYROSCOPE:
        Serial.print("Raw Gyro - x: ");
        Serial.print(sensorValue.un.rawGyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawGyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawGyroscope.z);
        break;
    case SH2_RAW_MAGNETOMETER:
        Serial.print("Raw Magnetic Field - x: ");
        Serial.print(sensorValue.un.rawMagnetometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawMagnetometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawMagnetometer.z);
        break;

    case SH2_SHAKE_DETECTOR: {
        Serial.print("Shake Detector - shake detected on axis: ");
        sh2_ShakeDetector_t detection = sensorValue.un.shakeDetector;
        switch (detection.shake) {
        case SHAKE_X:
            Serial.println("X");
            break;
        case SHAKE_Y:
            Serial.println("Y");
            break;
        case SHAKE_Z:
            Serial.println("Z");
            break;
        default:
            Serial.println("None");
            break;
        }
    }

    case SH2_PERSONAL_ACTIVITY_CLASSIFIER: {

        sh2_PersonalActivityClassifier_t activity =
            sensorValue.un.personalActivityClassifier;
        Serial.print("Activity classification - Most likely: ");
        printActivity(activity.mostLikelyState);
        Serial.println("");

        Serial.println("Confidences:");
        // if PAC_OPTION_COUNT is ever > 10, we'll need to
        // care about page
        for (uint8_t i = 0; i < PAC_OPTION_COUNT; i++) {
            Serial.print("\t");
            printActivity(i);
            Serial.print(": ");
            Serial.println(activity.confidence[i]);
        }
    }
    }
}