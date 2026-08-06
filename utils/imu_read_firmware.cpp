#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float roll_deg = 0, pitch_deg = 0;
float temp_c = 0;

/*
 * parses data output of an ESP32 to CSV for compatibility with imu_read.py
 */

void enableReports() {
    bno.enableReport(SH2_GAME_ROTATION_VECTOR,   5000);  // 200Hz
    bno.enableReport(SH2_ACCELEROMETER,          5000);  // 200Hz
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED,   5000);  // 200Hz
}

// Convert quaternion to roll/pitch on the ESP32 side
void quatToRollPitch(float w, float x, float y, float z,
                     float &roll, float &pitch) {
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    roll = atan2f(sinr, cosr) * 180.0f / PI;

    float sinp = 2.0f * (w * y - z * x);
    sinp = constrain(sinp, -1.0f, 1.0f);
    pitch = asinf(sinp) * 180.0f / PI;
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    Wire.begin(8, 9);
    Wire.setClock(400000);
    delay(100);

    if (!bno.begin_I2C(0x4A, &Wire)) {
        while (1) {
            Serial.println("BNO085 not found");
            delay(1000);
        }
    }
    enableReports();
    Serial.println("serial_ok");  // imu_stream.py skips this line
}

void loop() {
    if (bno.wasReset()) {
        enableReports();
    }

    if (!bno.getSensorEvent(&sensorValue)) {
        return;
    }

    switch (sensorValue.sensorId) {

        case SH2_GAME_ROTATION_VECTOR: {
            float w = sensorValue.un.gameRotationVector.real;
            float x = sensorValue.un.gameRotationVector.i;
            float y = sensorValue.un.gameRotationVector.j;
            float z = sensorValue.un.gameRotationVector.k;
            quatToRollPitch(w, x, y, z, roll_deg, pitch_deg);
            break;
        }

        case SH2_ACCELEROMETER: {
            // Convert m/s² to g
            ax_g = sensorValue.un.accelerometer.x / 9.80665f;
            ay_g = sensorValue.un.accelerometer.y / 9.80665f;
            az_g = sensorValue.un.accelerometer.z / 9.80665f;
            break;
        }

        case SH2_GYROSCOPE_CALIBRATED: {
            // Convert rad/s to deg/s
            gx_dps = sensorValue.un.gyroscope.x * 180.0f / PI;
            gy_dps = sensorValue.un.gyroscope.y * 180.0f / PI;
            gz_dps = sensorValue.un.gyroscope.z * 180.0f / PI;

            // Emit CSV when gyro updates (ties output rate to gyro rate)
            // Format: t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg
            Serial.print(millis());     Serial.print(",");
            Serial.print(ax_g, 5);     Serial.print(",");
            Serial.print(ay_g, 5);     Serial.print(",");
            Serial.print(az_g, 5);     Serial.print(",");
            Serial.print(gx_dps, 5);   Serial.print(",");
            Serial.print(gy_dps, 5);   Serial.print(",");
            Serial.print(gz_dps, 5);   Serial.print(",");
            Serial.print(roll_deg, 3); Serial.print(",");
            Serial.println(pitch_deg, 3);
            break;
        }
    }
}
