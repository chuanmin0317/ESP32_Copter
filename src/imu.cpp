#include "imu.h"
#include "MyCommon.h"

extern MPU9250 mpu9250;

void calibration()
{
    mpu9250.setGyroBias(-1.27, -0.38, -1.76);
    mpu9250.setAccBias(-4.69, 58.14, 83.74);
    mpu9250.setMagBias(04.54, 368.57, -327.77);
    mpu9250.setMagScale(0.66, 1.26, 1.43);
}

void MpuInit()
{
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    delay(250);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A8G;           // 加速度計量程 ±8g
    setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;        // 陀螺儀量程 ±1000°/s
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS; // 磁力計解析度 16位
    setting.gyro_fchoice = 0x03;                        // 啟用陀螺儀濾波
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;   // 陀螺儀低通濾波器 41Hz
    setting.accel_fchoice = 0x01;                       // 啟用加速度計濾波
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ; // 加速度計低通濾波器 45Hz

    if (!mpu9250.setup(0x68, setting))
    {
        while (1)
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    mpu9250.setFilterIterations(10);
    calibration();
}

void GetImuData(st_Mpu *mpu, st_Angle *angle)
{
    if (mpu9250.update())
    {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 6)
        {
            mpu->accX = mpu9250.getAccX();
            mpu->accY = mpu9250.getAccY();
            mpu->accZ = mpu9250.getAccZ();

            mpu->gyroX = mpu9250.getGyroX();
            mpu->gyroY = mpu9250.getGyroY();
            mpu->gyroZ = mpu9250.getGyroZ();

            angle->roll = mpu9250.getRoll();
            angle->pitch = mpu9250.getPitch();
            angle->yaw = mpu9250.getYaw();
            prev_ms = millis();
        }
    }
    Serial.print("roll: ");
    Serial.print(angle->roll);
    Serial.print(" pitch: ");
    Serial.print(angle->pitch);
    Serial.print(" yaw: ");
    Serial.println(angle->yaw);
}
