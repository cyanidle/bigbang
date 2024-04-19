#ifndef IMU_H
#define IMU_H
#include "Wire.h"
#include "MPU6050_light.h"
#include <bigbang_eurobot/RawImu.h>
#include "global.h"
struct Acceleration
{
    float x;
    float y;
    float z;
};

struct IMUData
{
    Acceleration linear;
    Acceleration rotation;
    float angleX;
    float angleY;
    float angleZ;
    void toMsg(bigbang_eurobot::RawImu *msg)
    {
        msg->linX = linear.x;
        msg->linY = linear.y;
        msg->linZ = linear.z;
        msg->rotX = rotation.x;
        msg->rotY = rotation.y;
        msg->angX = angleX;
        msg->angY = angleY;
        msg->angZ = angleZ;
    }
};

class IMU
{
public:
    IMU(TwoWire &wire, bool calcAngles = true) :
        updateAngles(calcAngles),
        mpu(wire) {};
    void setup() {
        Wire.begin();
        if (mpu.begin()) {
            mpu.calcOffsets();
        } else {
            nh.logerror("Imu setup error!");
        }
    }
    void update() {
        if (updateAngles) {
            mpu.update();
        } else {
            mpu.fetchData();
        }
    }
    void getData(IMUData *out) {
        out->rotation.x = mpu.getAccAngleX();
        out->rotation.y = mpu.getAccAngleY();
        out->linear.x = mpu.getAccX();
        out->linear.y = mpu.getAccY();
        out->linear.z = mpu.getAccZ();
        if (updateAngles) {
            out->angleX = mpu.getAngleX();
            out->angleY = mpu.getAngleY();
            out->angleZ = mpu.getAngleZ();
        }
    }
    void setGyroOffsets(float x, float y, float z) {
        mpu.setGyroOffsets(x, y, z);
    }
    void setAccOffsets(float x, float y, float z) {
        mpu.setAccOffsets(x, y, z);
    }
    void setFilterGyroCoef(float coeff)
    {
        mpu.setFilterGyroCoef(coeff);
    }
private:
    MPU6050 mpu;
    bool updateAngles;
};
#endif
