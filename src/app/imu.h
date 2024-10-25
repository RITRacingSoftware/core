#include <stdint.h>
#include <stdbool.h>

#ifndef IMU_H
#define IMU_H

#define MEMBER_OFFSET(member) ((uint32_t)(&(((imu_result_t*)0)->member)))

typedef struct imu_result_s {
    // IMU data
    float UncompMagX, UncompMagY, UncompMagZ,
          UncompAccelX, UncompAccelY, UncompAccelZ,
          UncompGyroX, UncompGyroY, UncompGyroZ,
          Temperature,
          Pressure,
          DeltaThetaT, DeltaThetaX, DeltaThetaY, DeltaThetaZ,
          DeltaVelX, DeltaVelY, DeltaVelZ,
          MagX, MagY, MagZ,
          AccelX, AccelY, AccelZ,
          AngularRateX, AngularRateY, AngularRateZ;
    uint16_t SensSat;
    // INS data
    uint16_t InsStatus;
    double PosLlaL, PosLlaO, PosLlaA,
           PosEcefX, PosEcefY, PosEcefZ;
    float VelBodyX, VelBodyY, VelBodyZ,
          VelNedN, VelNedE, VelNedD,
          VelEcefX, VelEcefY, VelEcefZ,
          MagEcefX, MagEcefY, MagEcefZ,
          AccelEcefX, AccelEcefY, AccelEcefZ,
          LinAccelEcefX, LinAccelEcefY, LinAccelEcefZ,
          PosU,
          VelU;
} imu_result_t;

bool imu_parse(uint8_t *buf, imu_result_t *data);

#endif
