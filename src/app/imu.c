#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "imu.h"

static const uint32_t imu_group_sizes[] = {0, 0, 12, 0, 0, 11, 0};

static const uint32_t imu_offsets[] = {
    // common
    // time
    // imu
    0, MEMBER_OFFSET(UncompMagX), MEMBER_OFFSET(UncompAccelX), MEMBER_OFFSET(UncompGyroX), MEMBER_OFFSET(Temperature), MEMBER_OFFSET(Pressure), MEMBER_OFFSET(DeltaThetaT), MEMBER_OFFSET(DeltaVelX), MEMBER_OFFSET(MagX), MEMBER_OFFSET(AccelX), MEMBER_OFFSET(AngularRateX), MEMBER_OFFSET(SensSat),
    // gnss
    // attitude
    // ins
    MEMBER_OFFSET(InsStatus), MEMBER_OFFSET(PosLlaL), MEMBER_OFFSET(PosEcefX), MEMBER_OFFSET(VelBodyX), MEMBER_OFFSET(VelNedN), MEMBER_OFFSET(VelEcefX), MEMBER_OFFSET(MagEcefX), MEMBER_OFFSET(AccelEcefX), MEMBER_OFFSET(LinAccelEcefX), MEMBER_OFFSET(PosU), MEMBER_OFFSET(VelU),
    // gnss2
};

static const uint32_t imu_lengths[] = {
    // common
    // time
    // imu
    0, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2,
    // gnss
    // attitude
    // ins
    2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4,
    // gnss2
};

bool imu_parse(uint8_t *buf, imu_result_t *data) {
    if (buf[0] != 0xFA) return false;
    uint16_t mask_ptr = 2;
    uint16_t ptr = 2;
    for (uint8_t i=0; i < 7; i++) {
        if (buf[1] & (1<<i)) ptr += 2;
    }
    uint16_t lut_idx = 0;
    uint16_t mask;
    for (uint8_t i=0; i < 7; i++) {
        if (buf[1] & (1<<i)) {
            mask = *((uint16_t*)(buf + mask_ptr));
            for (uint8_t j=0; j < imu_group_sizes[i]; j++) {
                if (mask & (1<<j)) {
                    memcpy(((uint8_t*)data) + imu_offsets[lut_idx + j], buf+ptr, imu_lengths[lut_idx + j]);
                    ptr += imu_lengths[lut_idx + j];
                }
            }
            mask_ptr += 2;
        }
        lut_idx += imu_group_sizes[i];
    }
    return true;
}


/*int main() {
    for (int i=0; i < 23; i++) printf("%d ", imu_offsets[i]);
    printf("\n");
    FILE *fptr;
    uint8_t buf[512];
    imu_result_t data;
    fptr = fopen("/tmp/imu.bin", "r");
    fread(buf, 512, 1, fptr);
    fclose(fptr);
    parse(buf, &data);
    printf("Acceleration: %f, %f, %f\n", data.AccelX, data.AccelY, data.AccelZ);
}*/
