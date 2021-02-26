#ifndef APP_MPU9250_H
#define APP_MPU9250_H

#include <stdbool.h>
#include <stdint.h>
#include "mpu9250_register_map.h"
#include "nrf_mpu9250_twi_drv.h"








void mpu9250_ad0_config(bool set);
uint32_t mpu9250_config(mpu_config_t * config);
uint32_t mpu_magnetometer_init(mpu_magn_config_t * p_magnetometer_conf);
uint32_t mpu_read_magnetometer(mag_values_t * p_magnetometer_values, mpu_magn_read_status_t * p_read_status);









#endif


