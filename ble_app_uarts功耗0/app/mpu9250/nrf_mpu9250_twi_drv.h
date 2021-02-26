#ifndef NRF_MPU9250_TWI_DRV_H
#define NRF_MPU9250_TWI_DRV_H

#include <stdbool.h>
#include <stdint.h>
//TWI³¬Ê±Öµ
#define MPU9250_TWI_TIMEOUT 			  10000 

//MPU9250  ID£¬¼´WHO_AM_I¼Ä´æÆ÷µÄÖµ = 0x71
#define MPU9250_WHO_AM_I_VALUE      0x71 

//SPIÆ¬Ñ¡Òý½Å£¬Ê¹ÓÃI2CÊ±±ØÐëÎª¸ßµçÆ½
#define MPU9250_CS_PIN             10 //24
//I2C´Ó»úµØÖ·µÄAD0Î»¿ØÖÆ¹Ü½Ì
#define MPU9250_AD0_PIN            11 //25

//I2C´Ó»úµØÖ·£¬Èç¹û AD0= 1£¬´Ó»úµØÖ· = 0x69£¬Èç¹ûAD0= 0£¬´Ó»úµØÖ· = 0x68
#define MPU9250_ADDRESS             0x68
#define AK8963_MAGN_ADDRESS         0x0C


typedef struct
{
	uint8_t 			: 3;  
	uint8_t overflow 	: 1; //  single measurement mode, continuous measurement mode, external trigger measurement mode and self-test mode, magnetic sensor may overflow even though measurement data regiseter is not saturated. 
	uint8_t res_mirror 	: 1; // Output bit setting (mirror) 
}mpu_magn_read_status_t;

/**@brief Enum defining Accelerometer's Full Scale range posibillities in Gs. */
enum accel_range {
  AFS_2G = 0,       // 2 G
  AFS_4G,           // 4 G
  AFS_8G,           // 8 G
  AFS_16G           // 16 G
};

/**@brief Enum defining Gyroscopes? Full Scale range posibillities in Degrees Pr Second. */
enum gyro_range {
  GFS_250DPS = 0,   // 250 deg/s
  GFS_500DPS,       // 500 deg/s
  GFS_1000DPS,      // 1000 deg/s
  GFS_2000DPS       // 2000 deg/s
};

typedef struct
{
	uint8_t mode : 4;  
	uint8_t resolution : 1;

}mpu_magn_config_t;


/**@brief MPU driver digital low pass fileter and external Frame Synchronization (FSYNC) pin sampling configuration structure */
typedef struct
{
    uint8_t dlpf_cfg     :3; // 3-bit unsigned value. Configures the Digital Low Pass Filter setting.
    uint8_t ext_sync_set :3; // 3-bit unsigned value. Configures the external Frame Synchronization (FSYNC) pin sampling.

    uint8_t fifo_mode    :1; // When set to ‘1’, when the fifo is full, additional writes will not be written to fifo. When set to ‘0’, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
    uint8_t              :1;

}sync_dlpf_config_t;

/**@brief MPU driver gyro configuration structure. */
typedef struct
{
    uint8_t f_choice        :2;
    uint8_t                 :1;
    uint8_t fs_sel          :2; // FS_SEL 2-bit unsigned value. Selects the full scale range of gyroscopes.   
    uint8_t gz_st           :1;
    uint8_t gy_st           :1;
    uint8_t gx_st           :1;

}gyro_config_t;

/**@brief MPU driver accelerometer configuration structure. */
typedef struct
{
    uint8_t                 :3;
    uint8_t afs_sel         :2; // 2-bit unsigned value. Selects the full scale range of accelerometers.
    uint8_t za_st           :1; // When set to 1, the Z- Axis accelerometer performs self test.
    uint8_t ya_st           :1; // When set to 1, the Y- Axis accelerometer performs self test.
    uint8_t xa_st           :1; // When set to 1, the X- Axis accelerometer performs self test.
}accel_config_t;

/**@brief MPU driver general configuration structure. */
typedef struct
{
    uint8_t             smplrt_div;         // Divider from the gyroscope output rate used to generate the Sample Rate for the MPU-9150. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    sync_dlpf_config_t  sync_dlpf_gonfig;   // Digital low pass fileter and external Frame Synchronization (FSYNC) configuration structure
    gyro_config_t       gyro_config;        // Gyro configuration structure
    accel_config_t      accel_config;       // Accelerometer configuration structure
}mpu_config_t;


#define MPU_DEFAULT_CONFIG()                          \
    {                                                     \
        .smplrt_div                     = 7,              \
        .sync_dlpf_gonfig.dlpf_cfg      = 1,              \
        .sync_dlpf_gonfig.ext_sync_set  = 0,              \
        .gyro_config.fs_sel             = GFS_2000DPS,    \
        .gyro_config.f_choice           = 0,              \
        .gyro_config.gz_st              = 0,              \
        .gyro_config.gy_st              = 0,              \
        .gyro_config.gx_st              = 0,              \
        .accel_config.afs_sel           = AFS_2G,        \
        .accel_config.za_st             = 0,              \
        .accel_config.ya_st             = 0,              \
        .accel_config.xa_st             = 0,              \
    }


enum magn_op_mode {
	POWER_DOWN_MODE = 0,   				// Power to almost all internal circuits is turned off.
	SINGLE_MEASUREMENT_MODE,       		// Sensor is measured, and after sensor measurement and signal processing is finished, measurement data is stored to measurement data registers (HXL to HZH), then AK8963 transits to power-down mode automatically.
	CONTINUOUS_MEASUREMENT_8Hz_MODE,      //  Sensor is measured periodically at 8Hz
	EXTERNAL_TRIGGER_MODE = 4,       	// When external trigger measurement mode is set, AK89xx waits for trigger input. When a pulse is input from TRG pin, sensor measurement is started on the rising edge of TRG pin.
	CONTINUOUS_MEASUREMENT_100Hz_MODE = 6,  //  Sensor is measured periodically at 100Hz
	SELF_TEST_MODE = 8,					// Self-test mode is used to check if the sensor is working normally
	FUSE_ROM_ACCESS_MODE = 0xFF			// Fuse ROM access mode is used to read Fuse ROM data. Sensitivity adjustment data for each axis is stored in fuse ROM.
};
/**@brief MPU driver interrupt pin configuration structure. */    
typedef struct
{
    uint8_t clkout_en       :1;  // When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For further information regarding CLKOUT, please refer to the MPU-9150 Product Specification document.
    uint8_t i2c_bypass_en   :1;  // When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-9150. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-9150 regardless of the state of I2C_MST_EN (Register 106 bit[5]).
    uint8_t fsync_int_en    :1;  // When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor. When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.
    uint8_t fsync_int_level :1;  // When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high. When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low.
    uint8_t int_rd_clear    :1;  // When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58). When this bit is equal to 1, interrupt status bits are cleared on any read operation.
    uint8_t latch_int_en    :1;  // When this bit is equal to 0, the INT pin emits a 50us long pulse. When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
    uint8_t int_open        :1;  // When this bit is equal to 0, the INT pin is configured as push-pull. When this bit is equal to 1, the INT pin is configured as open drain.
    uint8_t int_level       :1; // When this bit is equal to 0, the logic level for the INT pin is active high. When this bit is equal to 1, the logic level for the INT pin is active low.
}mpu_int_pin_cfg_t;

typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}accel_values_t;

typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}gyro_values_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}mag_values_t;

typedef int16_t temp_value_t;




/**@brief Function for reading an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @param[in]   length          Number of bytes to write
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length);



/**@brief Function for reading an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu_write_single_register(uint8_t reg, uint8_t data);



/**@brief Function for reading arbitrary register(s)
 *
 * @param[in]   reg             Register to read
 * @param[in]   p_data          Pointer to place to store value(s)
 * @param[in]   length          Number of registers to read
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);
uint32_t nrf_drv_mpu_read_magnetometer_registers(uint8_t reg, uint8_t * p_data, uint32_t length);
uint32_t nrf_drv_mpu_write_magnetometer_register(uint8_t reg, uint8_t data);
uint32_t mpu9250_read_accel(accel_values_t * accel_values);
uint32_t mpu9250_read_gyro(gyro_values_t * gyro_values);
uint32_t mpu9250_read_temp(temp_value_t * temperature);
uint32_t mpu9250_read_mag(mag_values_t * gyro_values);


uint32_t mpu9250_init(void);
void mpu9250_ad0_config(bool set);
void twi_master_init(void);


uint32_t nrf_drv_mpu_write_registersd(uint8_t addr,uint8_t reg, uint32_t length, uint8_t * p_data);
uint32_t nrf_drv_mpu_read_registersd(uint8_t addr,uint8_t reg, uint32_t length, uint8_t * p_data);
#endif




