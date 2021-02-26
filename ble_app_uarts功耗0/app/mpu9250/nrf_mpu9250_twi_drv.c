
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"

#include "mpu9250_register_map.h"
#include "nrf_mpu9250_twi_drv.h"

#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

//TWI引脚定义
#define TWI_SCL_M           NRF_GPIO_PIN_MAP(0,12)       //!< Master SCL pin.
#define TWI_SDA_M           NRF_GPIO_PIN_MAP(0,4)       //!< Master SDA pin.


#define MPU_TWI_BUFFER_SIZE     	  14



static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

static uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

static void nrf_drv_mpu_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}


void twi_master_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_mpu9250_config = {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_mpu9250_config, nrf_drv_mpu_twi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}

uint32_t nrf_drv_mpu_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    ret_code_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg, p_data, length);
	  err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDRESS, twi_tx_buffer, length + 1,false);
    APP_ERROR_CHECK(err_code);
	  while((!twi_tx_done) && --timeout){};
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;
		
		return err_code;
}
uint32_t nrf_drv_mpu_write_single_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};


    err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDRESS, packet, 2, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    return err_code;
}


uint32_t nrf_drv_mpu_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, MPU9250_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU9250_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}
uint32_t nrf_drv_mpu_read_magnetometer_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, AK8963_MAGN_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, AK8963_MAGN_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU9250_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}
uint32_t nrf_drv_mpu_write_magnetometer_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};

    err_code = nrf_drv_twi_tx(&m_twi, AK8963_MAGN_ADDRESS, packet, 2, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    twi_tx_done = false;

    return err_code;
}
uint32_t mpu9250_read_accel(accel_values_t * accel_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];
    err_code = nrf_drv_mpu_read_registers(MPU9250_REG_ACCEL_XOUT_H, raw_values, 6);
    if(err_code != NRF_SUCCESS) return err_code;
			
		// Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}
uint32_t mpu9250_read_gyro(gyro_values_t * gyro_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];
    err_code = nrf_drv_mpu_read_registers(MPU9250_REG_GYRO_XOUT_H, raw_values, 6);
    if(err_code != NRF_SUCCESS) return err_code;

    uint8_t *data;
    data = (uint8_t*)gyro_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}

uint32_t mpu9250_read_mag(mag_values_t * mag_values)
{
	  uint32_t err_code;

    err_code = nrf_drv_mpu_read_magnetometer_registers(MPU9250_REG_AK8963_XOUT_L, (uint8_t *)mag_values, 6);
    if(err_code != NRF_SUCCESS) return err_code;
    
		nrf_drv_mpu_write_magnetometer_register(MPU9250_REG_AK8963_CNTL,0x11); //AK8963每次读完以后都需要重新设置为单次测量模式，否则数据不会更新
    return NRF_SUCCESS;
}

uint32_t mpu9250_read_temp(temp_value_t * temperature)
{
    uint32_t err_code;
    uint8_t raw_values[2];
    err_code = nrf_drv_mpu_read_registers(MPU9250_REG_TEMP_OUT_H, raw_values, 2);
    if(err_code != NRF_SUCCESS) return err_code;  
    
    *temperature = (temp_value_t)(raw_values[0] << 8) + raw_values[1];
    
    return NRF_SUCCESS;    
}

void mpu9250_ad0_config(bool set)
{
	  if(set == true)nrf_gpio_pin_set(MPU9250_AD0_PIN);
	  else nrf_gpio_pin_clear(MPU9250_AD0_PIN);
}
uint32_t mpu9250_config(mpu_config_t * config)
{
    uint8_t *data;
    data = (uint8_t*)config;
    return nrf_drv_mpu_write_registers(MPU9250_REG_SMPLRT_DIV, data, 4);
}
uint32_t mpu_int_cfg_pin(mpu_int_pin_cfg_t *cfg)
{
    uint8_t *data;
    data = (uint8_t*)cfg;
    return nrf_drv_mpu_write_single_register(MPU9250_REG_INT_PIN_CFG, *data);
    
}
uint32_t mpu_magnetometer_init(mpu_magn_config_t * p_magnetometer_conf)
{	
	  uint32_t err_code;
	
	  //先读出INT Pin / Bypass 寄存器的值
	  mpu_int_pin_cfg_t bypass_config;
	  err_code = nrf_drv_mpu_read_registers(MPU9250_REG_INT_PIN_CFG, (uint8_t *)&bypass_config, 1);
	
	  //修改bypass_en位为1
	  bypass_config.i2c_bypass_en = 1;
	  //修改后再写回INT Pin / Bypass 寄存器	
	  err_code = mpu_int_cfg_pin(&bypass_config);
	  if (err_code != NRF_SUCCESS) return err_code;
	
	  //写INT Pin / Bypass 	
	  uint8_t *data;
    data = (uint8_t*)p_magnetometer_conf;	
    return nrf_drv_mpu_write_magnetometer_register(MPU9250_REG_AK8963_CNTL, *data);
}

uint32_t mpu9250_init(void)
{
    uint32_t err_code;
	  uint8_t id =0,temp;
	
	  //设置mpu9250 i2c地址AD0位为0
	  nrf_gpio_cfg_output(MPU9250_AD0_PIN);
	  mpu9250_ad0_config(false);
	  //CS引脚设置为高电平，使用I2C接口
	  nrf_gpio_cfg_output(MPU9250_CS_PIN);
	  nrf_gpio_pin_set(MPU9250_CS_PIN);
	
	  //初始化TWI
	  twi_master_init();

	
    //读取MPU9250器件ID(WHO_AM_I寄存器值)，读出的值应是0x71
	  do
	  {
		    nrf_drv_mpu_read_registers(MPU9250_REG_WHO_AM_I,&id,1);
			  if(id != MPU9250_WHO_AM_I_VALUE)
				{
					  printf("ID:  %d\r\n",id);
			      printf("mpu9250 init err...\r\n");
			      nrf_delay_ms(500);
				}
		}while(id != MPU9250_WHO_AM_I_VALUE);

	  //复位gyro, accelerometer temperature sensor
	  uint8_t reset_value = 7; 
    err_code = nrf_drv_mpu_write_single_register(MPU9250_REG_SIGNAL_PATH_RESET, reset_value);
    if(err_code != NRF_SUCCESS) 
		{
			return err_code;
		}

    //使用内部20MHz晶振
    err_code = nrf_drv_mpu_write_single_register(MPU9250_REG_PWR_MGMT_1, 0x00);
    if(err_code != NRF_SUCCESS) 
		{
			return err_code;
		}
		
		//配置mpu9250
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); //加载默认值
    p_mpu_config.smplrt_div = 7;   //设置采样速率。采样速率 = Internal_Sample_Rate / (1 + SMPLRT_DIV)。 19对应的是50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; //设置加速度计满量程2G
    
	  err_code = mpu9250_config(&p_mpu_config); 
    APP_ERROR_CHECK(err_code); 
    
		//测试
		nrf_drv_mpu_read_registers(MPU9250_REG_WHO_AM_I,&temp,1);
		printf("WHO_AM_I:  %d\r\n",temp);
		nrf_delay_ms(100);
		
		nrf_drv_mpu_read_registers(MPU9250_REG_SIGNAL_PATH_RESET,&temp,1);
		printf("reg 104(0x68):  %d\r\n",temp);
		nrf_delay_ms(100);
	
	  // Enable magnetometer
	  mpu_magn_config_t magnetometer_config;
	  magnetometer_config.mode = SINGLE_MEASUREMENT_MODE;
	  magnetometer_config.resolution = 1;
    err_code = mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(err_code); 

    return NRF_SUCCESS;
}

#if 1
uint32_t nrf_drv_mpu_write_registersd(uint8_t addr,uint8_t reg, uint32_t length, uint8_t * p_data)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    ret_code_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg, p_data, length);
	
	  err_code = nrf_drv_twi_tx(&m_twi, addr, twi_tx_buffer, length + 1,false);
    APP_ERROR_CHECK(err_code);
	  while((!twi_tx_done) && --timeout){};
    if(!timeout) return NRF_ERROR_TIMEOUT;
		nrf_delay_ms(1);
    twi_tx_done = false;
		
		return err_code;
}

uint32_t nrf_drv_mpu_read_registersd(uint8_t addr,uint8_t reg, uint32_t length, uint8_t * p_data)
{
    uint32_t err_code;
    uint32_t timeout = MPU9250_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, addr, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
	nrf_delay_ms(1);
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, addr, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU9250_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    twi_rx_done = false;

    return err_code;
}

#endif








