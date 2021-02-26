 /* MPU9250 Register Map
 */
#define MPU9250_REG_AK8963_ST1       0x02  // data ready status bit 0
#define MPU9250_REG_AK8963_XOUT_L    0x03  // data
#define MPU9250_REG_AK8963_XOUT_H    0x04
#define MPU9250_REG_AK8963_YOUT_L    0x05
#define MPU9250_REG_AK8963_YOUT_H    0x06
#define MPU9250_REG_AK8963_ZOUT_L    0x07
#define MPU9250_REG_AK8963_ZOUT_H    0x08
#define MPU9250_REG_AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define MPU9250_REG_AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define MPU9250_REG_AK8963_ASTC      0x0C  // Self test control
#define MPU9250_REG_AK8963_I2CDIS    0x0F  // I2C disable
#define MPU9250_REG_AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define MPU9250_REG_AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define MPU9250_REG_AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_REG_SMPLRT_DIV       0x19


#define MPU9250_REG_INT_PIN_CFG      0x37  //55
#define MPU9250_REG_INT_ENABLE       0x38  //56

#define MPU9250_REG_TEMP_OUT_H       0x41  //65
#define MPU9250_REG_TEMP_OUT_L       0x42  //66
//陀螺仪输出数据
#define MPU9250_REG_GYRO_XOUT_H      0x43  //67
#define MPU9250_REG_GYRO_XOUT_L      0x44  //68
#define MPU9250_REG_GYRO_YOUT_H      0x45  //69
#define MPU9250_REG_GYRO_YOUT_L      0x46  //70
#define MPU9250_REG_GYRO_ZOUT_H      0x47  //71
#define MPU9250_REG_GYRO_ZOUT_L      0x48  //72
//加速度计输出数据
#define MPU9250_REG_ACCEL_XOUT_H     0x3B  //59
#define MPU9250_REG_ACCEL_XOUT_L     0x3C  //60
#define MPU9250_REG_ACCEL_YOUT_H     0x3D  //61
#define MPU9250_REG_ACCEL_YOUT_L     0x3E  //62
#define MPU9250_REG_ACCEL_ZOUT_H     0x3F  //63
#define MPU9250_REG_ACCEL_ZOUT_L     0x40  //64

#define MPU9250_REG_SIGNAL_PATH_RESET  0x68  //104
#define MPU9250_REG_MOT_DETECT_CTRL    0x69  //105
#define MPU9250_REG_USER_CTRL          0x6A  //106
#define MPU9250_REG_PWR_MGMT_1         0x6B  //107
#define MPU9250_REG_PWR_MGMT_2         0x6C

#define MPU9250_REG_WHO_AM_I           0x75  //117

 

