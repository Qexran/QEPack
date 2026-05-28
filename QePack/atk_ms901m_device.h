/**
  * @file       atk_ms901m_device.h
  * @author     正点原子团队(ALIENTEK) & Qe_xr
  * @version    V1.3.0
  * @date       2026/05/14
  * @brief      ATK-MS901M 模块驱动，基于 STM32 HAL 库 / TI DriverLib
  *             支持阻塞轮询模式和流式缓存回调模式
  */

#include "project_config.h"

#if ATK_MS901M_IS_ENABLE

#ifndef __ATK_MS901M_DEVICE_H
#define __ATK_MS901M_DEVICE_H

#include "uart_device.h"

/* ATK-MS901M 设备号枚举 */
typedef enum
{
    emAtkMs901mDevNum0 = 0,                         // ATK-MS901M0
    emAtkMs901mDevNum1,                             // ATK-MS901M1
    emAtkMs901mDevNum2,                             // ATK-MS901M2
    emAtkMs901mDevNum3,                             // ATK-MS901M3
} emAtkMs901mDevNumTdf;

/* ATK-MS901M UART通讯帧数据最大长度 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28

/* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    /* 姿态角 */
#define ATK_MS901M_FRAME_ID_QUAT            0x02    /* 四元数 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    /* 陀螺仪、加速度计 */
#define ATK_MS901M_FRAME_ID_MAG             0x04    /* 磁力计 */
#define ATK_MS901M_FRAME_ID_BARO            0x05    /* 气压计 */
#define ATK_MS901M_FRAME_ID_PORT            0x06    /* 端口 */

/* ATK-MS901M应答帧ID */
#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    /* （  W）保存当前配置到Flash */
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    /* （  W）设置传感器校准 */
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    /* （R  ）读取传感器校准状态 */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    /* （R/W）设置陀螺仪量程 */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    /* （R/W）设置加速度计量程 */
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    /* （R/W）设置陀螺仪带宽 */
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    /* （R/W）设置加速度计带宽 */
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    /* （R/W）设置UART通讯波特率 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    /* （R/W）设置回传内容 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    /* （R/W）设置回传内容2（保留） */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    /* （R/W）设置回传速率 */
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    /* （R/W）设置算法 */
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    /* （R/W）设置安装方向 */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    /* （R/W）设置陀螺仪自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    /* （R/W）设置气压计自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    /* （R/W）设置LED开关 */
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    /* （R/W）设置端口D0模式 */
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    /* （R/W）设置端口D1模式 */
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    /* （R/W）设置端口D2模式 */
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    /* （R/W）设置端口D3模式 */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    /* （R/W）设置端口D1 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    /* （R/W）设置端口D3 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    /* （R/W）设置端口D1 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    /* （R/W）设置端口D3 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    /* （  W）恢复默认设置 */

/* ATK-MS901M帧类型 */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       /* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       /* ATK-MS901M应答帧ID */

/* 姿态角数据结构体 */
typedef struct
{
    float roll;                                     /* 横滚角，单位：° */
    float pitch;                                    /* 俯仰角，单位：° */
    float yaw;                                      /* 航向角，单位：° */
} atk_ms901m_attitude_data_t;

/* 四元数数据结构体 */
typedef struct
{
    float q0;                                       /* Q0 */
    float q1;                                       /* Q1 */
    float q2;                                       /* Q2 */
    float q3;                                       /* Q3 */
} atk_ms901m_quaternion_data_t;

/* 陀螺仪数据结构体 */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X轴原始数据 */
        int16_t y;                                  /* Y轴原始数据 */
        int16_t z;                                  /* Z轴原始数据 */
    } raw;
    float x;                                        /* X轴旋转速率，单位：dps */
    float y;                                        /* Y轴旋转速率，单位：dps */
    float z;                                        /* Z轴旋转速率，单位：dps */
} atk_ms901m_gyro_data_t;

/* 加速度计数据结构体 */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X轴原始数据 */
        int16_t y;                                  /* Y轴原始数据 */
        int16_t z;                                  /* Z轴原始数据 */
    } raw;
    float x;                                        /* X轴加速度，单位：G */
    float y;                                        /* Y轴加速度，单位：G */
    float z;                                        /* Z轴加速度，单位：G */
} atk_ms901m_accelerometer_data_t;

/* 磁力计数据结构体 */
typedef struct
{
    int16_t x;                                      /* X轴磁场强度 */
    int16_t y;                                      /* Y轴磁场强度 */
    int16_t z;                                      /* Z轴磁场强度 */
    float temperature;                              /* 温度，单位：℃ */
} atk_ms901m_magnetometer_data_t;

/* 气压计数据结构体 */
typedef struct
{
    int32_t pressure;                               /* 气压，单位：Pa */
    int32_t altitude;                               /* 海拔，单位：cm */
    float temperature;                              /* 温度，单位：℃ */
} atk_ms901m_barometer_data_t;

/* 端口数据结构体 */
typedef struct
{
    uint16_t d0;                                    /* 端口D0数据 */
    uint16_t d1;                                    /* 端口D1数据 */
    uint16_t d2;                                    /* 端口D2数据 */
    uint16_t d3;                                    /* 端口D3数据 */
} atk_ms901m_port_data_t;

/* ATK-MS901M LED状态枚举 */
typedef enum
{
    ATK_MS901M_LED_STATE_ON  = 0x00,                /* LED灯打开 */
    ATK_MS901M_LED_STATE_OFF = 0x01,                /* LED灯关闭 */
} atk_ms901m_led_state_t;

/* ATK-MS901M端口枚举 */
typedef enum
{
    ATK_MS901M_PORT_D0 = 0x00,                      /* 端口D0 */
    ATK_MS901M_PORT_D1 = 0x01,                      /* 端口D1 */
    ATK_MS901M_PORT_D2 = 0x02,                      /* 端口D2 */
    ATK_MS901M_PORT_D3 = 0x03,                      /* 端口D3 */
} atk_ms901m_port_t;

/* ATK-MS901M端口模式枚举 */
typedef enum
{
    ATK_MS901M_PORT_MODE_ANALOG_INPUT   = 0x00,     /* 模拟输入 */
    ATK_MS901M_PORT_MODE_INPUT          = 0x01,     /* 数字输入 */
    ATK_MS901M_PORT_MODE_OUTPUT_HIGH    = 0x02,     /* 输出数字高电平 */
    ATK_MS901M_PORT_MODE_OUTPUT_LOW     = 0x03,     /* 输出数字低电平 */
    ATK_MS901M_PORT_MODE_OUTPUT_PWM     = 0x04,     /* 输出PWM */
} atk_ms901m_port_mode_t;

/* 错误代码 */
#define ATK_MS901M_EOK      0                       /* 没有错误 */
#define ATK_MS901M_ERROR    1                       /* 错误 */
#define ATK_MS901M_EINVAL   2                       /* 错误函数参数 */
#define ATK_MS901M_ETIMEOUT 3                       /* 超时错误 */

/* ATK-MS901M 设备参数（用户通过 c_pstGetAtkMs901mDeviceParam 只读访问） */
typedef struct
{
    atk_ms901m_attitude_data_t      stAttitude;
    atk_ms901m_quaternion_data_t    stQuaternion;
    atk_ms901m_gyro_data_t          stGyro;
    atk_ms901m_accelerometer_data_t stAccel;
    atk_ms901m_magnetometer_data_t  stMag;
    atk_ms901m_barometer_data_t     stBaro;
    atk_ms901m_port_data_t          stPort;
} stAtkMs901mDeviceParamTdf;

const stAtkMs901mDeviceParamTdf *c_pstGetAtkMs901mDeviceParam(emAtkMs901mDevNumTdf emDevNum);

/* ======================== 阻塞轮询 API（兼容模式） ======================== */
/* 调用后阻塞等待，直到收到指定帧或超时。适用于偶尔查询的场景。     */
/* 注意：阻塞期间会持续轮询 UART 环形缓冲区，不可在中断中调用。   */

uint8_t atk_ms901m_read_reg_by_id(emUartDevNumTdf emDevNum, uint8_t id, uint8_t *dat, uint32_t timeout);
uint8_t atk_ms901m_write_reg_by_id(emUartDevNumTdf emDevNum, uint8_t id, uint8_t len, uint8_t *dat);
uint8_t atk_ms901m_init(emUartDevNumTdf emDevNum);
uint8_t atk_ms901m_init_default(emUartDevNumTdf emDevNum);                           /* 低配版模块专用：跳过寄存器读取，使用默认FSR */
uint8_t atk_ms901m_get_attitude(emUartDevNumTdf emDevNum, atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout);
uint8_t atk_ms901m_get_quaternion(emUartDevNumTdf emDevNum, atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout);
uint8_t atk_ms901m_get_gyro_accelerometer(emUartDevNumTdf emDevNum, atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_magnetometer(emUartDevNumTdf emDevNum, atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_barometer(emUartDevNumTdf emDevNum, atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_port(emUartDevNumTdf emDevNum, atk_ms901m_port_data_t *port_dat, uint32_t timeout);
uint8_t atk_ms901m_get_led_state(emUartDevNumTdf emDevNum, atk_ms901m_led_state_t *state, uint32_t timeout);
uint8_t atk_ms901m_set_led_state(emUartDevNumTdf emDevNum, atk_ms901m_led_state_t state, uint32_t timeout);
uint8_t atk_ms901m_get_port_mode(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout);
uint8_t atk_ms901m_set_port_mode(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout);
uint8_t atk_ms901m_get_port_pwm_pulse(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout);
uint8_t atk_ms901m_set_port_pwm_pulse(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout);
uint8_t atk_ms901m_get_port_pwm_period(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t *period, uint32_t timeout);
uint8_t atk_ms901m_set_port_pwm_period(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t period, uint32_t timeout);

/* ====================== 流式缓存 API（推荐高频场景） ====================== */
/* 模块接管 UART 回调，在 vUartDevicePeriodExecute 中自动解析所有帧并缓存。 */
/* 用户调用 atk_ms901m_read_xxx() 零阻塞读取最新数据。                    */

/**
 * @brief  启动流式缓存模式
 * @param  emDevNum    : ATK-MS901M 设备号
 * @param  emUartDevNum: 绑定的 UART 设备号
 * @note   调用前需先完成 UART 的 vUartDeviceInit() 和 atk_ms901m_init()
 *         此后 UART 回调由本模块接管，用户不应再设置 vCallbackFcn
 *         解析在 vUartDevicePeriodExecute() 上下文中完成，建议每 1ms 调用
 * @retval ATK_MS901M_EOK  : 启动成功
 *         ATK_MS901M_ERROR: 启动失败（设备号越界）
 */
uint8_t atk_ms901m_start_streaming(emAtkMs901mDevNumTdf emDevNum, emUartDevNumTdf emUartDevNum);

/**
 * @brief  停止流式缓存模式
 * @param  emDevNum: ATK-MS901M 设备号
 * @note   恢复 UART 回调为空，之后可使用阻塞 API
 */
void atk_ms901m_stop_streaming(emAtkMs901mDevNumTdf emDevNum);

/**
 * @brief  读取姿态角（非阻塞，返回最新缓存值）
 * @param  emDevNum    : ATK-MS901M 设备号
 * @param  attitude_dat: 姿态角输出
 * @retval ATK_MS901M_EOK  : 读取成功
 *         ATK_MS901M_ERROR: 尚无数据（模块未启动流式或还未收到帧）
 */
uint8_t atk_ms901m_read_attitude(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_attitude_data_t *attitude_dat);

/**
 * @brief  读取四元数（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_quaternion(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_quaternion_data_t *quaternion_dat);

/**
 * @brief  读取陀螺仪数据（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_gyro(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_gyro_data_t *gyro_dat);

/**
 * @brief  读取加速度计数据（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_accelerometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_accelerometer_data_t *accelerometer_dat);

/**
 * @brief  读取磁力计数据（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_magnetometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_magnetometer_data_t *magnetometer_dat);

/**
 * @brief  读取气压计数据（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_barometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_barometer_data_t *barometer_dat);

/**
 * @brief  读取端口数据（非阻塞，返回最新缓存值）
 */
uint8_t atk_ms901m_read_port(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_port_data_t *port_dat);

/* ==================== 传感器基类适配 ==================== */
#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
    void vGyroSensorRegister(emSensorDevNumTdf emSensorDevNum);
#endif

#endif

#endif

/*
    usage:
    
    stUartStaticParamTdf stUartEmmInit = {
        .pstUartHandle = &TI_GET_UART_STRUCTURE(UART_ATK),
        .emFrameEn = emUartFrameOff,
        .ulBaudRate = 115200,
        .vCallbackFcn = vUartEmmCallback,             // 回调由 atk_ms901m_init_default 注册
    };

    vUartDeviceInit(&stUartEmmInit, UART_DEVICE_1);

    atk_ms901m_init_default(UART_DEVICE_1);
    
    // uint8_t ret = atk_ms901m_init(UART_DEVICE_1);
    // if (ret != ATK_MS901M_EOK) while(1);

    // 流式传输
    atk_ms901m_start_streaming(ATK_MS901M0, UART_DEVICE_1);

    {
        const stAtkMs901mDeviceParamTdf *pstDev = c_pstGetAtkMs901mDeviceParam(ATK_MS901M0);
        if (pstDev != NULL) {
            vOledPrintf(OLED0, 0, 0, OLED_8X16,  "Roll:%.2f ", pstDev->stAttitude.roll);
            vOledPrintf(OLED0, 0, 16, OLED_8X16, "Pitch:%.2f", pstDev->stAttitude.pitch);
            vOledPrintf(OLED0, 0, 32, OLED_8X16, "Yaw:%.2f", pstDev->stAttitude.yaw);

            vUartPrintf(UART_DEVICE_0,
            "Roll:%.2f,Pitch:%.2f,Yaw:%.2f\r\n",
            pstDev->stAttitude.roll, pstDev->stAttitude.pitch, pstDev->stAttitude.yaw
            );
        }
    }
    vOledUpdate(OLED0);

 */
