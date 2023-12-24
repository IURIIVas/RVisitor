/// \file power_measure.c
/// \brief current and voltage measure with ina226 source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "power_measure.h"
#include "queue.h"
#include "ina226.h"
#include "i2c.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t power_measure_task_handler;
ina226_measures_s power_measures_struct = {.current = 0, .voltage = 0};
QueueHandle_t queue_power_measure;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _i2c_gpio_init(void)
{
    gpio_init_s gpio_init_struct = {0};
    RCC_APB2PeriphClockCmd(I2C_RCC_GPIO_PORT, ENABLE);

    gpio_init_struct.GPIO_Pins = I2C_GPIO_SDA_PIN | I2C_GPIO_SCL_PIN;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO_PORT, &gpio_init_struct);
}

static void _i2c_init(u32 bound, u16 address)
{
    i2c_init_s  i2c_init_struct = {0};

    RCC_APB1PeriphClockCmd(I2C_RCC, ENABLE);

    i2c_init_struct.I2C_ClockSpeed = bound;
    i2c_init_struct.I2C_Mode = I2C_Mode_I2C;
    i2c_init_struct.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c_init_struct.I2C_OwnAddress1 = address;
    i2c_init_struct.I2C_Ack = I2C_Ack_Enable;
    i2c_init_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C_POWER_MES, &i2c_init_struct);

    I2C_Cmd(I2C_POWER_MES, ENABLE);
    I2C_AcknowledgeConfig(I2C_POWER_MES, ENABLE);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void power_measure_task(void *pvParameters)
{
    queue_power_measure = xQueueCreate(2, sizeof(double));

    ina226_set_config(I2C_POWER_MES, INA226_ADDR, INA226_RESET_ACTIVE);
    ina226_set_calibration_reg(I2C_POWER_MES, INA226_ADDR, INA226_CAL_VAL);
    ina226_set_config(I2C_POWER_MES, INA226_ADDR, INA226_AVG_64 | INA226_VBUS_1100uS |
                                                  INA226_VSH_1100uS | INA226_MODE_CONT_SHUNT_AND_BUS);

    while (1)
    {
        power_measures_struct.current = ina226_get_current(I2C_POWER_MES, INA226_ADDR);
        power_measures_struct.voltage = ina226_get_busv(I2C_POWER_MES, INA226_ADDR);

        xQueueSend(queue_power_measure, (void*) &power_measures_struct.current, portMAX_DELAY);
        xQueueSend(queue_power_measure, (void*) &power_measures_struct.voltage, portMAX_DELAY);

        vTaskDelay(POWER_MEAS_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

/// \brief
/// \param
/// \retval
/// \return
void power_measure_task_init(void)
{
    _i2c_gpio_init();
    _i2c_init(INA226_BAUDRATE, INA226_ADDR);

    xTaskCreate((TaskFunction_t )power_measure_task,
                (const char*    )"power_measure",
                (uint16_t       )POWER_MEASURE_TASK_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )POWER_MEASURE_TASK_PRIORITY,
                (TaskHandle_t*  )&power_measure_task_handler);
}
