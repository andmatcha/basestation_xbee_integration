#include "modules/lcd_driver.h"

#include "debug_log.h"

#include <stddef.h>
#include <string.h>

#define LCD_I2C_ADDRESS_7BIT  0x3EU
#define LCD_CONTROL_COMMAND   0x00U
#define LCD_CONTROL_DATA      0x40U
#define LCD_TIMEOUT_MS        100U

#define LCD_CMD_CLEAR         0x01U
#define LCD_CMD_ENTRY_MODE    0x06U
#define LCD_CMD_DISPLAY_ON    0x0CU
#define LCD_CMD_FUNCTION      0x38U
#define LCD_CMD_FUNCTION_EXT  0x39U
#define LCD_CMD_SET_DDRAM     0x80U
#define LCD_CONTRAST          0x24U
#define LCD_FOLLOWER_GAIN     0x04U
#define LCD_POWER_ICON_ON     0x08U
#define LCD_POWER_BOOSTER_ON  0x04U
#define LCD_FOLLOWER_ON       0x08U
#define LCD_COLS              16U

static I2C_HandleTypeDef *g_lcd_i2c;
static uint8_t g_lcd_i2c_address = (LCD_I2C_ADDRESS_7BIT << 1);
static bool g_lcd_ready;

static HAL_StatusTypeDef lcd_write(uint8_t control, uint8_t value)
{
    uint8_t buffer[2] = {control, value};
    HAL_StatusTypeDef status;

    if (g_lcd_i2c == NULL) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Master_Transmit(g_lcd_i2c, g_lcd_i2c_address,
                                     buffer, sizeof(buffer), LCD_TIMEOUT_MS);
    if (status != HAL_OK) {
        LOG("[integrated] lcd write failed: control=0x%02X value=0x%02X status=%d err=0x%08lX\r\n",
            control, value, status, (unsigned long)HAL_I2C_GetError(g_lcd_i2c));
    }

    return status;
}

static HAL_StatusTypeDef lcd_command(uint8_t command)
{
    return lcd_write(LCD_CONTROL_COMMAND, command);
}

static HAL_StatusTypeDef lcd_data(uint8_t data)
{
    return lcd_write(LCD_CONTROL_DATA, data);
}

static HAL_StatusTypeDef lcd_command_with_delay(uint8_t command, uint32_t delay_ms)
{
    HAL_StatusTypeDef status = lcd_command(command);

    if (status == HAL_OK) {
        HAL_Delay(delay_ms);
    }

    return status;
}

static HAL_StatusTypeDef lcd_clear(void)
{
    return lcd_command_with_delay(LCD_CMD_CLEAR, 2U);
}

static HAL_StatusTypeDef lcd_set_cursor(uint8_t row, uint8_t col)
{
    static const uint8_t row_offsets[] = {0x00U, 0x40U};

    if ((row >= 2U) || (col >= LCD_COLS)) {
        return HAL_ERROR;
    }

    return lcd_command((uint8_t)(LCD_CMD_SET_DDRAM | (row_offsets[row] + col)));
}

static HAL_StatusTypeDef lcd_write_padded(const char *text)
{
    HAL_StatusTypeDef status;
    uint8_t written = 0U;

    if (text == NULL) {
        text = "";
    }

    while ((written < LCD_COLS) && (*text != '\0')) {
        status = lcd_data((uint8_t)*text);
        if (status != HAL_OK) {
            return status;
        }
        text++;
        written++;
    }

    while (written < LCD_COLS) {
        status = lcd_data((uint8_t)' ');
        if (status != HAL_OK) {
            return status;
        }
        written++;
    }

    return HAL_OK;
}

HAL_StatusTypeDef lcd_driver_init(I2C_HandleTypeDef *hi2c,
                                  GPIO_TypeDef *reset_port,
                                  uint16_t reset_pin)
{
    HAL_StatusTypeDef status;

    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    g_lcd_i2c = hi2c;
    g_lcd_i2c_address = (LCD_I2C_ADDRESS_7BIT << 1);
    g_lcd_ready = false;

    if (reset_port != NULL) {
        HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_RESET);
        HAL_Delay(2U);
        HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
    }

    HAL_Delay(50U);

    status = lcd_command_with_delay(LCD_CMD_FUNCTION, 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay(LCD_CMD_FUNCTION_EXT, 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay(0x14U, 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay((uint8_t)(0x70U | (LCD_CONTRAST & 0x0FU)), 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay((uint8_t)(0x50U |
                                             LCD_POWER_ICON_ON |
                                             LCD_POWER_BOOSTER_ON |
                                             ((LCD_CONTRAST >> 4) & 0x03U)), 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay((uint8_t)(0x60U |
                                             LCD_FOLLOWER_ON |
                                             (LCD_FOLLOWER_GAIN & 0x07U)), 200U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay(LCD_CMD_DISPLAY_ON, 1U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_clear();
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_command_with_delay(LCD_CMD_ENTRY_MODE, 1U);
    if (status == HAL_OK) {
        g_lcd_ready = true;
    }

    return status;
}

HAL_StatusTypeDef lcd_driver_write_lines(const char *line0, const char *line1)
{
    HAL_StatusTypeDef status;

    if (!g_lcd_ready) {
        return HAL_ERROR;
    }

    status = lcd_set_cursor(0U, 0U);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_write_padded(line0);
    if (status != HAL_OK) {
        return status;
    }

    status = lcd_set_cursor(1U, 0U);
    if (status != HAL_OK) {
        return status;
    }

    return lcd_write_padded(line1);
}

bool lcd_driver_is_ready(void)
{
    return g_lcd_ready;
}
