/**
 * ******************************************************************************
 *  @file    : BQ27427.h
 *  @brief   : Header for BQ27427.h file.
 *  @author  : Wendell
 * ******************************************************************************
 */

#ifndef INC_BQ27427_H_
#define INC_BQ27427_H_

/*
 * ******************************************************************************
 *  Includes   <System Includes> , "Project Includes"
 * ******************************************************************************
 */

#include "main.h"

/*
 * ******************************************************************************
 *  Exported constants
 * ******************************************************************************
 */

/* Endereço I2C do sensor BQ27427. */
#define BQ27427_I2C_ADDR (0x55 << 1)
#define BQ27427_MAX_DELAY 600

/* comandos de controle */
#define BQ27427_CMD_CTRL_SUBCMD     0x00
#define BQ27427_SUBCMD_UNSEAL       0x8000
#define BQ27427_SUBCMD_CHEM_ID      0x0008
#define BQ27427_SUBCMD_BAT_INSERT      0x000C
#define BQ27427_SUBCMD_CFGUPDATE    0x0013
#define BQ27427_SUBCMD_SEALED       0x0020
#define BQ27427_SUBCMD_SOFT_RESET   0x0042

/* blocos de dados */
#define BQ27427_REG_BLOCKDATA_CTRL  0x61
#define BQ27427_REG_DATACLASS       0x3E
#define BQ27427_REG_DATAOFFSET      0x3F
#define BQ27427_BLOCK_CLASS_STATE   0x52

/* offsets dentro do bloco “State” */
#define BQ27427_OFFSET_DESIGN_CAP_LSB  0x46
#define BQ27427_OFFSET_DESIGN_CAP_MSB  0x47
#define BQ27427_OFFSET_TAPER_RATE_LSB  0x55
#define BQ27427_OFFSET_TAPER_RATE_MSB  0x56
#define BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_LSB  0x40
#define BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_MSB  0x41
#define BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_LSB  0x42
#define BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_MSB  0x43
#define BQ27427_OFFSET_QUIT_CURRENT_LSB  0x44
#define BQ27427_OFFSET_QUIT_CURRENT_MSB  0x45
#define BQ27427_OFFSET_mWH_LSB  0x48
#define BQ27427_OFFSET_mWH_MSB  0x49
#define BQ27427_OFFSET_TERMINATE_VOLTAGE_LSB  0x4A
#define BQ27427_OFFSET_TERMINATE_VOLTAGE_MSB  0x4B
#define BQ27427_OFFSET_CHECKSUM        0x60

/* Chemistry Profile */
#define CHEM_A 0
#define CHEM_B 1
#define CHEM_C 2
#define BQ27427_CHEMISTRY_PROFILE_A        0x30
#define BQ27427_CHEMISTRY_PROFILE_B        0x31
#define BQ27427_CHEMISTRY_PROFILE_C        0x32

#define BATERRY_CAPACITY 100
#define BATERRY_TERMINATE_VOLTAGE 3500

/* flags */
#define BQ27427_FLAG_CFGUPMODE       (1U << 4)

/* Macro to send 16-bit control subcommands (LSB first) */
#define BQ27427_sendCmd(sensor_BQ27427, subcmd)                               \
    do {                                                   \
        uint8_t _buf[2] = { (uint8_t)((subcmd) & 0xFF), (uint8_t)(((subcmd) >> 8) & 0xFF) }; \
        BQ27427_Write(sensor_BQ27427, BQ27427_CMD_CTRL_SUBCMD, _buf, 2);  \
    } while (0)


/*brief Estrutura representando o sensor BQ27427.*/
typedef struct {
    I2C_HandleTypeDef *hi2c; /* > Ponteiro para a interface I2C utilizada para comunicação. */
} BQ27427_t;

extern BQ27427_t bq_sensor;

/*
 * ******************************************************************************
 *  Exported global functions (to be accessed by other files)
 * ******************************************************************************
 */

void BQ27427_Init(BQ27427_t *sensor_BQ27427, I2C_HandleTypeDef *hi2c);

uint16_t BQ27427_Control(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadTemperature(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadVoltage(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadFlags(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadNominalAvailableCapacity(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadFullAvailableCapacity(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadRemainingCapacity(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadFullChargeCapacity(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadAverageCurrent(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadAveragePower(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadStateOfCharge(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadInternalTemperature(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadRemainingCapacityUnfiltered(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadRemainingCapacityFiltered(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadFullChargeCapacityUnfiltered(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadFullChargeCapacityFiltered(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadStateOfChargeUnfiltered(BQ27427_t *sensor_BQ27427);

uint16_t BQ27427_ReadDesignCapacity(BQ27427_t *sensor_BQ27427);

HAL_StatusTypeDef BQ27427_SetChemistryProfile(BQ27427_t *sensor_BQ27427, uint8_t profile);

HAL_StatusTypeDef BQ27427_SetDesignCapacity(BQ27427_t *sensor_BQ27427, uint16_t capacity_mAh, uint16_t mV);

uint16_t BQ27427_GetDesignCapacity(BQ27427_t *sensor_BQ27427);

#endif /* INC_BQ27427_H_ */
