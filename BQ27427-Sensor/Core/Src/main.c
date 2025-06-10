/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bq27427.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

uint16_t I2C_ADDRESS = 0x55 << 1;

//uint8_t aTxBuffer[3] = {0x00,0x01,0x00};
uint8_t aRxBuffer[3];

BQ27427_t sensor; // Criação do objeto sensor

  /* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef gauge_write(uint8_t reg, uint8_t *data, uint8_t len){
    return HAL_I2C_Mem_Write(&hi2c1, BQ27427_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

static uint16_t gauge_read(uint8_t reg, uint8_t *data, uint8_t len){
    return HAL_I2C_Mem_Read(&hi2c1, BQ27427_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
static uint16_t gauge_read16(uint16_t reg){
	uint8_t aRxBuffer[2];

    if (HAL_I2C_Mem_Read(&hi2c1, BQ27427_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 2, HAL_MAX_DELAY) != HAL_OK){
    	return 0xFFFF; // Retorna um valor inválido em caso de falha
    }
	return  ((uint16_t)aRxBuffer[1] << 8) | aRxBuffer[0]; // Concatena os bytes em um valor de 16 bits
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  BQ27427_Init(&sensor, &hi2c1); // Inicializa o sensor
//  BQ27427_SetChemistryProfile(&sensor, CHEM_B);
//  BQ27427_SetDesignCapacity(&sensor, 100, 3500);

void __attribute__((unused)) bq27427_set_design_capacity(uint16_t mAh)
{
	/* -------- 1. Desbloquear (“unseal”) o gauge ---------------------------- */
    uint8_t unseal[2] = {0x00, 0x80};
    gauge_write(0x00, unseal, 2);           // primeira metade
    gauge_write(0x00, unseal, 2);           // segunda metade

	/* -------- 2. Entrar em CONFIG UPDATE mode ---------------------------- */
    uint8_t cfg[2] = {0x13, 0x00};			//CONFIG UPDATE
    gauge_write(0x00, cfg, 2);
    HAL_Delay(1100);

    // Depois, fique lendo Flags() (0x06/0x07) até que o bit [CFGUPMODE] seja 1 (pode levar ≤1 s)
	uint16_t flags;
    do{
    	flags = BQ27427_ReadFlags(&sensor);
    }while (!(flags & (1U << 4)));

    /* -------- 3. Preparar o bloco de dados da RAM para escrita ------------- */
    uint8_t zero = 0x00, cls = 0x52;
    gauge_write(0x61, &zero, 1);            // BlockDataControl
    gauge_write(0x3E, &cls, 1);             // DataBlockClass
    gauge_write(0x3F, &zero, 1);            // DataBlock offset 0-31

    /* -------- 4. Ler, escrever e recalcular checksum ------------- */
	uint8_t old_csum, old_msb, old_lsb;
    HAL_Delay(100);
	gauge_read(0x60, &old_csum, 1);   // deve ser o mesmo checksum estável
    HAL_Delay(100);
	gauge_read(0x46, &old_msb, 1);   // deve ser 0xB0
    HAL_Delay(100);
	gauge_read(0x47, &old_lsb, 1);   // deve ser 0x04

    uint8_t msb = (mAh >> 8) & 0xFF;
    uint8_t lsb =  mAh       & 0xFF;

    uint8_t sum_old_mod = (255u - old_csum) & 0xFF;
	int16_t sum_new_mod = sum_old_mod - old_msb - old_lsb + msb + lsb;

	// garanta wrap-around em 0–255:
	sum_new_mod = (sum_new_mod & 0xFF);

	uint8_t newCsum = 255u - (uint8_t)sum_new_mod;

    /* -------- 4.1. Escreva o novo valor de Design Capacity ---------------------- */
    gauge_write(0x46, &lsb, 1);       // 0x04
    gauge_write(0x47, &msb, 1);       // 0xB0
    gauge_write(0x60, &newCsum, 1);


    /* -------- 5. Sai do CONFIG UPDATE ------------------------------ */
    HAL_Delay(2000);
    uint8_t rst[2] = {0x42, 0x00};
    gauge_write(0x00, rst, 2);
    HAL_Delay(2000);

    /* Leitura da Flags referente a density capacity para confirmação visual se o valor foi realmente alterado  */
    //Passo 1
	gauge_write(0x00, unseal, 2);           // primeira metade
	gauge_write(0x00, unseal, 2);           // segunda metade
	//Passo 2
	gauge_write(0x00, cfg, 2);
	HAL_Delay(1100);
	//Passo 3
    zero = 0x00, cls = 0x52;
	gauge_write(0x61, &zero, 1);            // BlockDataControl
	gauge_write(0x3E, &cls, 1);             // DataBlockClass
	gauge_write(0x3F, &zero, 1);            // DataBlock offset 0-31

	uint16_t new_cap_lib = gauge_read16(0x4746);
	(void)new_cap_lib;

	HAL_Delay(2000);
	gauge_write(0x00, rst, 2);

    /* Leitura da Flags referente a density capacity para confirmação visual se o valor foi realmente alterado  */

    /* -------- 6. selar novamente (Opcional) ------------------------------ */
	HAL_Delay(2000);
    uint8_t seal[2] = {0x20, 0x00};
	gauge_write(0x00, seal, 2);


}

void __attribute__((unused)) bq27427_set_zerar_bit_OpConfig()
{
	/* -------- 1. UNSEAL --------------------------------------------------- */
	uint8_t unseal[2] = {0x00, 0x80};
	gauge_write(0x00, unseal, 2);
	gauge_write(0x00, unseal, 2);

	/* -------- 2. CONFIG UPDATE ------------------------------------------- */
	uint8_t cfg[2] = {0x13, 0x00};
	gauge_write(0x00, cfg, 2);
	HAL_Delay(5);
	uint16_t flags;
	do { flags = BQ27427_ReadFlags(&sensor); }
	while (!(flags & (1U<<4)));            /* espera CFGUPMODE = 1 */

	/* -------- 3. Seleciona bloco Configuration (ID 0x40) ----------------- */
	uint8_t zero = 0x00, cls = 0x40;
	gauge_write(0x61, &zero, 1);           /* BlockDataControl */
	gauge_write(0x3E, &cls,  1);           /* DataClass = 0x40 */
	gauge_write(0x3F, &zero, 1);           /* DataBlock = 0 */

	/* -------- 4. Lê OpConfig-LSB e checksum ------------------------------ */
	uint8_t oldOpCfg, oldCsum;
	gauge_read (0x40, &oldOpCfg, 1);       /* OpConfig-LSB */
	gauge_read (0x60, &oldCsum, 1);        /* checksum     */

	/* -------- 5. Zera BIE (bit-5) ---------------------------------------- */
	uint8_t newOpCfg = oldOpCfg & ~(1<<5);  /* limpa BIE     */
	uint8_t delta    = (uint8_t)((oldOpCfg - newOpCfg) & 0xFF);
	uint8_t newCsum  = (uint8_t)((oldCsum + delta)   & 0xFF);

	/* -------- 6. Grava novo byte + novo checksum ------------------------- */
	gauge_write(0x40, &newOpCfg, 1);       /* primeiro o dado */
	gauge_write(0x60, &newCsum,  1);       /* depois o csum   */

	/* -------- 7. SOFT_RESET para sair de CFGUP --------------------------- */
	uint8_t rst[2] = {0x42, 0x00};
	gauge_write(0x00, rst, 2);
	HAL_Delay(5);

	/* -------- 8. (Opcional) selar novamente ------------------------------ */
	uint8_t seal[2] = {0x20, 0x00};
	gauge_write(0x00, seal, 2);

}
void __attribute__((unused)) bq27427_mudar_perfil_quimico(){
	uint8_t unseal[2] = {0x00, 0x80};
	gauge_write(0x00, unseal, 2);
	gauge_write(0x00, unseal, 2);

	uint8_t subcmd[3] = { 0x00, 0x08, 0x00 };   // reg, LSB, MSB
	gauge_write(0x00, &subcmd[1], 2);
	uint8_t chem_id_raw[2];
	gauge_read(0x00, chem_id_raw, 2);
	uint16_t chem_id = chem_id_raw[0] | (chem_id_raw[1] << 8);   // 1202, 3142 ou 3230
	(void) chem_id;
	uint8_t cfg[2] = {0x13, 0x00};
	gauge_write(0x00, cfg, 2);
	HAL_Delay(1100);
//
	uint16_t flags;
	do{
		flags = BQ27427_ReadFlags(&sensor);
	}while (!(flags & (1U << 4)));
//
	uint8_t chem_b[2] = {0x31, 0x00};
	gauge_write(0x00, chem_b, 2);

	uint8_t rst[2] = {0x42, 0x00};
	gauge_write(0x00, rst, 2);
	HAL_Delay(5);

	do { flags = BQ27427_ReadFlags(&sensor);
	} while ((flags & (1<<4)) != 0);

	gauge_write(0x00, &subcmd[1], 2);
	gauge_read(0x00, chem_id_raw, 2);
	chem_id = chem_id_raw[0] | (chem_id_raw[1] << 8);   // 1202, 3142 ou 3230
	(void) chem_id;

//	uint8_t seal[2] = {0x20, 0x00};
//	gauge_write(0x00, seal, 2);
}

void __attribute__((unused)) bq27427_set_valores_padrao(uint16_t mAh, uint16_t mV)
{
	bq27427_mudar_perfil_quimico();

	uint8_t unseal[2] = {0x00, 0x80};
	gauge_write(0x00, unseal, 2);
	gauge_write(0x00, unseal, 2);

	/* -------- 2. CONFIG UPDATE ------------------------------------------- */
	uint8_t cfg[2] = {0x13, 0x00};
	gauge_write(0x00, cfg, 2);
	HAL_Delay(5);


    /*--- Esperar Flags()[4] = 1 ---*/
	uint16_t flags;
	do { flags = BQ27427_ReadFlags(&sensor); }
	while (!(flags & (1U<<4)));
    /* -----------------------------*/

	uint8_t zero = 0x00, cls = 0x52;
	gauge_write(0x61, &zero, 1);           /* BlockDataControl */
	gauge_write(0x3E, &cls,  1);           /* DataClass = 0x40 */
	gauge_write(0x3F, &zero, 1);           /* DataBlock = 0 */


	uint8_t old_csum, old_msb_DC, old_lsb_DC, old_msb_DCT, old_lsb_DCT, old_msb_CCT, old_lsb_CCT, old_msb_QC, old_lsb_QC, old_msb_TV, old_lsb_TV;
    HAL_Delay(100);
	gauge_read(0x60, &old_csum, 1);   // deve ser o mesmo checksum estável

//	uint8_t old_lsb_Qmax, old_msb_Qmax;
//	HAL_Delay(100);
//	gauge_read(0x00, &old_lsb_Qmax, 1);
//    HAL_Delay(100);
//	gauge_read(0x01, &old_msb_Qmax, 1);

    HAL_Delay(100);
	gauge_read(0x46, &old_lsb_DC, 1);
    HAL_Delay(100);
	gauge_read(0x47, &old_msb_DC, 1);
    HAL_Delay(100);
    uint8_t old_lsb_CTTC;
    uint8_t old_msb_CTTC;
	gauge_read(0x55, &old_lsb_CTTC, 1);
	gauge_read(0x56, &old_msb_CTTC, 1);
//    HAL_Delay(100);
//	gauge_read(0x56, &old_msb_CTTC, 1);
    HAL_Delay(100);
	gauge_read(0x40, &old_lsb_DCT, 1);
    HAL_Delay(100);
	gauge_read(0x41, &old_msb_DCT, 1);
    HAL_Delay(100);
	gauge_read(0x42, &old_lsb_CCT, 1);
    HAL_Delay(100);
	gauge_read(0x43, &old_msb_CCT, 1);
    HAL_Delay(100);
	gauge_read(0x44, &old_lsb_QC, 1);
    HAL_Delay(100);
	gauge_read(0x45, &old_msb_QC, 1);
	uint8_t old_lsb_mWh, old_msb_mWh;
    HAL_Delay(100);
	gauge_read(0x48, &old_lsb_mWh, 1);
    HAL_Delay(100);
	gauge_read(0x49, &old_msb_mWh, 1);
    HAL_Delay(100);
	gauge_read(0x4A, &old_lsb_TV, 1);
    HAL_Delay(100);
	gauge_read(0x4B, &old_msb_TV, 1);

	uint8_t msb_DC, lsb_DC, msb_DCT, lsb_DCT, msb_CCT, lsb_CCT, msb_QC, lsb_QC, msb_TV, lsb_TV;

//	uint8_t lsb_Qmax, msb_Qmax;
//	lsb_Qmax = 0xD6;
//	msb_Qmax = 0x00;
//	gauge_write(0x00, &lsb_Qmax, 1);
//	gauge_write(0x01, &msb_Qmax, 1);

	lsb_DC =  mAh       & 0xFF;
    msb_DC = (mAh >> 8) & 0xFF;
//	lsb_DC = 0x96;
//	msb_DC = 0x00;
	gauge_write(0x46, &lsb_DC, 1);
	gauge_write(0x47, &msb_DC, 1);

	uint8_t lsb_CTTC = 0xA;
	uint8_t msb_CTTC = 0x00;
	gauge_write(0x55, &lsb_CTTC, 1);
	gauge_write(0x56, &msb_CTTC, 1);

	lsb_DCT = 0x2;
	msb_DCT = 0x00;
	gauge_write(0x40, &lsb_DCT, 1);
	gauge_write(0x41, &msb_DCT, 1);
	lsb_CCT = 0x4;
	msb_CCT = 0x00;
	gauge_write(0x42, &lsb_CCT, 1);
	gauge_write(0x43, &msb_CCT, 1);
	lsb_QC = 0x1;
	msb_QC = 0x01;
	gauge_write(0x44, &lsb_QC, 1);
	gauge_write(0x45, &msb_QC, 1);
	lsb_TV =  mV       & 0xFF;
    msb_TV = (mV >> 8) & 0xFF;
//	lsb_TV = 0xAC;
//	msb_TV = 0x0D;
	gauge_write(0x4A, &lsb_TV, 1);
	gauge_write(0x4B, &msb_TV, 1);

	uint8_t lsb_mWh, msb_mWh;
    uint16_t mWh = (mAh/100)*(mV/100);
	lsb_mWh =  mWh       & 0xFF;
    msb_mWh = (mWh >> 8) & 0xFF;
//
    gauge_write(0x48, &lsb_mWh, 1);       // 0x04
    gauge_write(0x49, &msb_mWh, 1);       // 0xB0

    uint8_t old_sum = (0xFF - old_csum) & 0xFF;
//    uint8_t new_sum = (old_sum - old_msb_Qmax - old_lsb_Qmax - old_msb_DC - old_lsb_DC - old_msb_CTTC - old_lsb_CTTC - old_msb_DCT - old_lsb_DCT - old_msb_CCT - old_lsb_CCT - old_msb_QC - old_lsb_mWh - old_msb_mWh - old_lsb_QC - old_msb_TV - old_lsb_TV + lsb_Qmax + msb_Qmax + msb_DC + lsb_DC + msb_CTTC + lsb_CTTC + msb_DCT + lsb_DCT + msb_CCT + lsb_CCT + msb_QC + lsb_QC + msb_TV + lsb_TV + msb_mWh + lsb_mWh) & 0xFF;
    uint8_t new_sum = (old_sum - old_msb_DC - old_lsb_DC - old_msb_CTTC - old_lsb_CTTC - old_msb_DCT - old_lsb_DCT - old_msb_CCT - old_lsb_CCT - old_msb_QC - old_lsb_mWh - old_msb_mWh - old_lsb_QC - old_msb_TV - old_lsb_TV + msb_DC + lsb_DC + msb_CTTC + lsb_CTTC + msb_DCT + lsb_DCT + msb_CCT + lsb_CCT + msb_QC + lsb_QC + msb_TV + lsb_TV + msb_mWh + lsb_mWh) & 0xFF;
	uint8_t newCsum = (0xFF - new_sum) & 0xFF;

    gauge_write(0x60, &newCsum, 1);

    /*--- Esperar Flags()[4] = 1 ---*/
	do { flags = BQ27427_ReadFlags(&sensor); }
	while (!(flags & (1U<<4)));
    /*-----------------------------*/
    uint8_t init[2] = {0x0C, 0x00};
	gauge_write(0x00, init, 2);
	HAL_Delay(5);

    uint8_t rst[2] = {0x42, 0x00};
	gauge_write(0x00, rst, 2);
	HAL_Delay(5);
//    uint8_t rst2[2] = {0x41, 0x00};
//	gauge_write(0x00, rst2, 2);
//	HAL_Delay(5);

//	uint8_t seal[2] = {0x20, 0x00};
//	gauge_write(0x00, seal, 2);

}

void __attribute__((unused)) bq27427_shutdown(){
	uint8_t unseal[2] = {0x00, 0x80};
	gauge_write(0x00, unseal, 2);
	gauge_write(0x00, unseal, 2);

	uint8_t cmd_shutdown_enable[2] = { 0x1B, 0x00 };   // Little-endian: 0x001B
	gauge_write(0x00, cmd_shutdown_enable, 2);

	uint8_t cmd_shutdown[2] = { 0x1C, 0x00 };
	gauge_write(0x00, cmd_shutdown, 2);
}


//	uint8_t buf_status[2] = {0};
////	do{
//		HAL_I2C_Mem_Read(
//		&hi2c1,
//		BQ27427_I2C_ADDR, // HAL espera 7-bit<<1 para endereço 8-bit
//		0x00, // MemAddress = subcomando CONTROL_STATUS
//		I2C_MEMADD_SIZE_8BIT,
//		buf_status, // buffer de 2 bytes (LSB, MSB)
//		2, // queremos ler dois bytes
//		HAL_MAX_DELAY
//		);
//		// Agora combine LSB e MSB em um uint16_t:
//	//	control_status = (uint16_t)(buf_status[0] | (buf_status[1] << 8));
//		// Exemplo: verificar o bit ‘CHEMCHANGE’ (bit 0 do LSB):
//		HAL_Delay(2);
////	}while ( (buf_status[0] & 0x01) == 0 );
//
//	HAL_Delay(100);
//
////	uint8_t buf_ctrl[2];
////	// Envia Control(0x0000) para pedir CONTROL_STATUS
////	uint8_t ctrl_status_cmd[2] = {0x00, 0x00};
////	do{
////	gauge_write(0x00, ctrl_status_cmd, 2);
////	HAL_Delay(5);
////	// Lê 2 bytes em reg 0x00 → retorna CONTROL_STATUS[15:0]
////	HAL_I2C_Mem_Read(&hi2c1, BQ27427_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf_ctrl, 2, HAL_MAX_DELAY);
////	// Agora buf_ctrl[1] (byte alto) contém o bit [SS] em sua posição
////	}while( (buf_ctrl[1] & 0x80) != 0 );
//
////	uint8_t chem_id[2] = {0x08, 0x00};
////	gauge_write(0x00, chem_id, 2);
////	HAL_Delay(10);
//	uint8_t chem_id_r[2];
//	HAL_I2C_Mem_Read(&hi2c1,BQ27427_I2C_ADDR,0x0008,I2C_MEMADD_SIZE_8BIT,chem_id_r,2,HAL_MAX_DELAY);
//	uint16_t newChemID;
//	newChemID = chem_id_r[0] | (chem_id_r[1] << 8);
//	(void)newChemID;
//
////	uint16_t control;
////	do { control = BQ27427_Control(&sensor);
////	} while ((control & (1 << 0)) != 0 );
//
//	HAL_Delay(10);
//
////	uint8_t buf_status[2] = {0};
////	uint16_t control_status;
//	do{
//	HAL_I2C_Mem_Read(
//	&hi2c1,
//	BQ27427_I2C_ADDR, // HAL espera 7-bit<<1 para endereço 8-bit
//	0x0000, // MemAddress = subcomando CONTROL_STATUS
//	I2C_MEMADD_SIZE_8BIT,
//	buf_status, // buffer de 2 bytes (LSB, MSB)
//	2, // queremos ler dois bytes
//	HAL_MAX_DELAY
//	);
//	// Agora combine LSB e MSB em um uint16_t:
////	control_status = (uint16_t)(buf_status[0] | (buf_status[1] << 8));
//	// Exemplo: verificar o bit ‘CHEMCHANGE’ (bit 0 do LSB):
//	HAL_Delay(2);
//	}while ( (buf_status[0] & 0x01) == 0 );
//
//
////	do{
////	HAL_I2C_Mem_Read(
////	&hi2c1,
////	BQ27427_I2C_ADDR, // HAL espera 7-bit<<1 para endereço 8-bit
////	0x0000, // MemAddress = subcomando CONTROL_STATUS
////	I2C_MEMADD_SIZE_8BIT,
////	buf_status, // buffer de 2 bytes (LSB, MSB)
////	2, // queremos ler dois bytes
////	HAL_MAX_DELAY
////	);
////	// Agora combine LSB e MSB em um uint16_t:
//////	control_status = (uint16_t)(buf_status[0] | (buf_status[1] << 8));
////	// Exemplo: verificar o bit ‘CHEMCHANGE’ (bit 0 do LSB):
////	HAL_Delay(2);
////	}while ( (buf_status[0] & 0x01) == 0 );
//
	/*--- Esperar Flags()[4] = 1 ---*/

//	HAL_I2C_Mem_Read(&hi2c1,BQ27427_I2C_ADDR,0x0908,I2C_MEMADD_SIZE_8BIT,chem_id_r,2,HAL_MAX_DELAY);
//	newChemID = chem_id_r[0] | (chem_id_r[1] << 8);
//	(void)newChemID;
//    /*-----------------------------*/
////    uint8_t rst2[2] = {0x41, 0x00};
////	gauge_write(0x00, rst2, 2);
////	HAL_Delay(5);
//
////	do { control = BQ27427_Control(&sensor);
////	} while ((control & (1 << 0)) != 0 );
//
//
//	// Subcomando 0x0008 → Read ChemID
////	HAL_I2C_Mem_Write(&hi2c1, BQ27427_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x08, 0x00}, 2, HAL_MAX_DELAY);
////	HAL_Delay(10);
//
////	gauge_write(0x00, chem_id, 2);
//	HAL_Delay(10);
//	uint8_t buf[2];
//	HAL_I2C_Mem_Read(&hi2c1, BQ27427_I2C_ADDR, 0x0908, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
//	newChemID = buf[0] | (buf[1] << 8);
//	(void)newChemID;


//	do { control = BQ27427_Control(&sensor);
//	} while ((control & (1 << 0)) != 0 );


//	bq27427_set_valores_padrao(100, 3500);

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

//      uint16_t temperature = BQ27427_ReadTemperature(&sensor);
//      (void)temperature; // Evita o warning de variável não utilizada
//
//      uint16_t tensao = BQ27427_ReadVoltage(&sensor);
//      (void)tensao;
//
//      uint16_t battery_level = BQ27427_ReadStateOfCharge(&sensor);
//      (void)battery_level;
//
//      uint16_t capacidade_restante_em_mA = BQ27427_ReadRemainingCapacity(&sensor);
//      (void)capacidade_restante_em_mA;
//
//      uint16_t capacidade_total_disponivel_em_mAh = BQ27427_ReadFullChargeCapacity(&sensor);
//      (void)capacidade_total_disponivel_em_mAh;

//	  uint16_t nova_capacidade = BQ27427_SetDesignCapacity(&sensor, 150);
//	  (void) nova_capacidade;

//	  uint16_t nova_capacidade = BQ27427_SetDesignCapacity(&sensor, 150);
//	  (void) nova_capacidade;
	  uint8_t old_lsb_CTTC;
	  uint8_t old_msb_CTTC;
	  gauge_read(0x55, &old_lsb_CTTC, 1);
	  gauge_read(0x56, &old_msb_CTTC, 1);
	  uint16_t capacidade_da_bateria = BQ27427_GetDesignCapacity(&sensor);
	  (void)capacidade_da_bateria;
	  uint16_t nivel_da_bateria = BQ27427_ReadStateOfCharge(&sensor);
	  (void)nivel_da_bateria;
	  uint16_t voltage = BQ27427_ReadVoltage(&sensor);
	  (void)voltage;
	  uint16_t current = BQ27427_ReadAverageCurrent(&sensor);
	  (void)current;
      uint16_t NominalAvailableCapacity = BQ27427_ReadNominalAvailableCapacity(&sensor);
      (void)NominalAvailableCapacity;
      uint16_t FullAvailableCapacity = BQ27427_ReadFullAvailableCapacity(&sensor);
      (void)FullAvailableCapacity;
//      uint8_t bat_ins[] = {0x0C,0x00};
//      gauge_write(0x00, bat_ins, 2);
//      HAL_Delay(10);
      uint16_t RemainingCapacity = BQ27427_ReadRemainingCapacity(&sensor);
      (void)RemainingCapacity;
      uint16_t FullChargeCapacity = BQ27427_ReadFullChargeCapacity(&sensor);
      (void)FullChargeCapacity;
      uint16_t RemainingCapacityUnfiltered = BQ27427_ReadRemainingCapacityUnfiltered(&sensor);
      (void)RemainingCapacityUnfiltered;

//      uint16_t write_Design_Capacity = BQ27427_SetDesignCapacity(&sensor, 150);
//      (void) write_Design_Capacity;
//
//      uint16_t read_Design_Capacity = BQ27427_GetDesignCapacity(&sensor);
//      (void) read_Design_Capacity;

//	  uint16_t flags = BQ27427_ReadFlags(&sensor);
//	  (void)flags;
//      BQ27427_VerifyDesignCapacity(150);
//	  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, aTxBuffer, 3, HAL_MAX_DELAY);
//	  HAL_Delay(1000);
//	  HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, aRxBuffer, 3, HAL_MAX_DELAY);
//	  HAL_Delay(1000);
//	  HAL_I2C_Master_Transmit(&hi2c1, 0xAA, aTxBuffer, 3, HAL_MAX_DELAY);
//	  HAL_Delay(5);
//	  HAL_I2C_Master_Receive(&hi2c1, 0xAB, aRxBuffer, 3, HAL_MAX_DELAY);
//	  HAL_Delay(5);

//	  Control()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0100, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  Temperature()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0302, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //Voltage()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0504, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //Flags()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0706, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //NominalAvailableCapacity()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0908, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //FullAvailableCapacity()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0A0B, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //RemainingCapacity()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0D0C, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //FullChargeCapacity()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x0E0F, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //AverageCurrent()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x1110, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //AveragePower()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x1918, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //StateOfCharge()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x1D1C, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  InternalTemperature()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x1F1E, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //RemainingCapacityUnfiltered()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x2928, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //RemainingCapacityFiltered()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x2B2A, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //FullChargeCapacityUnfiltered()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x2D2C, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //FullChargeCapacityFiltered()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x2F2E, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
//	  //StateOfChargeUnfiltered()
//	  HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS, 0x3130, 1, aRxBuffer, 2, HAL_MAX_DELAY);
//	  HAL_Delay(660);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
