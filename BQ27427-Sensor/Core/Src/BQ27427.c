/*
 * BQ27427.c
 *
 *  Created on: Feb 6, 2025
 *      Author: Wendell
 */

/*
 * ******************************************************************************
 *  Includes   <System Includes> , "Project Includes"
 * ******************************************************************************
 */

#include "BQ27427.h"

/*
 * ******************************************************************************
 *  Variables
 * ******************************************************************************
 */

BQ27427_t bq_sensor; // Criação do objeto sensor

/*
 * ******************************************************************************
 *  Exported global functions (to be accessed by other files)
 * ******************************************************************************
 */

/*
 * ******************************************************************************
 *  Local Functions
 * ******************************************************************************
 */
/**
 * @brief Lê um registrador do sensor.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor.
 * @param command Endereço do registrador a ser lido.
 * @return Valor de 16 bits do registrador lido. Retorna 0xFFFF em caso de erro.
 */
static uint16_t BQ27427_Read(BQ27427_t *sensor_BQ27427, uint16_t command) {
    uint8_t aRxBuffer[2];
    uint16_t resultado;

    if (HAL_I2C_Mem_Read(sensor_BQ27427->hi2c, BQ27427_I2C_ADDR, command, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 2, BQ27427_MAX_DELAY) != HAL_OK) {
        resultado = 0xFFFF; // Retorna um valor inválido em caso de falha
    } else {
        resultado = (aRxBuffer[1] << 8) | aRxBuffer[0]; // Concatena os bytes em um valor de 16 bits
    }
    return resultado;
}

/**
 * @brief   Escreve dados em um registrador do gauge via I2C.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   command         Endereço do registrador a ser escrito.
 * @param   aTxBuffer       Ponteiro para o buffer de dados a serem enviados.
 * @param   len             Quantidade de bytes a escrever.
 * @return  Código de status HAL (HAL_OK em sucesso).
 */

static HAL_StatusTypeDef BQ27427_Write(BQ27427_t *sensor_BQ27427, uint8_t command, const uint8_t *aTxBuffer, uint8_t len){
    return HAL_I2C_Mem_Write(sensor_BQ27427->hi2c, BQ27427_I2C_ADDR, command, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aTxBuffer, len, BQ27427_MAX_DELAY);
}

/**
 * @brief Inicializa o sensor BQ27427.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor.
 * @param hi2c Ponteiro para a interface I2C utilizada na comunicação.
 */

//HAL_StatusTypeDef BQ27427_Init(BQ27427_t *sensor_BQ27427, I2C_HandleTypeDef *hi2c) {
//	/* 1. Salva handle I²C                 */
//	sensor_BQ27427->hi2c = hi2c;
//	/* 2. Confirma DeviceType ------------------------------------------------ */
//	uint8_t cmd_devtype[2] = {0x01, 0x00};
//	if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_devtype, 2) != HAL_OK) return HAL_ERROR;
//	if (BQ27427_Read(sensor_BQ27427, 0x0100) != 0x0427)                 return HAL_ERROR;
//
//	/* 3. Sair de CONFIG UPDATE se necessário -------------------------------- */
//	uint16_t flags = BQ27427_ReadFlags(sensor_BQ27427);
//	if (flags & (1U << 4)) {                               /* CFGUPMODE?     */
//		const uint8_t cmd_rst[2] = {0x42, 0x00};           /* SOFT_RESET     */
//		if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_rst, 2) != HAL_OK)  return HAL_ERROR;
//		HAL_Delay(5);
//		flags = BQ27427_ReadFlags(sensor_BQ27427);                      /* flags novos    */
//	}
//
//	/* 4. Garante que o gauge sabe que a bateria está presente --------------- */
//	if (!(flags & (1U << 15))) {                           /* BATT_PRESENT?  */
//		const uint8_t cmd_bat[2] = {0x0C, 0x00};           /* BAT_INSERT     */
//		if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_bat, 2) != HAL_OK)  return HAL_ERROR;
//		/* após BAT_INSERT o gauge volta ao modo INITIALIZE                   */
//	}
//
//	/* 5. Espera INITCOMP em CONTROL_STATUS ---------------------------------- */
//	const uint8_t cmd_cs[2] = {0x00, 0x00};                /* CONTROL_STATUS */
//	uint32_t t0 = HAL_GetTick();
//	while (1) {
//		if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_cs, 2) != HAL_OK)   return HAL_ERROR;
//		uint16_t cs = BQ27427_Read(sensor_BQ27427, 0x0100);
//		if (cs & 0x0080) break;                            /* bit 7 = INITCOMP */
//		if (HAL_GetTick() - t0 > 3000UL)                   /* 3 s timeout    */
//			return HAL_TIMEOUT;
//		HAL_Delay(20);
//	}
//
//	/* 6. Opcional: sincroniza valores suavizados ---------------------------- */
//	if (!BQ27427_Read(sensor_BQ27427, 0x28)) {                          /* RC Unfiltered  */
//		const uint8_t cmd_sync[2] = {0x19, 0x00};          /* SMOOTH_SYNC    */
//		if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_sync, 2) != HAL_OK) return HAL_ERROR;
//		HAL_Delay(10);
//	}
//
//	return HAL_OK;
//}

void BQ27427_Init(BQ27427_t *sensor_BQ27427, I2C_HandleTypeDef *hi2c) {
    sensor_BQ27427->hi2c = hi2c; // Associa a interface I2C ao sensor
}

/**
 * @brief Envia um comando de controle para o sensor.
 * @return Valor de resposta do comando de controle.
 */
uint16_t BQ27427_Control(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0100);
}

/**
 * @brief Lê a temperatura da bateria em décimos de grau Celsius.
 * @return Temperatura medida pelo sensor.
 */
uint16_t BQ27427_ReadTemperature(BQ27427_t *sensor_BQ27427) {
    return (BQ27427_Read(sensor_BQ27427, 0x0302) * 0.1) - 273.15;
}

/**
 * @brief Lê a voltagem atual da bateria em mV.
 * @return Voltagem da bateria.
 */
uint16_t BQ27427_ReadVoltage(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0504);
}

/**
 * @brief Lê os flags de status do sensor.
 * @return Flags indicando o estado da bateria.
 */
uint16_t BQ27427_ReadFlags(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0706);
}


/**
 * @brief Lê a capacidade nominal disponível da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade nominal disponível em mAh.
 */
uint16_t BQ27427_ReadNominalAvailableCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0908);
}

/**
 * @brief Lê a capacidade total disponível da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total disponível em mAh.
 */
uint16_t BQ27427_ReadFullAvailableCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0A0B);
}

/**
 * @brief Lê a capacidade restante da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante em mAh.
 */
uint16_t BQ27427_ReadRemainingCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0D0C);
}

/**
 * @brief Lê a capacidade total de carga da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga em mAh.
 */
uint16_t BQ27427_ReadFullChargeCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0E0F);
}

/**
 * @brief Lê a corrente média consumida pela bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Corrente média em mA.
 */
uint16_t BQ27427_ReadAverageCurrent(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1110);
}

/**
 * @brief Lê a potência média consumida pela bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Potência média em mW.
 */
uint16_t BQ27427_ReadAveragePower(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1918);
}

/**
 * @brief Lê o estado de carga da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Estado de carga em porcentagem (%).
 */
uint16_t BQ27427_ReadStateOfCharge(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1D1C);
}

/**
 * @brief Lê a temperatura interna do sensor.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Temperatura interna em décimos de grau Celsius.
 */
uint16_t BQ27427_ReadInternalTemperature(BQ27427_t *sensor_BQ27427) {
    return (BQ27427_Read(sensor_BQ27427, 0x1F1E) * 0.1) - 273.15;
}

/**
 * @brief Lê a capacidade restante não filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante não filtrada em mAh.
 */
uint16_t BQ27427_ReadRemainingCapacityUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2928);
}

/**
 * @brief Lê a capacidade restante filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante filtrada em mAh.
 */
uint16_t BQ27427_ReadRemainingCapacityFiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2B2A);
}

/**
 * @brief Lê a capacidade total de carga não filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga não filtrada em mAh.
 */
uint16_t BQ27427_ReadFullChargeCapacityUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2D2C);
}

/**
 * @brief Lê a capacidade total de carga filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga filtrada em mAh.
 */
uint16_t BQ27427_ReadFullChargeCapacityFiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2F2E);
}

/**
 * @brief Lê o estado de carga não filtrado da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Estado de carga não filtrado em porcentagem (%).
 */
uint16_t BQ27427_ReadStateOfChargeUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x3130);
}

uint16_t BQ27427_ReadDesignCapacity(BQ27427_t *sensor_BQ27427) {
//	 /* 1. Sai de CONFIG-UPDATE, se estiver */
//	uint16_t flags = BQ27427_ReadFlags(sensor_BQ27427);
//	if (flags & (1U << 4)) {                     /* bit 4 = CFGUPMODE */
//		const uint8_t rst[2] = {0x42, 0x00};     /* SOFT_RESET        */
//		if (BQ27427_Write(sensor_BQ27427, 0x00, rst, 2) != HAL_OK) return 0xFFFF;
//		HAL_Delay(5);
//	}
//
//	/* 2. Habilita IT se ainda não estiver ativo */
//	const uint8_t cmd_cs[2] = {0x00, 0x00};      /* CONTROL_STATUS    */
//	if (BQ27427_Write(sensor_BQ27427, 0x00, cmd_cs, 2) != HAL_OK) return 0xFFFF;
//	uint16_t cs = BQ27427_Read(sensor_BQ27427, 0x0100);       /* lê CtrlStatus     */
//
//	if (!(cs & 0x0010)) {                        /* bit 4 = QEN ?     */
//		const uint8_t it[2] = {0x21, 0x00};      /* IT_ENABLE         */
//		if (BQ27427_Write(sensor_BQ27427, 0x00, it, 2) != HAL_OK) return 0xFFFF;
//		HAL_Delay(2);
//	}

	/* 3. Lê Design Capacity via standard-command 0x3C/0x3D  */
	return BQ27427_Read(sensor_BQ27427, 0x3D3C);
}

/**
 * @brief   Lê dados de um registrador de 8 bits do gauge via I2C.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   command         Endereço do registrador a ser lido.
 * @param   aRxBuffer       Ponteiro para o buffer onde serão armazenados os dados lidos.
 * @param   len             Quantidade de bytes a ler.
 * @return  Código de status HAL (HAL_OK em sucesso).
 */

static HAL_StatusTypeDef BQ27427_ReadInternal(BQ27427_t *sensor_BQ27427, uint8_t command, uint8_t *aTxBuffer, uint8_t len){
    return HAL_I2C_Mem_Read(sensor_BQ27427->hi2c, BQ27427_I2C_ADDR, command, I2C_MEMADD_SIZE_8BIT, aTxBuffer, len, BQ27427_MAX_DELAY);
}

/**
 * @brief   Seleciona o bloco de dados e offset para acesso à RAM de configuração.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   blockClass      Identificador da classe de bloco (DataBlockClass).
 * @param   offset          Offset dentro do bloco (DataBlockOffset).
 * @return  Código de status HAL (HAL_OK em sucesso).
 */

static HAL_StatusTypeDef BQ27427_selectDataBlock(BQ27427_t *sensor_BQ27427, uint8_t blockClass, uint8_t offset){
    HAL_StatusTypeDef ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_REG_BLOCKDATA_CTRL, (uint8_t[]){0x00}, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_REG_DATACLASS, &blockClass, 1);
    if (ret != HAL_OK) return ret;
    return BQ27427_Write(sensor_BQ27427, BQ27427_REG_DATAOFFSET, &offset, 1);
}

/**
 * @brief   Calcula o checksum do bloco conforme especificado no datasheet.
 * @param   oldCsum  Valor de checksum atual lido.
 * @param   oldLsb   Byte LSB antigo do campo de dados.
 * @param   oldMsb   Byte MSB antigo do campo de dados.
 * @param   newLsb   Byte LSB novo a ser escrito.
 * @param   newMsb   Byte MSB novo a ser escrito.
 * @return  Novo valor de checksum para gravação.
 */

//static uint8_t BQ27427_calcChecksum(uint8_t oldCsum, uint8_t oldLsb, uint8_t oldMsb, uint8_t newLsb, uint8_t newMsb){
//    uint8_t sum = (uint8_t)(255 - oldCsum);
//    sum = (uint8_t)((sum - oldLsb - oldMsb + newLsb + newMsb) & 0xFF);
//    return (uint8_t)(255 - sum);
//}
static uint8_t BQ27427_calcChecksum(uint8_t oldCsum, uint8_t old_msb_DC, uint8_t old_lsb_DC, uint8_t old_lsb_CTTC, uint8_t old_msb_DCT, uint8_t old_lsb_DCT, uint8_t old_msb_CCT, uint8_t old_lsb_CCT, uint8_t old_msb_QC, uint8_t old_lsb_QC, uint8_t old_msb_mWh, uint8_t old_lsb_mWh, uint8_t old_msb_TV, uint8_t old_lsb_TV, uint8_t msb_DC, uint8_t lsb_DC, uint8_t lsb_CTTC, uint8_t msb_DCT, uint8_t lsb_DCT, uint8_t msb_CCT, uint8_t lsb_CCT, uint8_t msb_QC, uint8_t lsb_QC, uint8_t msb_TV, uint8_t lsb_TV, uint8_t msb_mWh, uint8_t lsb_mWh){
    uint8_t old_sum = (0xFF - oldCsum) & 0xFF;
	uint8_t new_sum = (old_sum - old_msb_DC - old_lsb_DC - old_lsb_CTTC - old_msb_DCT - old_lsb_DCT - old_msb_CCT - old_lsb_CCT - old_msb_QC - old_lsb_mWh - old_msb_mWh - old_lsb_QC - old_msb_TV - old_lsb_TV + msb_DC + lsb_DC + lsb_CTTC + msb_DCT + lsb_DCT + msb_CCT + lsb_CCT + msb_QC + lsb_QC + msb_TV + lsb_TV + msb_mWh + lsb_mWh) & 0xFF;
	uint8_t newCsum = (0xFF - new_sum) & 0xFF;

    return newCsum;
}

/**
 * @brief   Atualiza o parâmetro “Design Capacity” do gauge.
 * @param   sensor_BQ27427   Ponteiro para a estrutura do sensor inicializado.
 * @param   capacity_mAh     Capacidade desejada em mAh.
 * @return  HAL_OK em sucesso, código de erro HAL_I2C_… em caso de falha.
 */

//HAL_StatusTypeDef BQ27427_SetDesignCapacity(BQ27427_t *sensor_BQ27427, uint16_t capacity_mAh){
//    HAL_StatusTypeDef ret;
//    uint16_t flags;
//    uint8_t  oldLsb, oldMsb, oldCsum;
//    uint8_t  newLsb = (uint8_t)(capacity_mAh & 0xFF);
//    uint8_t  newMsb = (uint8_t)((capacity_mAh >> 8) & 0xFF);
//    uint8_t  newCsum;
//
//    // 1) Unseal twice
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
//
//    // 2) Enter CONFIG UPDATE
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);
//    HAL_Delay(1100);
//
//    // 3) Wait for CONFIG UPDATE flag
//    do {
//        flags = BQ27427_ReadFlags(sensor_BQ27427);
//    } while (!(flags & BQ27427_FLAG_CFGUPMODE));
//
//    // 4) Select State block, offset 0
//    ret = BQ27427_selectDataBlock(sensor_BQ27427, BQ27427_BLOCK_CLASS_STATE, 0);
//    if (ret != HAL_OK) return ret;
//
//    // 5) Read old LSB, MSB, checksum
//    HAL_Delay(100);
//    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &oldLsb, 1);
//    if (ret != HAL_OK) return ret;
//    HAL_Delay(100);
//    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &oldMsb, 1);
//    if (ret != HAL_OK) return ret;
//    HAL_Delay(100);
//    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM,       &oldCsum, 1);
//    if (ret != HAL_OK) return ret;
//
//    // 6) Calculate and write new values
//    newCsum = BQ27427_calcChecksum(oldCsum, oldLsb, oldMsb, newLsb, newMsb);
//    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &newLsb, 1);
//    if (ret != HAL_OK) return ret;
//    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &newMsb, 1);
//    if (ret != HAL_OK) return ret;
//    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM,      &newCsum, 1);
//    if (ret != HAL_OK) return ret;
//
//    // 7) Soft-reset to exit CONFIG UPDATE
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);
//    HAL_Delay(10);
//
//    // Optional: re-seal
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SEALED);
//
//    return HAL_OK;
//}
HAL_StatusTypeDef BQ27427_SetDesignCapacity(BQ27427_t *sensor_BQ27427, uint16_t capacity_mAh, uint16_t mV){
    HAL_StatusTypeDef ret;
    uint16_t flags;
    uint8_t old_csum, old_msb_DC, old_lsb_DC, old_lsb_CTTC, old_msb_DCT, old_lsb_DCT, old_msb_CCT, old_lsb_CCT, old_msb_QC, old_lsb_QC, old_msb_mWh, old_lsb_mWh, old_msb_TV, old_lsb_TV, msb_DC, lsb_DC, lsb_CTTC, msb_DCT, lsb_DCT, msb_CCT, lsb_CCT, msb_QC, lsb_QC, msb_TV, lsb_TV, msb_mWh, lsb_mWh;
    lsb_DC = (uint8_t)(capacity_mAh & 0xFF);
    msb_DC = (uint8_t)((capacity_mAh >> 8) & 0xFF);
    uint16_t mWh = capacity_mAh*mV/1000;
    lsb_mWh =  mWh       & 0xFF;
    msb_mWh = (mWh >> 8) & 0xFF;
    uint8_t newCsum;

    // 1) Unseal twice
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);

    // 2) Enter CONFIG UPDATE
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);
    HAL_Delay(1100);

    // 3) Wait for CONFIG UPDATE flag
    do {
        flags = BQ27427_ReadFlags(sensor_BQ27427);
    } while (!(flags & BQ27427_FLAG_CFGUPMODE));

    // 4) Select State block, offset 0
    ret = BQ27427_selectDataBlock(sensor_BQ27427, BQ27427_BLOCK_CLASS_STATE, 0);
    if (ret != HAL_OK) return ret;

    // 5) Read old LSB, MSB, checksum
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &old_lsb_DC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &old_msb_DC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_LSB, &old_lsb_CTTC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_LSB, &old_lsb_DCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_MSB, &old_msb_DCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_LSB, &old_lsb_CCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_MSB, &old_msb_CCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_LSB, &old_lsb_QC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_MSB, &old_msb_QC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_LSB, &old_lsb_TV, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_MSB, &old_msb_TV, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_LSB, &old_lsb_mWh, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_MSB, &old_msb_mWh, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(5);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM,       &old_csum, 1);
    if (ret != HAL_OK) return ret;

    // 6) Calculate and write new values
    newCsum = BQ27427_calcChecksum(old_csum, old_msb_DC, old_lsb_DC, old_lsb_CTTC, old_msb_DCT, old_lsb_DCT, old_msb_CCT, old_lsb_CCT, old_msb_QC, old_lsb_QC, old_msb_mWh, old_lsb_mWh, old_msb_TV, old_lsb_TV, msb_DC, lsb_DC, lsb_CTTC, msb_DCT, lsb_DCT, msb_CCT, lsb_CCT, msb_QC, lsb_QC, msb_TV, lsb_TV, msb_mWh, lsb_mWh);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_DC, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_DC, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_LSB, &lsb_CTTC, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_DCT, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_DCT, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_CCT, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_CCT, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_QC, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_QC, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_mWh, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_mWh, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_TV, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_TV, 1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM,      &newCsum, 1);
    if (ret != HAL_OK) return ret;

    // 7) Soft-reset to exit CONFIG UPDATE
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);
    HAL_Delay(10);

    // Optional: re-seal
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SEALED);

    return HAL_OK;
}

/**
 * @brief   Lê o valor atual de "Design Capacity" configurado no gauge.
 * @param   sensor_BQ27427 Ponteiro para o handle do sensor inicializado.
 * @return  Capacidade configurada em mAh, ou 0xFFFF em caso de erro.
 */

uint16_t BQ27427_GetDesignCapacity(BQ27427_t *sensor_BQ27427){
		HAL_StatusTypeDef ret;
//	    uint16_t flags;
	    uint16_t densign_capacity;
	    // 1) Unseal twice
	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);

	    // 2) Enter CONFIG UPDATE
	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);
	    HAL_Delay(1100);

	    // 3) Wait for CONFIG UPDATE flag
//	    do {
//	        flags = BQ27427_ReadFlags(sensor_BQ27427);
//	    } while (!(flags & BQ27427_FLAG_CFGUPMODE));

	    // 4) Select State block, offset 0
	    ret = BQ27427_selectDataBlock(sensor_BQ27427, BQ27427_BLOCK_CLASS_STATE, 0);
	    if (ret != HAL_OK) return ret;

	    HAL_Delay(100);
	    densign_capacity = BQ27427_Read(sensor_BQ27427, 0x4746);

	    // 7) Soft-reset to exit CONFIG UPDATE
	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);
	    HAL_Delay(10);

	    // Optional: re-seal
	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SEALED);

	    return densign_capacity;
}
