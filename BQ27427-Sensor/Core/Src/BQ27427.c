/**
 * ******************************************************************************
 *  @file    : BQ27427.c
 *  @brief   : Header for BQ27427.c file.
 *  @author  : Wendell
 * ******************************************************************************
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
 * ******************************************************************************
 * Function Name: BQ27427_Read
 * @brief:  Lê um registrador do sensor.
 * @param	:  Sensor_BQ27427 Ponteiro para a estrutura do sensor.
 * @param	:  Command Endereço do registrador a ser lido.
 * @return	:  Valor de 16 bits do registrador lido. Retorna 0xFFFF em caso de erro.
 * ******************************************************************************
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
 *  * ******************************************************************************
 * Function Name: BQ27427_Write
 * @brief   Escreve dados em um registrador do gauge via I2C.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   command         Endereço do registrador a ser escrito.
 * @param   aTxBuffer       Ponteiro para o buffer de dados a serem enviados.
 * @param   len             Quantidade de bytes a escrever.
 * @return  Código de status HAL (HAL_OK em sucesso).
 * ******************************************************************************
 */

static HAL_StatusTypeDef BQ27427_Write(BQ27427_t *sensor_BQ27427, uint8_t command, const uint8_t *aTxBuffer, uint8_t len){
    return HAL_I2C_Mem_Write(sensor_BQ27427->hi2c, BQ27427_I2C_ADDR, command, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aTxBuffer, len, BQ27427_MAX_DELAY);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_Init
 * @brief Inicializa o sensor BQ27427.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor.
 * @param hi2c Ponteiro para a interface I2C utilizada na comunicação.
 * ******************************************************************************
 */

void BQ27427_Init(BQ27427_t *sensor_BQ27427, I2C_HandleTypeDef *hi2c) {
    sensor_BQ27427->hi2c = hi2c; // Associa a interface I2C ao sensor
    uint16_t capacity = BQ27427_GetDesignCapacity(sensor_BQ27427);
    if (capacity != BATERRY_CAPACITY){
		BQ27427_SetChemistryProfile(sensor_BQ27427, CHEM_B);
		BQ27427_SetDesignCapacity(sensor_BQ27427, BATERRY_CAPACITY, BATERRY_TERMINATE_VOLTAGE);
    }
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_Control
 * @brief Envia um comando de controle para o sensor.
 * @return Valor de resposta do comando de controle.
 * ******************************************************************************
 */
uint16_t BQ27427_Control(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0100);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadTemperature
 * @brief Lê a temperatura da bateria em décimos de grau Celsius.
 * @return Temperatura medida pelo sensor.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadTemperature(BQ27427_t *sensor_BQ27427) {
    return (BQ27427_Read(sensor_BQ27427, 0x0302) * 0.1) - 273.15;
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadVoltage
 * @brief Lê a voltagem atual da bateria em mV.
 * @return Voltagem da bateria.
 * ******************************************************************************
 */

uint16_t BQ27427_ReadVoltage(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0504);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadFlags
 * @brief Lê os flags de status do sensor.
 * @return Flags indicando o estado da bateria.
 * ******************************************************************************
 */

uint16_t BQ27427_ReadFlags(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0706);
}


/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadNominalAvailableCapacity
 * @brief Lê a capacidade nominal disponível da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade nominal disponível em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadNominalAvailableCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0908);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadFullAvailableCapacity
 * @brief Lê a capacidade total disponível da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total disponível em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadFullAvailableCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0A0B);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadRemainingCapacity
 * @brief Lê a capacidade restante da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadRemainingCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0D0C);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadFullChargeCapacity
 * @brief Lê a capacidade total de carga da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadFullChargeCapacity(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x0E0F);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadAverageCurrent
 * @brief Lê a corrente média consumida pela bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Corrente média em mA.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadAverageCurrent(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1110);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadStateOfCharge
 * @brief Lê a potência média consumida pela bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Potência média em mW.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadAveragePower(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1918);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadStateOfCharge
 * @brief Lê o estado de carga da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Estado de carga em porcentagem (%).
 * ******************************************************************************
 */
uint16_t BQ27427_ReadStateOfCharge(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x1D1C);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadInternalTemperature
 * @brief Lê a temperatura interna do sensor.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Temperatura interna em décimos de grau Celsius.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadInternalTemperature(BQ27427_t *sensor_BQ27427) {
    return (BQ27427_Read(sensor_BQ27427, 0x1F1E) * 0.1) - 273.15;
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadRemainingCapacityUnfiltered
 * @brief Lê a capacidade restante não filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante não filtrada em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadRemainingCapacityUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2928);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadRemainingCapacityFiltered
 * @brief Lê a capacidade restante filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade restante filtrada em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadRemainingCapacityFiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2B2A);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadFullChargeCapacityFiltered
 * @brief Lê a capacidade total de carga não filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga não filtrada em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadFullChargeCapacityUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2D2C);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadFullChargeCapacityFiltered
 * @brief Lê a capacidade total de carga filtrada da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Capacidade total de carga filtrada em mAh.
 * ******************************************************************************
 */
uint16_t BQ27427_ReadFullChargeCapacityFiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x2F2E);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_ReadStateOfChargeUnfiltered
 * @brief Lê o estado de carga não filtrado da bateria.
 * @param sensor_BQ27427 Ponteiro para a estrutura do sensor BQ27427.
 * @return Estado de carga não filtrado em porcentagem (%).
 * ******************************************************************************
 */
uint16_t BQ27427_ReadStateOfChargeUnfiltered(BQ27427_t *sensor_BQ27427) {
    return BQ27427_Read(sensor_BQ27427, 0x3130);
}

/**
 * ******************************************************************************
 * Function Name: 2
 * @brief   Lê dados de um registrador de 8 bits do gauge via I2C.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   command         Endereço do registrador a ser lido.
 * @param   aRxBuffer       Ponteiro para o buffer onde serão armazenados os dados lidos.
 * @param   len             Quantidade de bytes a ler.
 * @return  Código de status HAL (HAL_OK em sucesso).
 * ******************************************************************************
 */

static HAL_StatusTypeDef BQ27427_ReadInternal(BQ27427_t *sensor_BQ27427, uint8_t command, uint8_t *aTxBuffer, uint8_t len){
    return HAL_I2C_Mem_Read(sensor_BQ27427->hi2c, BQ27427_I2C_ADDR, command, I2C_MEMADD_SIZE_8BIT, aTxBuffer, len, BQ27427_MAX_DELAY);
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_selectDataBlock
 * @brief   Seleciona o bloco de dados e offset para acesso à RAM de configuração.
 * @param   sensor_BQ27427  Ponteiro para a estrutura do sensor BQ27427.
 * @param   blockClass      Identificador da classe de bloco (DataBlockClass).
 * @param   offset          Offset dentro do bloco (DataBlockOffset).
 * @return  Código de status HAL (HAL_OK em sucesso).
 * ******************************************************************************
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
 * ******************************************************************************
 * Function Name: BQ27427_calcChecksum
 * @brief   Calcula o checksum do bloco de dados conforme especificado no datasheet
 *          do BQ27427, ajustando-o para refletir alterações em múltiplos
 *          campos de configuração.
 * @param   oldCsum        Checksum atual lido do bloco antes da modificação.
 * @param   old_msb_DC     Byte MSB antigo do campo Design Capacity.
 * @param   old_lsb_DC     Byte LSB antigo do campo Design Capacity.
 * @param   old_msb_CTTC   Byte MSB antigo do campo Charging Termination Current.
 * @param   old_lsb_CTTC   Byte LSB antigo do campo Charging Termination Current.
 * @param   old_msb_DCT    Byte MSB antigo do campo Discharge Current Threshold.
 * @param   old_lsb_DCT    Byte LSB antigo do campo Discharge Current Threshold.
 * @param   old_msb_CCT    Byte MSB antigo do campo Charge Current Threshold.
 * @param   old_lsb_CCT    Byte LSB antigo do campo Charge Current Threshold.
 * @param   old_msb_QC     Byte MSB antigo do campo Qmax (Quarterly Capacity).
 * @param   old_lsb_QC     Byte LSB antigo do campo Qmax (Quarterly Capacity).
 * @param   old_msb_mWh    Byte MSB antigo do campo Energy (mWh).
 * @param   old_lsb_mWh    Byte LSB antigo do campo Energy (mWh).
 * @param   old_msb_TV     Byte MSB antigo do campo Terminate Voltage.
 * @param   old_lsb_TV     Byte LSB antigo do campo Terminate Voltage.
 * @param   msb_DC         Byte MSB novo do campo Design Capacity.
 * @param   lsb_DC         Byte LSB novo do campo Design Capacity.
 * @param   msb_CTTC       Byte MSB novo do campo Charging Termination Current.
 * @param   lsb_CTTC       Byte LSB novo do campo Charging Termination Current.
 * @param   msb_DCT        Byte MSB novo do campo Discharge Current Threshold.
 * @param   lsb_DCT        Byte LSB novo do campo Discharge Current Threshold.
 * @param   msb_CCT        Byte MSB novo do campo Charge Current Threshold.
 * @param   lsb_CCT        Byte LSB novo do campo Charge Current Threshold.
 * @param   msb_QC         Byte MSB novo do campo Qmax (Quarterly Capacity).
 * @param   lsb_QC         Byte LSB novo do campo Qmax (Quarterly Capacity).
 * @param   msb_TV         Byte MSB novo do campo Terminate Voltage.
 * @param   lsb_TV         Byte LSB novo do campo Terminate Voltage.
 * @param   msb_mWh        Byte MSB novo do campo Energy (mWh).
 * @param   lsb_mWh        Byte LSB novo do campo Energy (mWh).
 * @return  Novo valor de checksum calculado para gravação no bloco de dados.
 * ******************************************************************************
 */

static uint8_t BQ27427_calcChecksum(uint8_t oldCsum, uint8_t old_msb_DC, uint8_t old_lsb_DC, uint8_t old_msb_CTTC, uint8_t old_lsb_CTTC, uint8_t old_msb_DCT, uint8_t old_lsb_DCT, uint8_t old_msb_CCT, uint8_t old_lsb_CCT, uint8_t old_msb_QC, uint8_t old_lsb_QC, uint8_t old_msb_mWh, uint8_t old_lsb_mWh, uint8_t old_msb_TV, uint8_t old_lsb_TV, uint8_t msb_DC, uint8_t lsb_DC, uint8_t msb_CTTC, uint8_t lsb_CTTC, uint8_t msb_DCT, uint8_t lsb_DCT, uint8_t msb_CCT, uint8_t lsb_CCT, uint8_t msb_QC, uint8_t lsb_QC, uint8_t msb_TV, uint8_t lsb_TV, uint8_t msb_mWh, uint8_t lsb_mWh){
    uint8_t old_sum = (0xFF - oldCsum) & 0xFF;
	uint8_t new_sum = (old_sum - old_msb_DC - old_lsb_DC - old_msb_CTTC - old_lsb_CTTC - old_msb_DCT - old_lsb_DCT - old_msb_CCT - old_lsb_CCT - old_msb_QC - old_lsb_mWh - old_msb_mWh - old_lsb_QC - old_msb_TV - old_lsb_TV + msb_DC + lsb_DC + msb_CTTC + lsb_CTTC + msb_DCT + lsb_DCT + msb_CCT + lsb_CCT + msb_QC + lsb_QC + msb_TV + lsb_TV + msb_mWh + lsb_mWh) & 0xFF;
	uint8_t newCsum = (0xFF - new_sum) & 0xFF;

    return newCsum;
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_SetDesignCapacity
 * @brief   Atualiza o parâmetro “Design Capacity” do gauge.
 * @param   sensor_BQ27427   Ponteiro para a estrutura do sensor inicializado.
 * @param   capacity_mAh     Capacidade desejada em mAh.
 * @return  HAL_OK em sucesso, código de erro HAL_I2C_… em caso de falha.
 * ******************************************************************************
 */

HAL_StatusTypeDef BQ27427_SetDesignCapacity(BQ27427_t *sensor_BQ27427, uint16_t capacity_mAh, uint16_t terminate_voltage){
    HAL_StatusTypeDef ret;
    uint16_t flags;
    uint8_t newCsum, old_csum, old_msb_DC, old_lsb_DC, old_lsb_CTTC, old_msb_CTTC, old_msb_DCT, old_lsb_DCT, old_msb_CCT, old_lsb_CCT, old_msb_QC, old_lsb_QC, old_msb_mWh, old_lsb_mWh, old_msb_TV, old_lsb_TV, msb_DC, lsb_DC, msb_CTTC, lsb_CTTC, msb_DCT, lsb_DCT, msb_CCT, lsb_CCT, msb_QC, lsb_QC, msb_TV, lsb_TV, msb_mWh, lsb_mWh;

    lsb_DC = (uint8_t)(capacity_mAh & 0xFF);
    msb_DC = (uint8_t)((capacity_mAh >> 8) & 0xFF);
	uint16_t taper_Rate = capacity_mAh / 10;
    lsb_CTTC =  taper_Rate       & 0xFF;
	msb_CTTC = (taper_Rate >> 8) & 0xFF;
    uint16_t chg_Current_Thr = capacity_mAh / 10;
	lsb_CCT =  chg_Current_Thr       & 0xFF;
    msb_CCT = (chg_Current_Thr >> 8) & 0xFF;
    uint16_t dsg_Current_Thr = capacity_mAh / 16.7;
	lsb_DCT =  dsg_Current_Thr       & 0xFF;
    msb_DCT = (dsg_Current_Thr >> 8) & 0xFF;
	lsb_TV =  terminate_voltage       & 0xFF;
    msb_TV = (terminate_voltage >> 8) & 0xFF;
    uint16_t quit_current = capacity_mAh / 25;
	lsb_QC =  quit_current       & 0xFF;
    msb_QC = (quit_current >> 8) & 0xFF;
    uint16_t mWh = (capacity_mAh/100)*(terminate_voltage/100);
    lsb_mWh =  mWh       & 0xFF;
    msb_mWh = (mWh >> 8) & 0xFF;

    /* 1) Unseal twice */
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);

    /* 2) Enter CONFIG UPDATE */
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);
    HAL_Delay(5);

    /* 3) Wait for CONFIG UPDATE flag */
    do {
        flags = BQ27427_ReadFlags(sensor_BQ27427);
    } while (!(flags & BQ27427_FLAG_CFGUPMODE));

    /* 4) Select State block, offset 0 */
    ret = BQ27427_selectDataBlock(sensor_BQ27427, BQ27427_BLOCK_CLASS_STATE, 0);
    if (ret != HAL_OK) return ret;

    /* 5) Read old LSB, MSB, checksum */
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM, &old_csum, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &old_lsb_DC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &old_msb_DC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_LSB, &old_lsb_CTTC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_MSB, &old_msb_CTTC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_LSB, &old_lsb_DCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_MSB, &old_msb_DCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_LSB, &old_lsb_CCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_MSB, &old_msb_CCT, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_LSB, &old_lsb_QC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_MSB, &old_msb_QC, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_mWH_LSB, &old_lsb_mWh, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_mWH_MSB, &old_msb_mWh, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_LSB, &old_lsb_TV, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_MSB, &old_msb_TV, 1);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);

    /* 6) Calculate and write new values */
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_LSB, &lsb_DC, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DESIGN_CAP_MSB, &msb_DC, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_LSB, &lsb_CTTC, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_TAPER_RATE_MSB, &msb_CTTC, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_LSB, &lsb_DCT, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_DSG_CURRENT_THRESHOLD_MSB, &msb_DCT, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_LSB, &lsb_CCT, 1);
    if (ret != HAL_OK) return ret;
//    HAL_Delay(1);
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_CHG_CURRENT_THRESHOLD_MSB, &msb_CCT, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_LSB, &lsb_QC, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_QUIT_CURRENT_MSB, &msb_QC, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_LSB, &lsb_TV, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_TERMINATE_VOLTAGE_MSB, &msb_TV, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_mWH_LSB, &lsb_mWh, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;
    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_mWH_MSB, &msb_mWh, 1);
//    HAL_Delay(1);
    if (ret != HAL_OK) return ret;

    newCsum = BQ27427_calcChecksum(old_csum, old_msb_DC, old_lsb_DC, old_msb_CTTC, old_lsb_CTTC, old_msb_DCT, old_lsb_DCT, old_msb_CCT, old_lsb_CCT, old_msb_QC, old_lsb_QC, old_msb_mWh, old_lsb_mWh, old_msb_TV, old_lsb_TV, msb_DC, lsb_DC, msb_CTTC, lsb_CTTC, msb_DCT, lsb_DCT, msb_CCT, lsb_CCT, msb_QC, lsb_QC, msb_TV, lsb_TV, msb_mWh, lsb_mWh);

    ret = BQ27427_Write(sensor_BQ27427, BQ27427_OFFSET_CHECKSUM,&newCsum, 1);
    if (ret != HAL_OK) return ret;

    do {
        flags = BQ27427_ReadFlags(sensor_BQ27427);
    } while (!(flags & BQ27427_FLAG_CFGUPMODE));

    /* 7) Forces the Flags() [BAT_DET] bit to set when the battery insertion detection is disabled via OpConfig [BIE] = 0*/
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_BAT_INSERT);
    HAL_Delay(1);

    /* 8) Soft-reset to exit CONFIG UPDATE */
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);
    HAL_Delay(1);

    /* Optional: re-seal */
//    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SEALED);

    return HAL_OK;
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_SetChemistryProfile
 * @brief   Configura o perfil de química do gauge BQ27427.
 *          Desbloqueia o sensor (unseal), entra em modo de atualização de configuração,
 *          grava o novo ID de química (CHEM_A, CHEM_B ou CHEM_C), e sai do modo
 *          por soft-reset, validando a operação ao ler novamente o CHEM_ID.
 * @param   sensor_BQ27427   Ponteiro para o handle do sensor BQ27427 inicializado.
 * @param   profile          Código do perfil de química a ser aplicado:
 *                            - CHEM_A
 *                            - CHEM_B
 *                            - CHEM_C
 * @return  HAL_StatusTypeDef HAL_OK em caso de sucesso, ou código de erro HAL em caso de falha.
 * ******************************************************************************
 */

HAL_StatusTypeDef BQ27427_SetChemistryProfile(BQ27427_t *sensor_BQ27427, uint8_t profile){
	HAL_StatusTypeDef ret;
	uint16_t flags;
	uint8_t cheim_id;

	/* 1) Unseal twice */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);

	/* 2) Write and Read CHEM_ID */
    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CHEM_ID); /*Primeiro escreve no endereço 0x08*/
	ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_CMD_CTRL_SUBCMD, &cheim_id, 2); /*E depois lê no endereço 0x08*/
	if (ret != HAL_OK) return ret;

	/* 3) Enter CONFIG UPDATE */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);

    /* 4) Wait for CONFIG UPDATE flag */
	do {
		flags = BQ27427_ReadFlags(sensor_BQ27427);
	} while (!(flags & BQ27427_FLAG_CFGUPMODE));

    /* 5) Write CHEM_ID */
	switch (profile) {
		case CHEM_A:
			BQ27427_sendCmd(sensor_BQ27427, BQ27427_CHEMISTRY_PROFILE_A);
			break;
		case CHEM_B:
			BQ27427_sendCmd(sensor_BQ27427, BQ27427_CHEMISTRY_PROFILE_B);
			break;
		case CHEM_C:
			BQ27427_sendCmd(sensor_BQ27427, BQ27427_CHEMISTRY_PROFILE_C);
			break;
		default:
			break;
	}

	/* 6) Soft-reset to exit CONFIG UPDATE */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);
	HAL_Delay(5);

    /* 7) Wait for CONFIG UPDATE flag */
	do {
		flags = BQ27427_ReadFlags(sensor_BQ27427);
	} while ((flags & BQ27427_FLAG_CFGUPMODE) != 0);

	/* 8) Read CHEM_ID */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CHEM_ID);
	ret = BQ27427_ReadInternal(sensor_BQ27427, BQ27427_CMD_CTRL_SUBCMD, &cheim_id, 2);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}

/**
 * ******************************************************************************
 * Function Name: BQ27427_GetDesignCapacity
 * @brief   Lê o valor atual de "Design Capacity" configurado no gauge.
 * @param   sensor_BQ27427 Ponteiro para o handle do sensor inicializado.
 * @return  Capacidade configurada em mAh, ou 0xFFFF em caso de erro.
 * ******************************************************************************
 */

uint16_t BQ27427_GetDesignCapacity(BQ27427_t *sensor_BQ27427){
	HAL_StatusTypeDef ret;
	uint16_t flags;
	uint16_t densign_capacity;
	/* 1) Unseal twice */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_UNSEAL);

	/* 2) Enter CONFIG UPDATE */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_CFGUPDATE);
//	HAL_Delay(1100);

	/* 3) Wait for CONFIG UPDATE flag */
	do {
		flags = BQ27427_ReadFlags(sensor_BQ27427);
	} while (!(flags & BQ27427_FLAG_CFGUPMODE));

	/* 4) Select State block, offset 0 */
	ret = BQ27427_selectDataBlock(sensor_BQ27427, BQ27427_BLOCK_CLASS_STATE, 0);
	if (ret != HAL_OK) return ret;

	HAL_Delay(1);
	densign_capacity = BQ27427_Read(sensor_BQ27427, 0x4746);

	/* 7) Soft-reset to exit CONFIG UPDATE */
	BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SOFT_RESET);

	/* Optional: re-seal */
//	    BQ27427_sendCmd(sensor_BQ27427, BQ27427_SUBCMD_SEALED);

	return densign_capacity;
}
