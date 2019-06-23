#include "nrf24l01.h"
extern UART_HandleTypeDef huart2;

static void NRF_CS_SETPIN(nrf24l01_dev* nrf)
{
    HAL_GPIO_WritePin(nrf->NRF_CSN_GPIOx, nrf->NRF_CSN_GPIO_PIN,
        GPIO_PIN_SET);
}

static void NRF_CS_RESETPIN(nrf24l01_dev* nrf)
{
    HAL_GPIO_WritePin(nrf->NRF_CSN_GPIOx, nrf->NRF_CSN_GPIO_PIN, GPIO_PIN_RESET);
}

static void NRF_CE_SETPIN(nrf24l01_dev* nrf)
{
    HAL_GPIO_WritePin(nrf->NRF_CE_GPIOx, nrf->NRF_CE_GPIO_PIN, GPIO_PIN_SET);
}

static void NRF_CE_RESETPIN(nrf24l01_dev* nrf)
{
    HAL_GPIO_WritePin(nrf->NRF_CE_GPIOx, nrf->NRF_CE_GPIO_PIN, GPIO_PIN_RESET);
}


static NRF_RESULT NRF_SetupGPIO(nrf24l01_dev* nrf)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // CE pin
    GPIO_InitStructure.Pin = nrf->NRF_CE_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(nrf->NRF_CE_GPIOx, &GPIO_InitStructure);
    // end CE pin
		
    // IRQ pin
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Pin = nrf->NRF_IRQ_GPIO_PIN;  
		HAL_GPIO_Init(nrf->NRF_IRQ_GPIOx, &GPIO_InitStructure);
	
    // CSN pin
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Pin = nrf->NRF_CSN_GPIO_PIN;
    HAL_GPIO_Init(nrf->NRF_CSN_GPIOx, &GPIO_InitStructure);
	
    /* Enable and set EXTI Line Interrupt to the given priority */
    HAL_NVIC_SetPriority(nrf->NRF_IRQn, nrf->NRF_IRQ_preempt_priority,
        nrf->NRF_IRQ_sub_priority);
    HAL_NVIC_EnableIRQ(nrf->NRF_IRQn);
    // end IRQ pin
		
    NRF_CS_RESETPIN(nrf);
    NRF_CE_RESETPIN(nrf);

    return NRF_OK;
}


NRF_RESULT NRF_Init(nrf24l01_dev* nrf)
{
    NRF_SetupGPIO(nrf);	
    NRF_PowerUp(nrf, 1);
    uint8_t config = 0;
	
    while ((config & 2) == 0) { // wait for powerup
        NRF_ReadRegister(nrf, NRF_CONFIG, &config);
    }
	
    NRF_SetRXPayloadWidth_P0(nrf, nrf->PayloadLength);
    NRF_SetRXAddress_P0(nrf, nrf->RX_ADDRESS);
    NRF_SetTXAddress(nrf, nrf->TX_ADDRESS);
    NRF_EnableRXDataReadyIRQ(nrf, 1);
    NRF_EnableTXDataSentIRQ(nrf, 1);
    NRF_EnableCRC(nrf, 1);
    NRF_SetCRCWidth(nrf, nrf->CRC_WIDTH);
    NRF_SetAddressWidth(nrf, nrf->ADDR_WIDTH);
    NRF_SetRFChannel(nrf, nrf->RF_CHANNEL);
    NRF_SetDataRate(nrf, nrf->DATA_RATE);
    NRF_SetRetransmittionCount(nrf, nrf->RetransmitCount);
    NRF_SetRetransmittionDelay(nrf, nrf->RetransmitDelay);

    NRF_EnableRXPipe(nrf, 0);
    NRF_EnableAutoAcknowledgement(nrf, 0);
		NRF_ClearInterrupts(nrf);

    NRF_RXTXControl(nrf, NRF_STATE_RX);

    return NRF_OK;
}

NRF_RESULT NRF_SendCommand(nrf24l01_dev* nrf, uint8_t cmd, uint8_t* tx, uint8_t* rx,
    uint8_t len)
{
    uint8_t myTX[len + 1];
    uint8_t myRX[len + 1];
    myTX[0] = cmd;

    int i = 0;
    for (i = 0; i < len; i++) {
        myTX[1 + i] = tx[i];
        myRX[i] = 0;
    }

    NRF_CS_RESETPIN(nrf);
		
    if (HAL_SPI_TransmitReceive(nrf->spi, myTX, myRX, 1 + len, NRF_SPI_TIMEOUT)
        != HAL_OK) {
        return NRF_ERROR;
    }
				
    for (i = 0; i < len; i++) {
        rx[i] = myRX[1 + i];
    }

    NRF_CS_SETPIN(nrf);

    return NRF_OK;
}

void NRF_IRQ_Handler(nrf24l01_dev* nrf)
{
    uint8_t status = 0;
    if (NRF_ReadRegister(nrf, NRF_STATUS, &status) != NRF_OK) {
        return;
    }

    if ((status & (1 << 6))) { // RX FIFO Interrupt
        uint8_t fifo_status = 0;
        NRF_CE_RESETPIN(nrf);
      //NRF_WriteRegister(nrf, NRF_STATUS, &status);
        NRF_ReadRegister(nrf, NRF_FIFO_STATUS, &fifo_status);
        if (nrf->BUSY_FLAG == 1 && (fifo_status & 1) == 0) {
            NRF_ReadRXPayload(nrf, nrf->RX_BUFFER);
            status |= 1 << 6;
            NRF_WriteRegister(nrf, NRF_STATUS, &status);
            nrf->BUSY_FLAG = 0;
        }
				else{
					if(nrf->BUSY_FLAG == 1)
						HAL_UART_Transmit(&huart2, (uint8_t *)"Rx FIFO EMPTY Error\r\n", strlen("Rx FIFO EMPTY Error\r\n"), 10);
				}
        NRF_CE_SETPIN(nrf);
    }
    if ((status & (1 << 5))) { // TX Data Sent Interrupt
        status |= 1 << 5; // clear the interrupt flag
        NRF_CE_RESETPIN(nrf);
        NRF_RXTXControl(nrf, NRF_STATE_RX);
        nrf->STATE = NRF_STATE_RX;
        NRF_CE_SETPIN(nrf);
        NRF_WriteRegister(nrf, NRF_STATUS, &status);
        nrf->BUSY_FLAG = 0;
    }
}

NRF_RESULT NRF_ReadRegister(nrf24l01_dev* nrf, uint8_t reg, uint8_t* data)
{
    uint8_t tx = 0;
    if (NRF_SendCommand(nrf, NRF_CMD_R_REGISTER | reg, &tx, data, 1)
        != NRF_OK) {
				HAL_UART_Transmit(&huart2, (uint8_t *)"NRF_ReadRegister Error\r\n", strlen("NRF_ReadRegister Error\r\n"), 10);
        return NRF_ERROR;
				
    }
    return NRF_OK;
}

NRF_RESULT NRF_WriteRegister(nrf24l01_dev* nrf, uint8_t reg, uint8_t* data)
{
    uint8_t rx = 0;
    if (NRF_SendCommand(nrf, NRF_CMD_W_REGISTER | reg, data, &rx, 1)
        != NRF_OK) {
				HAL_UART_Transmit(&huart2, (uint8_t *)"NRF_WriteRegister Error\r\n", strlen("NRF_WriteRegister Error\r\n"), 10);
        return NRF_ERROR;
				
    }
    return NRF_OK;
}

NRF_RESULT NRF_ReadRXPayload(nrf24l01_dev* nrf, uint8_t* data)
{
    uint8_t tx[nrf->PayloadLength];
    if (NRF_SendCommand(nrf, NRF_CMD_R_RX_PAYLOAD, tx, data, nrf->PayloadLength)
        != NRF_OK) {
				HAL_UART_Transmit(&huart2, (uint8_t *)"RF_ReadRXPayload Error\r\n", strlen("RF_ReadRXPayload Error\r\n"), 10);
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_WriteTXPayload(nrf24l01_dev* nrf, uint8_t* data)
{
    uint8_t rx[nrf->PayloadLength];
    if (NRF_SendCommand(nrf, NRF_CMD_W_TX_PAYLOAD, data, rx, nrf->PayloadLength)
        != NRF_OK) {
				HAL_UART_Transmit(&huart2, (uint8_t *)"RF_WriteTXPayload Error\r\n", strlen("RF_writeTXPayload Error\r\n"), 10);
        return NRF_ERROR;
    }
    return NRF_OK;
}


NRF_RESULT NRF_SetDataRate(nrf24l01_dev* nrf, NRF_DATA_RATE rate)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (rate & 1) { // low bit set
        reg |= 1 << 5;
    } else { // low bit clear
        reg &= ~(1 << 5);
    }

    if (rate & 2) { // high bit set
        reg |= 1 << 3;
    } else { // high bit clear
        reg &= ~(1 << 3);
    }
    if (NRF_WriteRegister(nrf, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->DATA_RATE = rate;
    return NRF_OK;
}

NRF_RESULT NRF_SetTXPower(nrf24l01_dev* nrf, NRF_TX_PWR pwr)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    reg &= 0xF9; // clear bits 1,2
    reg |= pwr << 1; // set bits 1,2
    if (NRF_WriteRegister(nrf, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->TX_POWER = pwr;
    return NRF_OK;
}
NRF_RESULT NRF_ClearInterrupts(nrf24l01_dev* nrf)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= 7 << 4; // setting bits 4,5,6

    if (NRF_WriteRegister(nrf, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_SetRFChannel(nrf24l01_dev* nrf, uint8_t ch)
{
    ch &= 0x7F;
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_RF_CH, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= ch; // setting channel

    if (NRF_WriteRegister(nrf, NRF_RF_CH, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->RF_CHANNEL = ch;
    return NRF_OK;
}

NRF_RESULT NRF_SetRetransmittionCount(nrf24l01_dev* nrf, uint8_t count)
{
    count &= 0x0F;
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0xF0; // clearing bits 0,1,2,3
    reg |= count; // setting count

    if (NRF_WriteRegister(nrf, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->RetransmitCount = count;
    return NRF_OK;
}

NRF_RESULT NRF_SetRetransmittionDelay(nrf24l01_dev* nrf, uint8_t delay)
{
    delay &= 0x0F;
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x0F; // clearing bits 1,2,6,7
    reg |= delay << 4; // setting delay

    if (NRF_WriteRegister(nrf, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->RetransmitDelay = delay;
    return NRF_OK;
}

NRF_RESULT NRF_SetAddressWidth(nrf24l01_dev* nrf, NRF_ADDR_WIDTH width)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x03; 
    reg |= width; 
		
    if (NRF_WriteRegister(nrf, NRF_SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->ADDR_WIDTH = width;
    return NRF_OK;
}

NRF_RESULT NRF_EnableRXPipe(nrf24l01_dev* nrf, uint8_t pipe)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_EN_RXADDR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= 1 << pipe;

    if (NRF_WriteRegister(nrf, NRF_EN_RXADDR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_EnableAutoAcknowledgement(nrf24l01_dev* nrf, uint8_t pipe)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_EN_AA, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= 1 << pipe;

    if (NRF_WriteRegister(nrf, NRF_EN_AA, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_EnableCRC(nrf24l01_dev* nrf, uint8_t activate)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (activate) {
        reg |= 1 << 3;
    } else {
        reg &= ~(1 << 3);
    }

    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_SetCRCWidth(nrf24l01_dev* nrf, NRF_CRC_WIDTH width)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (width == NRF_CRC_WIDTH_2B) {
        reg |= 1 << 2;
    } else {
        reg &= ~(1 << 2);
    }

    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->CRC_WIDTH = width;
    return NRF_OK;
}

NRF_RESULT NRF_PowerUp(nrf24l01_dev* nrf, uint8_t powerUp)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (powerUp) {
        reg |= 1 << 1;
    } else {
        reg &= ~(1 << 1);
    }

    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_RXTXControl(nrf24l01_dev* nrf, NRF_TXRX_STATE rx)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (rx) {
        reg |= 1;
    } else {
        reg &= ~(1);
    }

    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_EnableRXDataReadyIRQ(nrf24l01_dev* nrf, uint8_t activate)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (!activate) {
        reg |= 1 << 6;
    } else {
        reg &= ~(1 << 6);
    }

    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_EnableTXDataSentIRQ(nrf24l01_dev* nrf, uint8_t activate)
{
    uint8_t reg = 0;
    if (NRF_ReadRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (!activate) {
        reg |= 1 << 5;
    } else {
        reg &= ~(1 << 5);
    }
    if (NRF_WriteRegister(nrf, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT NRF_SetRXAddress_P0(nrf24l01_dev* nrf, uint8_t* address)
{
    uint8_t rx[5];
    if (NRF_SendCommand(nrf, NRF_CMD_W_REGISTER | NRF_RX_ADDR_P0, address, rx,
            5)
        != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->RX_ADDRESS = address;
    return NRF_OK;
}

NRF_RESULT NRF_SetTXAddress(nrf24l01_dev* nrf, uint8_t* address)
{
    uint8_t rx[5];
    if (NRF_SendCommand(nrf, NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx, 5)
        != NRF_OK) {
        return NRF_ERROR;
    }
    nrf->TX_ADDRESS = address;
    return NRF_OK;
}

NRF_RESULT NRF_SetRXPayloadWidth_P0(nrf24l01_dev* nrf, uint8_t width)
{
    width &= 0x3F;
    if (NRF_WriteRegister(nrf, NRF_RX_PW_P0, &width) != NRF_OK) {
        nrf->PayloadLength = 0;
        return NRF_ERROR;
    }
    nrf->PayloadLength = width;
    return NRF_OK;
}

NRF_RESULT NRF_SendPacket(nrf24l01_dev* nrf, uint8_t* data)
{

    nrf->BUSY_FLAG = 1;

    NRF_CE_RESETPIN(nrf);
    NRF_RXTXControl(nrf, NRF_STATE_TX);
    NRF_WriteTXPayload(nrf, data);
    NRF_CE_SETPIN(nrf);

    while (nrf->BUSY_FLAG == 1) {
        ;
    } // wait for end of transmittion

    return NRF_OK;
}

NRF_RESULT NRF_ReceivePacket(nrf24l01_dev* nrf, uint8_t* data)
{

    nrf->BUSY_FLAG = 1;

    NRF_CE_RESETPIN(nrf);
    NRF_RXTXControl(nrf, NRF_STATE_RX);
    NRF_CE_SETPIN(nrf);

    while (nrf->BUSY_FLAG == 1) {
        ;
    } // wait for reception

    int i = 0;
    for (i = 0; i < nrf->PayloadLength; i++) {
        data[i] = nrf->RX_BUFFER[i];
    }

    return NRF_OK;
}
