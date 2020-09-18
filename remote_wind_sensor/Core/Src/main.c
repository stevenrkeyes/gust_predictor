#include "main.h"
#include "system_initialization.h"
#include "nrf24l01.h"

static ADC_HandleTypeDef hadc;
static I2C_HandleTypeDef hi2c1;
static SPI_HandleTypeDef hspi1;
static UART_HandleTypeDef huart2;

static void uint16_to_hex(uint8_t dest[], uint16_t value)
{
  uint8_t HEX_DIGITS[] = "0123456789abcdef";
  dest[3] = HEX_DIGITS[(value & 0x000F) >> 0];
  dest[2] = HEX_DIGITS[(value & 0x00F0) >> 4];
  dest[1] = HEX_DIGITS[(value & 0x0F00) >> 8];
  dest[0] = HEX_DIGITS[(value & 0xF000) >> 12];
}

#pragma pack(1)
typedef struct {
  uint8_t  device_id;
  uint16_t wind_measurement_raw;
} report_packet_t;

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  GPIO_Init();
  ADC_Init(&hadc);
  I2C1_Init(&hi2c1);
  SPI1_Init(&hspi1);
  USART2_UART_Init(&huart2);

  // ADC calibration seems to reduce the noise by several bits
  if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  // Initialize the radio driver
  // Todo: would be good to break all these initialization values out into a struct or something
  nRF24_Init(&hspi1, GPIOA, Aux_Out_1_Pin, GPIOA, Radio_NSS_Pin);

  uint8_t check_result = nRF24_Check();
  if (check_result == 1) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"radio good\n", 11, 1000);
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t *)"radio bad\n", 10, 1000);
  }

  nRF24_Start();

  // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)
  // Enable ShockBurst (which includes ack packets) for all RX pipes
  nRF24_EnableAA(0xFF);
  // Set RF channel
  nRF24_SetRFChannel(115);
  // Set data rate
  nRF24_SetDataRate(nRF24_DR_1Mbps);
  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_2byte);
  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(5);
  // Configure TX PIPE
  static const uint8_t nRF24_ADDR[] = { 0x01, 0x01, 0x01, 0x01, 0x01 };
  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);
  // Pipe 0 must be configured the same address as TX to receive the ack packet
  nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR);
  // Enable the payload length to be dynamic
  nRF24_SetDynamicPayloadLength(1);

  // Set TX power (maximum)
  nRF24_SetTXPower(nRF24_TXPWR_0dBm);
  // Set operational mode (PTX == transmitter)
  nRF24_SetOperationalMode(nRF24_MODE_TX);
  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();
  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  while (1)
  {
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 1000);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc);

    uint8_t adc_value_as_hex[4];
    uint16_to_hex(adc_value_as_hex, adc_value);

    HAL_UART_Transmit(&huart2, adc_value_as_hex, sizeof(adc_value_as_hex), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *) "\n", 1, 1000);

    // Prefix payload with Device ID
    report_packet_t report_packet = {0};
    report_packet.device_id = DEVICE_ID;
    report_packet.wind_measurement_raw = adc_value;

    // Transmit a packet
    nRF24_TXResult transmit_result = nRF24_TransmitPacket((uint8_t *) &report_packet, sizeof(report_packet));

    switch (transmit_result) {
        case nRF24_TX_SUCCESS:
            HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n", 3, 1000);
            break;
        case nRF24_TX_TIMEOUT:
            HAL_UART_Transmit(&huart2, (uint8_t *) "TIMEOUT\n", 8, 1000);
            break;
        case nRF24_TX_MAXRT:
            HAL_UART_Transmit(&huart2, (uint8_t *) "MAX RETRANSMIT\n", 15, 1000);
            break;
        default:
            HAL_UART_Transmit(&huart2, (uint8_t *) "ERROR\n", 6, 1000);
            break;
    }

    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    HAL_Delay(500);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}
