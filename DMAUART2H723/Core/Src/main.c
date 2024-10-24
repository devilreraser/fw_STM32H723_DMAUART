/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

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

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */


#if defined( __ICCARM__ )
  #define DMA_BUFFER \
      _Pragma("location=\".dma_buffer\"")
#else
  #define DMA_BUFFER \
      __attribute__((section(".dma_buffer")))
#endif

/* application config */
uint8_t testSendingOnInit = 1;
uint8_t testSendingStopOnReceivedData = 1;

/* application status */
uint8_t testSending = 0;
uint8_t greenOn = 0;
uint8_t redOn = 0;
uint8_t yellowOn = 0;

/* statistics and debugging */

uint16_t errorCounter = 0;

uint16_t uartRXUnknownInterrupt = 0;
uint16_t uartRXParityErrorInterrupt = 0;
uint16_t uartRXFramingErrorInterrupt = 0;
uint16_t uartRXNoiseErrorInterrupt = 0;
uint16_t uartRXOverrunErrorInterrupt = 0;
uint16_t uartRXErrorInterrupt = 0;



uint16_t messagesTXCompleted = 0;				/* successfully sent messages */
uint16_t messagesTXCompletedUnexpected = 0;		/* unexpected (unknown not sent by us) successfully sent messages */
uint16_t messagesTXSkipped = 0;					/* skipped message sending because previous is not finished */
uint16_t messagesTXIdle = 0;					/* executed sendPreparedModbusResponse() w/ prepared response data (modbusResponseSize = 0) */

uint16_t uartRXActiveToIdleDetections = 0;		/* times entered RX active to idle line interrupt */
uint16_t uartRXFullBufferDetections = 0;		/* times entered RX dma full buffer interrupt */
uint16_t uartRXActiveToIdleNotExpected = 0;		/* not expected idle detections (still transmitting or message received while not processes previous received request) */
uint16_t uartRXFullBufferNotExpected = 0;		/* not expected buffer full (still transmitting or message received while not processes previous received request) */

uint16_t messagesRXProcessed = 0;				/* total received requests */
uint16_t messagesTXResponsesToRXRequests = 0;	/* responses to received requests */
uint16_t messagesNOResponsesToRXRequests = 0;	/* received requests without response */

uint16_t messagesRXNotProcessed = 0;
uint16_t messagesRXIdle = 0;					/* executed processReceivedModbusRequests() w/ received message to process (modbusBytesReceived = 0) */

uint16_t messageRXZeroBytes = 0;
uint16_t lastDMAUart2RXCounterTooBig = 0;

/* transmit semaphores */
volatile uint8_t sendingMessage = 0;
/* receive semaphores */
volatile uint8_t uartRXDeny = 0;
volatile uint8_t uartRXCompleteOnFullBuffer = 0;
volatile uint8_t uartRXCompleteOnActiveToIdleLine = 0;

/* Buffer and size for MODBUS response */
#define MODBUS_TX_BUFFER_SIZE 256  				/* Adjust this size based on expected response length */
DMA_BUFFER uint8_t modbusResponse[MODBUS_TX_BUFFER_SIZE];  /* Buffer for preparation of the MODBUS response message */
uint8_t modbusResponseSize = 0;

/* Buffer for Test Message Send */
const uint8_t messageTestSend[] = {0x01, 0x03, 0x02, 0x00, 0x10, 0xB9, 0x88};  /* Used for testing send */

/* Buffer and size for MODBUS requests */
#define MODBUS_RX_BUFFER_SIZE 256  				/* Adjust this size based on expected request length */
DMA_BUFFER uint8_t modbusRequest[MODBUS_RX_BUFFER_SIZE];  	/* Buffer to store incoming MODBUS requests */
volatile uint16_t modbusBytesReceived = 0;

/* timing */
uint32_t previousMillisTest = 0;   // To store the last tick for test message
const uint32_t intervalTest = 1000; // ms interval test

uint32_t previousMillisLedGreen = 0;   // To store the last tick for turn led off
uint32_t previousMillisLedRed = 0;   // To store the last tick for turn led off
uint32_t previousMillisLedYellow = 0;   // To store the last tick for turn led off
const uint32_t intervalLedGreen = 100; // ms interval for led
const uint32_t intervalLedRed = 100; // ms interval for led
const uint32_t intervalLedYellow = 1000; // ms interval for led



#define UART_BUFFER_SIZE 16       // Buffer for up to 16 messages
#define UART_MAX_MESSAGE_SIZE 256  // Max size of each message

typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE][UART_MAX_MESSAGE_SIZE];  // Message buffer
    uint16_t lengths[UART_BUFFER_SIZE];                       // Message lengths
    uint8_t head;  // Points to the next message to be sent
    uint8_t tail;  // Points to the next free slot
    uint8_t count; // Number of messages in the buffer
} UART_MessageBuffer;

UART_MessageBuffer uart_tx_buffer = { .head = 0, .tail = 0, .count = 0 };  // Initialize buffer

volatile uint8_t uart_transmitting = 0;  // Flag to indicate if transmission is ongoing
uint16_t debug_buffer_ovf = 0;
uint16_t debug_buffer_ovf_print = 0;
uint16_t debug_uart_handle_enterings = 0;
uint16_t debug_uart_handle_transmit_complete = 0;
uint16_t debug_uart_handle_transmit_empty = 0;
uint16_t debug_uart_handle_unknown = 0;
uint16_t debug_messagesTXCompleted = 0;
uint16_t debug_messagesTXCompletedUnexpected = 0;

uint16_t debug_uart_handle_receive = 0;
uint16_t debug_uart_handle_overrun_error = 0;
uint16_t debug_uart_handle_framing_error = 0;
uint16_t debug_uart_handle_parity_error = 0;
uint16_t debug_uart_handle_noise_error = 0;

uint16_t debug_uart_tx_empty_func_enterings = 0;
uint16_t debug_uart_try_send_func_enterings = 0;
uint16_t debug_uart_try_send_func_exec = 0;
uint16_t debug_uart_try_send_func_idle = 0;
uint16_t debug_uart_add_to_buffer = 0;
uint16_t debug_uart_try_send_func_exec_immediate = 0;


uint16_t debug_transmit_immediate_status_total = 0;
uint16_t debug_transmit_immediate_status_ok = 0;
uint16_t debug_transmit_immediate_status_error = 0;
uint16_t debug_transmit_immediate_status_busy = 0;
uint16_t debug_transmit_immediate_status_timeout = 0;

uint16_t debug_transmit_status_total = 0;
uint16_t debug_transmit_status_ok = 0;
uint16_t debug_transmit_status_error = 0;
uint16_t debug_transmit_status_busy = 0;
uint16_t debug_transmit_status_timeout = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void enableReceiveMessage(void);
void printBufferHex(char* tag, uint8_t* buffer, uint16_t size);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void UART_TxEmpty(void)
//{
//	debug_uart_tx_empty_func_enterings++;
//	__HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);  // Specifically disable UART Transmit Empty interrupt
//}
void UART_TrySendMessagesFromBuffer(void)
{
	debug_uart_try_send_func_enterings++;
	if (uart_tx_buffer.count)
	{
		debug_uart_try_send_func_exec++;


		uart_transmitting = 1;
		HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart3, uart_tx_buffer.buffer[uart_tx_buffer.head], uart_tx_buffer.lengths[uart_tx_buffer.head]);
		//__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC); // Clear Transmission Complete flag
		//__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);  // Specifically enable UART Transmit Complete interrupt
		debug_transmit_status_total++;
		if(status == HAL_OK)
		{
			debug_transmit_status_ok++;
			// Transmission of the current message is complete
			uart_tx_buffer.head = (uart_tx_buffer.head + 1) % UART_BUFFER_SIZE;  // Move to the next message
			uart_tx_buffer.count--;
		}
		if(status == HAL_ERROR)
		{
			debug_transmit_status_error++;
		}
		if(status == HAL_BUSY)
		{
			debug_transmit_status_busy++;
		}
		if(status == HAL_TIMEOUT)
		{
			debug_transmit_status_timeout++;
		}
	}
	else
	{
		debug_uart_try_send_func_idle++;
		uart_transmitting = 0;  // No more messages to send
		//__HAL_UART_DISABLE_IT(&huart3, UART_IT_TC);  // Specifically disable UART Transmit Complete interrupt
	}
}

void UART_AddMessageToBuffer(uint8_t *data, uint16_t len)
{
    if (uart_tx_buffer.count < UART_BUFFER_SIZE)
    {
        // Add the message to the buffer
    	debug_uart_add_to_buffer++;
        memcpy(uart_tx_buffer.buffer[uart_tx_buffer.tail], data, len);
        uart_tx_buffer.lengths[uart_tx_buffer.tail] = len;
        uart_tx_buffer.tail = (uart_tx_buffer.tail + 1) % UART_BUFFER_SIZE;  // Increment tail
        uart_tx_buffer.count++;

        // If UART is not busy, start sending the first message
		if (uart_transmitting == 0)
        {
        	debug_uart_try_send_func_exec_immediate++;
        	UART_TrySendMessagesFromBuffer();
        }
    }
    else
    {
        // Buffer is full, handle overflow (e.g., discard message or notify error)
        //printf("Buffer full, message discarded!\r\n");
        debug_buffer_ovf++;
    }
}

int _write(int file, char *data, int len)
{
    UART_AddMessageToBuffer((uint8_t*)data, len);  // Add message to the buffer
    return len;  // Return the length of the message
}


void setGreenLed(void)
{
	HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);
	greenOn = 1;
	previousMillisLedGreen = HAL_GetTick();
}

void setRedLed(void)
{
	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
	redOn = 1;
	previousMillisLedRed = HAL_GetTick();
}

void setYellowLed(void)
{
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
	yellowOn = 1;
	previousMillisLedYellow = HAL_GetTick();
}

void resetGreenLed(void)
{
	HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);
	greenOn = 0;
}

void resetRedLed(void)
{
	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
	redOn = 0;
}

void resetYellowLed(void)
{
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
	yellowOn = 0;
}

void modbusResponsePrepareTest(void)
{
	uint8_t size;
	size = sizeof(messageTestSend);
	memcpy(modbusResponse, messageTestSend, size);
	modbusResponseSize = size;
}

void sendPreparedModbusResponse(void)
{
	if (sendingMessage)
	{
		messagesTXSkipped++;
		return;
	}
	if (modbusResponseSize)
	{
		/* Start UART transmission using DMA */
        printf("Transmit Message of %d bytes\r\n", modbusResponseSize);  // Debugging line
		printBufferHex("TX", modbusResponse, modbusResponseSize);

		HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, modbusResponse, modbusResponseSize);
		if (status != HAL_OK)
		{
			printf("Error in sendPreparedModbusResponse HAL_UART_Transmit_DMA() call 1 returns:%d\r\n", status);
			status = HAL_UART_AbortTransmit(&huart2);
			if (status != HAL_OK)
			{
				printf("Error in sendPreparedModbusResponse HAL_UART_AbortTransmit() returns:%d\r\n", status);
			}
			status = HAL_UART_Transmit_DMA(&huart2, modbusResponse, modbusResponseSize);
			if (status != HAL_OK)
			{
				printf("Error in sendPreparedModbusResponse HAL_UART_AbortTransmit() call 2 returns:%d\r\n", status);
			}
		}
	    /* Enable Transmission Complete Interrupt for UART2 */
	    //__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);

		sendingMessage = 1;
		setGreenLed();
	}
	else
	{
		messagesTXIdle++;
	}
}

void modbusResponsePrepareForLastReceivedRequest(void)
{
	/* evaluate modbusBytesReceived and modbusRequest */
	/* prepare modbusResponseSize and modbusResponse */
	uint8_t size;
	size = sizeof(messageTestSend);
	memcpy(modbusResponse, messageTestSend, size);
	modbusResponseSize = size;
}



void configureDMAforUART2TX(void)
{
    /* Configure DMA for UART2 TX */
    hdma_usart2_tx.Instance = DMA1_Stream0;  /* Select appropriate stream for UART2 TX */
    hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
    	/* Initialization error */
    	errorCounter++;
        Error_Handler();
    }

    /* Link DMA handle to UART handle */
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
}

void configureDMAforUART2RX(void)
{
    /* Configure DMA for UART2 RX */
    hdma_usart2_rx.Instance = DMA1_Stream1;  /* Select the correct stream for UART2 RX */
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;  /* Circular mode for continuous reception */
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
        /* Initialization error */
    	errorCounter++;
        Error_Handler();
    }

    // Link DMA handle to UART handle
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/* Callback function to handle end of transmission */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {


    	HAL_DMA_StateTypeDef status = HAL_DMA_GetState(&hdma_usart2_tx);
    	uint32_t  u32ErrorDMA =            HAL_DMA_GetError(&hdma_usart2_tx);
    	if ((u32ErrorDMA != 0) || (status != HAL_DMA_STATE_READY))
		{
    		printf("HAL_DMA_GetState:%d HAL_DMA_GetError:%u\r\n", (int)status, (unsigned int)u32ErrorDMA);
		}


        /* Transmission complete, handle post-transmission actions here */
    	messagesTXCompleted++;
    	if (sendingMessage)
    	{
    		// Transmission complete, now we can reset the gState if expected
    		//printf("HAL_UART_TxCpltCallback huart->gState:%d on start\r\n", (int)huart->gState);
    		if (huart->gState == HAL_UART_STATE_BUSY_TX)
    		{
				huart->gState = HAL_UART_STATE_READY;
    		}
    		else
    		{
    			printf("HAL_UART_TxCpltCallback huart->gState:%d not expected\r\n",(int) huart->gState);
    		}
    		//printf("HAL_UART_TxCpltCallback huart->gState:%d on final\r\n",(int) huart->gState);

    		sendingMessage = 0;

			enableReceiveMessage();
	        /* Optional: Disable the TC interrupt after the transmission is complete */
	        //__HAL_UART_DISABLE_IT(huart, UART_IT_TC);
    	}
    	else
    	{
    		messagesTXCompletedUnexpected++;
    	}

    }

    if (huart->Instance == USART3)  // Only handle USART3
    {
    	debug_messagesTXCompleted++;
    	UART_TrySendMessagesFromBuffer();
    }

}


/* DMA reception complete callback (when buffer is filled) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Reception complete, buffer is full, process the received message */

		uartRXFullBufferDetections++;

        if (uartRXDeny)
        {
        	uartRXFullBufferNotExpected++;
        }
        else
        {
			if (modbusBytesReceived)
			{
				messagesRXNotProcessed++;
			}

			modbusBytesReceived = MODBUS_RX_BUFFER_SIZE;
			uartRXCompleteOnFullBuffer = 1;
			uartRXDeny = 1;
        }
    }
}



/* UART interrupt handler for idle line detection */
void USART2_IRQHandler(void)
{
    uint32_t isrflags   = READ_REG(huart2.Instance->ISR);
    uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE));

    if (errorflags != RESET)
    {
        /* Handle error flags */
        if ((isrflags & USART_ISR_PE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);
            // Parity error handling
            printf("UART Parity Error\r\n");
            uartRXParityErrorInterrupt++;
        }
        if ((isrflags & USART_ISR_FE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
            // Framing error handling
            printf("UART Framing Error\r\n");
            uartRXFramingErrorInterrupt++;
        }
        if ((isrflags & USART_ISR_NE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);
            // Noise error handling
            printf("UART Noise Error\r\n");
            uartRXNoiseErrorInterrupt++;
        }
        if ((isrflags & USART_ISR_ORE) != RESET)
        {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
            // Overrun error handling
            printf("UART Overrun Error\r\n");
            uartRXOverrunErrorInterrupt++;
        }

        // Optional: abort ongoing transmission or reception
        HAL_UART_AbortReceive_IT(&huart2); // abort receive operation

        uartRXErrorInterrupt++;

        return;
    }

    // Handle Transmission Complete (TC)
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) && __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TC))
    {
        HAL_UART_TxCpltCallback(&huart2); // Call the Transmission Complete callback
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);  // Disable the Transmission Complete interrupt
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC); // Clear Transmission Complete flag
        return;
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        /* Clear the idle line flag by reading the status register and data register */
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		// Stop the DMA temporarily to process the received data
		HAL_UART_DMAStop(&huart2);

		uartRXActiveToIdleDetections++;

        if (uartRXDeny)
        {
        	uartRXActiveToIdleNotExpected++;
        }
        else
        {
			if (modbusBytesReceived)
			{
				messagesRXNotProcessed++;
			}

			// Calculate the number of bytes received so far
			uint32_t lastDMAUart2RXCounter = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			if (lastDMAUart2RXCounter <= MODBUS_RX_BUFFER_SIZE)
			{
				modbusBytesReceived = MODBUS_RX_BUFFER_SIZE - lastDMAUart2RXCounter;
			}
			else
			{
				modbusBytesReceived = MODBUS_RX_BUFFER_SIZE;
				lastDMAUart2RXCounterTooBig++;
			}

			uartRXCompleteOnActiveToIdleLine = 1;

			uartRXDeny = 1;
        }
    }
    else
    {
    	uartRXUnknownInterrupt++;
    }
}

void enableReceiveMessageOnInit(void)
{
    /* Start receiving data using UART and DMA in circular mode */
    uartRXDeny = 0;

	HAL_UART_Receive_DMA(&huart2, modbusRequest, sizeof(modbusRequest));

    /* Clear the idle line flag by reading the status register and data register */
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    /* Enable UART idle line detection interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

}

void enableReceiveMessage(void)
{
	/* Restart DMA reception for the next request */
	uartRXDeny = 0;

	HAL_UART_Receive_DMA(&huart2, modbusRequest, sizeof(modbusRequest));

}

void printBufferHex(char* tag, uint8_t* buffer, uint16_t size)
{
	char printdata[256] = {0};
	uint16_t printedbytes = 0;

	do
	{
		if((printedbytes % 16) == 0)
		{
			if(strlen(printdata))
			{
				printf("%s\r\n",printdata);
			}
			sprintf(printdata, "%s: ", tag);
			if(printedbytes == 0)
			{
				for (int i = 0; i < strlen(tag); i++)
				{
					tag[i] = ' ';
				}
			}
		}
		sprintf(&printdata[strlen(printdata)], " %02X", buffer[printedbytes]);
		printedbytes++;

	}while(printedbytes < size);
	printf("%s\r\n",printdata);

}

void processReceivedModbusRequests(void)
{

	if ((uartRXCompleteOnActiveToIdleLine) || (uartRXCompleteOnFullBuffer))
	{

		printf("Received Message %d bytes Causes: Idle:%d Full:%d\r\n", modbusBytesReceived, uartRXCompleteOnActiveToIdleLine, uartRXCompleteOnFullBuffer);  // Debugging line

		uartRXCompleteOnActiveToIdleLine = 0;
		uartRXCompleteOnFullBuffer = 0;


		if (modbusBytesReceived)		/* received non-empty message */
		{
			setRedLed();
			if (testSendingStopOnReceivedData)
			{
				testSending = 0;
			}

			messagesRXProcessed++;

			printBufferHex("RX", modbusRequest, modbusBytesReceived);

			/* Prepare response to the received MODBUS request */
			modbusResponsePrepareForLastReceivedRequest();	/* modbusBytesReceived and modbusRequest used inside | modbusResponseSize and modbusResponse touched inside */
			modbusBytesReceived = 0;

			if (modbusResponseSize)
			{
				messagesTXResponsesToRXRequests++;
				sendPreparedModbusResponse();
			}
			else
			{
				messagesNOResponsesToRXRequests++;
				enableReceiveMessage();
			}
		}
		else
		{
			messageRXZeroBytes++;
			enableReceiveMessage();
		}
	}
	else
	{
		messagesRXIdle++;
	}
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  configureDMAforUART2TX();
  configureDMAforUART2RX();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\r\n");
  printf(".\r\n");
  printf("..\r\n");
  printf("...\r\n");
  printf("-----------------------------------\r\n");
  printf("Starting Application UART2 DMA Test\r\n");
  printf("-----------------------------------\r\n");
  printf("Initialization Message Number 1\r\n");
  printf("Initialization Message Number 2\r\n");
  printf("Initialization Message Number 3\r\n");
  printf("Initialization Message Number 4\r\n");
  printf("Initialization Message Number 5\r\n");
  printf("Initialization Message Number 6\r\n");
  printf("Initialization Message Number 7\r\n");
  printf("Initialization Message Number 8\r\n");
  printf("Initialization Message Number 9\r\n");
  printf("...\r\n");
  HAL_Delay(10);	//10 ms delay
  enableReceiveMessageOnInit();
  testSending = testSendingOnInit;

  while (1)
  {
	  processReceivedModbusRequests();


	  uint32_t currrentMillis = HAL_GetTick();

	  if (testSending)
	  {
		  /* Non-blocking Test ms delay */
		  if (currrentMillis - previousMillisTest >= intervalTest)
		  {
			  previousMillisTest = currrentMillis;  /* Update the last tick */
			  /* Perform an action every 500 ms */

			  modbusResponsePrepareTest();
			  sendPreparedModbusResponse();
		  }

	  }


	  if (greenOn)
	  {
		  /* Non-blocking Led Green ms delay */
		  if (currrentMillis - previousMillisLedGreen >= intervalLedGreen)
		  {
			  previousMillisLedGreen = currrentMillis;  /* Update the last tick */
			  resetGreenLed(); /* turn off led  */
		  }
	  }

	  if (redOn)
	  {
		  /* Non-blocking Led Red ms delay */
		  if (currrentMillis - previousMillisLedRed >= intervalLedRed)
		  {
			  previousMillisLedRed = currrentMillis;  /* Update the last tick */
			  resetRedLed();  /* turn off led  */
		  }
	  }

	  /* Non-blocking Led Yellow ms delay */
	  if (currrentMillis - previousMillisLedYellow >= intervalLedYellow)
	  {
		  previousMillisLedYellow = currrentMillis;  /* Update the last tick */
		  if (yellowOn)
		  {
			  resetYellowLed(); /* turn off led  */
		      printf("Yellow Pulse Off\r\n");  // Debugging line
		  }
		  else
		  {
			  setYellowLed(); /* turn on led  */
			  printf("Yellow Pulse On\r\n");  // Debugging line
		  }
	  }

    /* USER CODE END WHILE */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);  // Set high priority for USART2
  HAL_NVIC_EnableIRQ(USART2_IRQn);  // Enable interrupt for USART2
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);  // Set high priority for USART3
  HAL_NVIC_EnableIRQ(USART3_IRQn);  // Enable interrupt for USART3
  //__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);  // Specifically enable UART Transmit Complete interrupt
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
