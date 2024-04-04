
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "main.h"
#include "motor.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t debouncer;


void I2C_WriteReg(uint16_t deviceAdd, uint8_t regAdd, uint8_t data);
int8_t I2C_ReadReg(uint16_t deviceAdd);
void I2C_SetRegAdd(uint16_t deviceAdd, uint8_t regAdd);
void SystemClock_Config(void);
void Gyroscope();
void GyroInit();

int16_t Xaxis;
int16_t Yaxis;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void HAL_SYSTICK_Callback(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
    switch(target_rpm) {
        case 80:
            target_rpm = 50;
            break;
        case 50:
            target_rpm = 81;
            break;
        case 0:
            target_rpm = 80;
            break;
        default:
            target_rpm = 0;
            break;
        }
    }
}

/* -------------------------------------------------------------------------------------------------------------
 * Main Program Code
 *
 * Starts initialization of peripherals
 * Blinks green LED (PC9) in loop as heartbeat
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t encoder_count = 0;

int main(int argc, char* argv[]) {
	Xaxis = 0;
	Yaxis = 0;
	
	debouncer = 0;                          // Initialize global variables
	HAL_Init();															// Initialize HAL
	LED_init();                             // Initialize LED's
	button_init();                          // Initialize button

	motor_init();                           // Initialize motor code
	
	GyroInit();
	
	

	while (1) {
			//GPIOC->ODR ^= GPIO_ODR_9;           // Toggle green LED (heartbeat)
			encoder_count = TIM2->CNT;
			Gyroscope();
			HAL_Delay(1);                      // Delay 1/8 second
	}
}



void Gyroscope() {
	int32_t threshold = 1300;
	
	I2C_SetRegAdd(0x69, 0x28);
	int8_t xlow = I2C_ReadReg(0x69);
	I2C_SetRegAdd(0x69, 0x29);
	int8_t xhigh = I2C_ReadReg(0x69);
	Xaxis = ((int16_t)xhigh << 8) | (uint8_t)xlow;

	I2C_SetRegAdd(0x69, 0x2A);
	int8_t ylow = I2C_ReadReg(0x69);
	I2C_SetRegAdd(0x69, 0x2B);
	int8_t yhigh = I2C_ReadReg(0x69);
	Yaxis = ((int16_t)yhigh << 8) | (uint8_t)ylow;

	GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 |
									GPIO_ODR_9);  // Reset the ODR bits for LEDs
	int fake = 300;
	Xaxis -= 20;
	if (Xaxis > fake)
	{
		GPIOC->ODR |= GPIO_ODR_6;  // Red LED for positive Y
	}
	else if (Xaxis < -fake)
	{
		GPIOC->ODR |= GPIO_ODR_7;  // Blue LED for negative Y
	}

	if (Xaxis > threshold)
	{
		GPIOC->ODR |= GPIO_ODR_9;  // Green LED for positive X
	}
	else if (Xaxis < -threshold)
	{
		GPIOC->ODR |= GPIO_ODR_8;  // Orange LED for negative X
	}

	//HAL_Delay(500);
}

void GyroInit() {	
	/* Configure the system clock */
	SystemClock_Config();
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	GPIO_InitTypeDef initc6789 = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc6789);

	// Configure PB11 in alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER11);  // Clear the bits
	GPIOB->MODER |= (GPIO_MODER_MODER11_1); // Set to alternate function mode

	// Configure PB11 as open-drain output
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;

	// Configure the alternate function for PB11 (assuming it's AF4 for I2C2_SDA, check your datasheet/reference manual)
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11; // Clear the bits
	GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos); // Set AF4 for I2C2_SDA

	// Configure PB13 in alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER13);  // Clear the bits
	GPIOB->MODER |= (GPIO_MODER_MODER13_1); // Set to alternate function mode

	// Configure PB11 as open-drain output
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

	// Configure the alternate function for PB13 it's AF5 for I2C2_SDA
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13; // Clear the bits
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos); // Set AF5 for I2C2_SCL

	// Configure PB14 in general purpose output mode
	GPIOB->MODER &= ~GPIO_MODER_MODER14;  // Clear the bits
	GPIOB->MODER |= GPIO_MODER_MODER14_0; // Set to general purpose output mode

	// Configure PB14 as push-pull output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;

	// Initialize/set PB14 high
	GPIOB->BSRR |= GPIO_BSRR_BS_14;

	// Configure PC0 in general purpose output mode
	GPIOC->MODER &= ~GPIO_MODER_MODER0;  // Clear the bits
	GPIOC->MODER |= GPIO_MODER_MODER0_0; // Set to general purpose output mode

	// Configure PC0 as push-pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;

	// Initialize/set PC0 high
	GPIOC->BSRR |= GPIO_BSRR_BS_0;

	// Enable I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	// Set the parameters for 100 kHz standard-mode in TIMINGR register
	// Assuming a 16 MHz system clock, adjust the values accordingly
	I2C2->TIMINGR = (0x1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0X4 << 20);

	// Enable the I2C2 peripheral using the PE bit in the CR1 register
	I2C2->CR1 |= I2C_CR1_PE;

	I2C_WriteReg(0x69, 0x20, 0xB);
}



void I2C_SetRegAdd(uint16_t deviceAdd, uint8_t regAdd) {
  I2C2->CR2 = 0;  // clear register
  // Use SADD[7:1] bit field in CR2 register to set slave address to addr
  I2C2->CR2 |= (deviceAdd << 1);
  // Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to
  // 1
  I2C2->CR2 |= (0x1 << 16);
  // Set RD_WRN to WRITE operation - 0 indicates WRITE
  I2C2->CR2 &= ~(1 << 10);
  // Set START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  // While TXIS or NACKF flags not set wait
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
  }  // getting stuck here on second call to this function!
  // Once TXIS flag set continue

  // Check if NACK set
  if (I2C2->ISR & I2C_ISR_NACKF) {
    // GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
  }

  // Write data into the TXDR
  I2C2->TXDR = regAdd;

  // Wait until TC flag set - transfer complete
  while (!(I2C2->ISR & I2C_ISR_TC)) {
  }
}


void I2C_WriteReg(uint16_t deviceAdd, uint8_t regAdd, uint8_t data) {
  I2C2->CR2 = 0;  // clear register
  // Use SADD[7:1] bit field in CR2 register to set slave address to addr
  I2C2->CR2 |= (deviceAdd << 1);
  // Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to
  // 1
  I2C2->CR2 |= (0x2 << 16);
  // Set RD_WRN to WRITE operation - 0 indicates WRITE
  I2C2->CR2 &= ~(1 << 10);
  // Set START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  // While TXIS or NACKF flags not set wait
  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
  }  // getting stuck here on second call to this function!
  // Once TXIS flag set continue

  // Check if NACK set
  if (I2C2->ISR & I2C_ISR_NACKF) {
    // GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
  }

  // Set reg address
  I2C2->TXDR = regAdd;

  while (!(I2C2->ISR & I2C_ISR_TXIS)) {
  }

  // Write data into the TXDR
  I2C2->TXDR = data;

  // Wait until TC flag set - transfer complete
  while (!(I2C2->ISR & I2C_ISR_TC)) {
  }
}

int8_t I2C_ReadReg(uint16_t deviceAdd) {
  I2C2->CR2 = 0;  // clear register
  int8_t data = 0;

  // SADD[7:1] bit field in CR2 register to set slave address to L3GD20
  I2C2->CR2 |= (deviceAdd << 1);
  //  NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
 
  I2C2->CR2 |= (0x1 << 16);
  // RD_WRN to READ operation - 1 indicates READ
  I2C2->CR2 |= (1 << 10);
  // START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  // While RXNE or NACKF flags not set wait
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {
  }
  // Once RXNE flag set continue

  // Check if NACK set
  if (I2C2->ISR & I2C_ISR_NACKF) {
    GPIOC->ODR |= GPIO_ODR_8;  // ORANGE - I2C not working!
  }

  // Wait for TC flag set
  while (!(I2C2->ISR & I2C_ISR_TC)) {
  }

  // Read contents of RXDR register and return data - remember it is 1 byte at a
  // time
  data = I2C2->RXDR;
  return data;
}

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

// ----------------------------------------------------------------------------
