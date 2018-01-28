/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "pervasive_eink_configuration.h"
#include "pervasive_eink_hardware_driver.h"

#define	HZ	1000
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct strPointU8
{
    uint8_t x;
    uint8_t y;
}PointU8;

typedef struct strRectU8
{
    uint8_t l;
    uint8_t r;
    uint8_t t;
    uint8_t b;
}RectU8;

/* Data type of E-INK update types */
typedef enum
{ 
    CY_EINK_PARTIAL,
    CY_EINK_FULL_4STAGE,
    CY_EINK_FULL_2STAGE 
}   cy_eink_update_t;

/* Time in mS during which the logo will be displayed during startup */
#define LOGO_DELAY                      (uint16_t) (25000u)

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the code example document for the details of frame buffers */
#define NUMBNER_OF_FRAME_BUFFERS    (uint8_t) (0x02u)

/* Enumerated data type used to identify the frame buffers */
typedef enum
{
    BUFFER0 = 0x00,
    BUFFER1 = 0x01
}   frame_buffers_t;

/* Variable that stores the current frame buffer being used */
frame_buffers_t     currentFrameBuffer   = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the code
   example document for details of frame buffers */
uint8_t     frameBuffer[NUMBNER_OF_FRAME_BUFFERS][PV_EINK_IMAGE_SIZE];

/* Variable that stores the pointer to the current frame being displayed */
uint8_t*    currentFrame = PV_EINK_WHITE_FRAME_ADDRESS;

/* Variables from screen_contents.c that store images in flash */
extern uint8_t const logo                [PV_EINK_IMAGE_SIZE];
extern uint8_t const image2  [PV_EINK_IMAGE_SIZE];
     
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_120(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define ADC_NUMOFCHANNEL	3

__IO uint16_t g_adcBuf[ADC_NUMOFCHANNEL];


/*******************************************************************************
* Function Name: bool ST_EINK_Power(bool powerCtrl)
********************************************************************************
*
* Summary: This function is used to turn on/off the E-INK display power.
*
*  Note: This function can not be used to clear the E-INK display. The display 
*  will retain the previously written frame even when it's turned off.
*
* Parameters:
*  bool powerCtrl : "False" turns off and "True" turns on the display.
*
* Return:
*  bool           : "True" if operation was successful; "False" otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
bool ST_EINK_Power(bool powerCtrl)
{
    pv_eink_status_t pwrStatus;
    
    /* Turn on the E-INK power if powerCtrl is "true" */
    if (powerCtrl == true)
    {
        pwrStatus = Pv_EINK_HardwarePowerOn();
    }
    /* Turn off the E-INK power if powerCtrl is "false" */
    else
    {
        pwrStatus = Pv_EINK_HardwarePowerOff();
    }
    
    /* Return the outcome of the power control operation */
    return (pwrStatus == PV_EINK_RES_OK);
}

/*******************************************************************************
* Function Name: void ST_EINK_Clear(bool background, bool powerCycle)
********************************************************************************
*
* Summary: This function is used to clear the display to all white or all black
*  pixels.
*
*  Note1: The E-INK display should be powered on (using ST_EINK_Power function) 
*  before calling this function if "powerCycle" is false. Otherwise the display 
*  won't be cleared.
*
*  Note2: This function is intended to be called only after a reset/power up.
*  Use ST_EINK_ShowFrame() function to clear the display if you know the frame
*  that has been written to the display.
*
* Parameters:
*  bool background   : False for black background and True for white background.
*  bool powerCycle   : True for automatic power cycle. False otherwise
* 
* Return:
*  None
*
* Side Effects:
*  This is a blocking function that can take as many as 2 seconds 
*
*******************************************************************************/
void ST_EINK_Clear(bool background, bool powerCycle)
{
    if (powerCycle)
    {
        ST_EINK_Power(true);
    }
    
    if (background == true)
    {
        /* Two consecutive display updates to reduce ghosting */
        Pv_EINK_FullStageHandler(PV_EINK_WHITE_FRAME_ADDRESS, PV_EINK_STAGE4);
        Pv_EINK_FullStageHandler(PV_EINK_WHITE_FRAME_ADDRESS, PV_EINK_STAGE4);
    }
    else
    {
        /* Two consecutive display updates to reduce ghosting */
        Pv_EINK_FullStageHandler(PV_EINK_BLACK_FRAME_ADDRESS, PV_EINK_STAGE4);
        Pv_EINK_FullStageHandler(PV_EINK_BLACK_FRAME_ADDRESS, PV_EINK_STAGE4);
    }
    
    if (powerCycle)
    {
        ST_EINK_Power(false);
    }
}

/*******************************************************************************
* Function Name: void ST_EINK_ShowFrame(uint8_t* prevFrame, 
*      uint8_t* newFrame,CY_EINK_UpdateType updateType, bool powerCycle)
********************************************************************************
*
* Summary: Updates the E-INK display with a frame/image stored in the flash or RAM.
*
*  Notes: This function requires the previous frame data as well as the new frame
*  data. If the previous frame data changes from the actual frame previously 
*  written to the display, considerable ghosting may occur.
*
*  The E-INK display should be powered on (using ST_EINK_Power function) before 
*  calling this function, if "powerCycle" parameter is false. Otherwise the display 
*  won't be updated.
*
* Parameters:
*  uint8_t* prevFrame    : Pointer to the previous frame written on the display
*  uint8_t* newFrame     : Pointer to the new frame that need to be written
*  cy_eink_update_t              : Full update (2/4 stages) or  Partial update
*  bool powerCycle               : "true" for automatic power cycle, "false" for manual
*
*  Return:
*  None
*
* Side Effects:
*  This is a blocking function that can take as many as 2 seconds 
*
********************************************************************************/
void ST_EINK_ShowFrame(uint8_t* prevFrame, uint8_t* newFrame,
                       cy_eink_update_t updateType, bool powerCycle)
{
    /* If power cycle operation requested, turn on E-INK power */
    if (powerCycle)
    {
        ST_EINK_Power(true);
    }
    /* Partial update stage */
    if (updateType == CY_EINK_PARTIAL)
    {
        /* Update the display with changes from previous frame */
        Pv_EINK_PartialStageHandler(prevFrame, newFrame);
    }
    /* Full update stages */
    else if ((updateType == CY_EINK_FULL_4STAGE) || 
             (updateType == CY_EINK_FULL_2STAGE))
    {
        /* Stage 1: update the display with the inverted version of the previous 
           frame */
        Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE1);
        
        /* Additional stages that reduce ghosting for a 4 stage full update */
        if (updateType == CY_EINK_FULL_4STAGE)
        {
            /* Stage 2: update the display with an all white frame */
            Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE2);
            /* Stage 3: update the display with the inverted version of the new 
               frame */
            Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE3);
        }
        
        /* Stage 4: update the display with the new frame */
        Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE4);
    }
    else
    {
    }
    
    /* If power cycle operation requested, turn off E-INK power */
    if (powerCycle)
    {
        ST_EINK_Power(false);
    }
}

void InitDisplay(void)
{
    /* Initialize the E-INK display hardware and associated PSoC components */
    Pv_EINK_Init();
    
    /* Perform temperature compensation of E-INK parameters */
//    Pv_EINK_SetTempFactor(AMBIENT_TEMPERATURE);
    Pv_EINK_SetTempFactor(41);
    
    /* Power on the display and check if the operation is successful */
    if (ST_EINK_Power(true)== true)
    {
        /* Clear the display to white background */
        ST_EINK_Clear(true, true);
    }
    /* If the power on operation has failed */
    else
    {
        /* Halt the CPU */
        __SEV();
    }
}

void DisplayImage(uint8_t* imagePointer)
{
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
   /* Perform a partial update for a fast refresh as the main menu images 
           are similar */        
    ST_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_FULL_2STAGE, 
                          true);
    /* Store the pointer to the current image, which is required for subsequent 
       updates */
    currentFrame = imagePointer;
    
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	SystemClock_Config_120();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&g_adcBuf, ADC_NUMOFCHANNEL);

	printf("EInk Demo For STM32L4R5 Nucleo Board @ %u Hz, %u, %u, %u\n",
	SystemCoreClock,
	HAL_RCC_GetHCLKFreq(),
	HAL_RCC_GetSysClockFreq(),
	HZ
	);

/* Initializations */
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* Initialize the display, touch detection and low power modules */
    InitDisplay();

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		printf("%u %u %u\n",
		g_adcBuf[0],
		g_adcBuf[1],
		g_adcBuf[2]);

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        DisplayImage((uint8_t*)logo);
        HAL_Delay(LOGO_DELAY);
        
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		
        DisplayImage((uint8_t*)image2);
        HAL_Delay(LOGO_DELAY); 
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		HAL_Delay(5000);
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		HAL_Delay(5000);
//		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);							
//		HAL_Delay(5000);
  }
	return 0;
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//The clock generated by CubeMX could not correctly configure the clock to 120 MHz.
//It seems a bug of current device pack.
void SystemClock_Config_120(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable voltage range 1 boost mode for frequency above 80 Mhz */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* To avoid undershoot due to maximum frequency, select PLL as system clock source */
  /* with AHB prescaler divider 2 as first step */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* AHB prescaler divider at 1 as second step */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
