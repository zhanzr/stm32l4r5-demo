/*****************************************************************************
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/
#ifndef PERVASIVE_EINK_HARDWARE_DRIVER_H
#define PERVASIVE_EINK_HARDWARE_DRIVER_H

/* Header file includes */
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "pervasive_eink_configuration.h"


#define EINK_VccOn       HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET)

#define EINK_VccOff      HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET)
    
/* Push the chip select pin to logic HIGH */
#define ST_EINK_CsHigh         HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

/* Pull the chip select pin to logic LOW */
#define ST_EINK_CsLow         HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)

/* Push the reset pin to logic HIGH */
#define ST_EINK_RstHigh        HAL_GPIO_WritePin(DISP_RST_L_GPIO_Port, DISP_RST_L_Pin, GPIO_PIN_SET)

/* Pull the reset pin to logic LOW */
#define ST_EINK_RstLow        HAL_GPIO_WritePin(DISP_RST_L_GPIO_Port, DISP_RST_L_Pin, GPIO_PIN_RESET)

/* Push the discharge pin to logic HIGH */
#define ST_EINK_DischargeHigh   HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET)

/* Pull the discharge pin to logic LOW */
#define ST_EINK_DischargeLow  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET)

/* Push the border pin to logic HIGH */
#define ST_EINK_BorderHigh      HAL_GPIO_WritePin(BRD_CTRL_GPIO_Port, BRD_CTRL_Pin, GPIO_PIN_SET)

/* Pull the border  pin to logic LOW */
#define ST_EINK_BorderLow      HAL_GPIO_WritePin(BRD_CTRL_GPIO_Port, BRD_CTRL_Pin, GPIO_PIN_RESET)

/* Pull the Enable I/O pin to logic LOW */
#define ST_EINK_EnableIO      HAL_GPIO_WritePin(DispIoEn_GPIO_Port, DispIoEn_Pin, GPIO_PIN_RESET)

/* Push the Enable I/O pin to logic HIGH */
#define ST_EINK_DisableIO      HAL_GPIO_WritePin(DispIoEn_GPIO_Port, DispIoEn_Pin, GPIO_PIN_SET)

/* Functions used for E-INK display timing */
void        Cy_EINK_TimerInit(void);
void        Cy_EINK_TimerStop(void);
uint32_t    Cy_EINK_GetTimeTick(void);

/* Functions used for E-INK driver communication */
//void        Cy_EINK_InitSPI(void);
void        Cy_EINK_WriteSPI(uint8_t data);
uint8_t     Cy_EINK_ReadSPI(uint8_t data);

//Invalid Data Address for all white or all black denotion.
//Could not be used paritial updating
#define PV_EINK_WHITE_FRAME_ADDRESS  NULL
/* Final address of the main Flash region is used to denote the black frame */
#define PV_EINK_BLACK_FRAME_ADDRESS  (uint8_t*)(0x8000000+0x200000-1)

/* Data-type of E-INK status messages */
typedef enum
{
    PV_EINK_RES_OK,
    PV_EINK_ERROR_BUSY,
    PV_EINK_ERROR_ID,
    PV_EINK_ERROR_BREAKAGE,
    PV_EINK_ERROR_DC,
    PV_EINK_ERROR_CHARGEPUMP
}   pv_eink_status_t;

/* Data-type of E-INK update stages */
typedef enum 
{ 
    PV_EINK_STAGE1,
    PV_EINK_STAGE2,
    PV_EINK_STAGE3,
    PV_EINK_STAGE4
}   pv_eink_stage_t ;

/* Declarations of functions defined in pv_eink_hardware_driver.c */
/* Power control and initialization functions */
void                Pv_EINK_Init(void);
pv_eink_status_t    Pv_EINK_HardwarePowerOn(void);
pv_eink_status_t    Pv_EINK_HardwarePowerOff(void);

/* Set refresh time factor based on the ambient temperature */
void    Pv_EINK_SetTempFactor(int8_t temperature);

/* Display update functions */
void    Pv_EINK_FullStageHandler(uint8_t* imagePtr, pv_eink_stage_t stageNumber);
void    Pv_EINK_PartialStageHandler(uint8_t* previousImagePtr, uint8_t* newImagePtr);

#endif  /* PERVASIVE_EINK_HARDWARE_DRIVER_H */
/* [] END OF FILE */
