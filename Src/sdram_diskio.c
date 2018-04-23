/**
  ******************************************************************************
  * @file    sdram_diskio.c (based on sdram_diskio_template.c v2.0.2)
  * @brief   SDRAM Disk I/O driver
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection */

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sdram_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Block Size in Bytes */
#define BLOCK_SIZE                512

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS SDRAMDISK_initialize (BYTE);
DSTATUS SDRAMDISK_status (BYTE);
DRESULT SDRAMDISK_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SDRAMDISK_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SDRAMDISK_ioctl (BYTE, BYTE, void*);
#endif /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SDRAMDISK_Driver =
{
  SDRAMDISK_initialize,
  SDRAMDISK_status,
  SDRAMDISK_read,
#if  _USE_WRITE
  SDRAMDISK_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  SDRAMDISK_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SDRAMDISK_initialize(BYTE lun)
{
  Stat = STA_NOINIT;

  /* Configure the SDRAM device */
  if(BSP_SDRAM_Init() == SDRAM_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SDRAMDISK_status(BYTE lun)
{
  return Stat;
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */

//TV: Moved to user code because changes
DRESULT SDRAMDISK_read(BYTE lun, BYTE *buff, DWORD sector, UINT count) {
	uint32_t *pSrcBuffer = (uint32_t *) &buff;
	uint32_t BufferSize = (BLOCK_SIZE * count) / 4;
	uint32_t *pSdramAddress = (uint32_t *) (SDRAM_DISK_ADDR
			+ (sector * BLOCK_SIZE));

	uint8_t RES_OK;

	//TV: uint32_t *pSdramAddress = (uint32_t *) (SDRAM_DEVICE_ADDR + (sector * BLOCK_SIZE));
	RES_OK = BSP_SDRAM_ReadData((uint32_t) pSdramAddress,
			(uint32_t *) *pSrcBuffer, BufferSize);

	return RES_OK;
}

/* USER CODE END beforeWriteSection */

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */

//TV: Moved to user code because changes
#if _USE_WRITE == 1
DRESULT SDRAMDISK_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count) {
	uint32_t *pDstBuffer = (uint32_t *) &buff;
	uint32_t BufferSize = (BLOCK_SIZE * count) / 4;
	uint32_t *pSramAddress = (uint32_t *) (SDRAM_DISK_ADDR
			+ (sector * BLOCK_SIZE));

	uint8_t RES_OK;

	//TV: uint32_t *pSramAddress = (uint32_t *) (SDRAM_DEVICE_ADDR + (sector * BLOCK_SIZE));
	RES_OK = BSP_SDRAM_WriteData((uint32_t) pSramAddress,
			(uint32_t *) *pDstBuffer, BufferSize);

	//for (; BufferSize != 0; BufferSize--) {
	//	*(__IO uint32_t *) pSramAddress++ = *pDstBuffer++;
	//}

	return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE END beforeIoctlSection */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */

//TV: Moved to user code because changes
#if _USE_IOCTL == 1
DRESULT SDRAMDISK_ioctl(BYTE lun, BYTE cmd, void *buff) {
	DRESULT res = RES_ERROR;

	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	switch (cmd) {
	/* Make sure that no pending write process */
	case CTRL_SYNC:
		res = RES_OK;
		break;

		/* Get number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT:
		*(DWORD*) buff = SDRAM_DISK_SIZE / BLOCK_SIZE;
		res = RES_OK;
		break;

		/* Get R/W sector size (WORD) */
	case GET_SECTOR_SIZE:
		*(WORD*) buff = BLOCK_SIZE;
		res = RES_OK;
		break;

		/* Get erase block size in unit of sector (DWORD) */
	case GET_BLOCK_SIZE:
		*(DWORD*) buff = 1;
		res = RES_OK;
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

