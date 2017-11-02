/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*                                                                       */
/*   Portions COPYRIGHT 2015 STMicroelectronics                          */
/*   Portions Copyright (C) 2014, ChaN, all right reserved               */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

/**
  ******************************************************************************
  * @file    diskio.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    08-May-2015
  * @brief   FatFs low level disk I/O module.
  ******************************************************************************
  * @attention
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on a_mspnc"AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "ff_gen_drv.h"

extern SPI_HandleTypeDef hspi2;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SDSC    0
#define SDHC    1
#define SDXC    2
#define BUFFER_SIZE 512

/* Private variables ---------------------------------------------------------*/
extern Disk_drvTypeDef  disk;

/* Private Variables ---------------------------------------------------------*/
uint8_t cmd[] = {0x00,  // CMD0         0       Reset
                  0x08, // CMD8         1       check voltage range
                  55,   // CMD55        2       leading application command
                  41,   // ACMD41       3       Initialization
                  58,   // CMD58        4       Read OCR
                  16,   // CMD16        5       set Block length
                  17,   // CMD17        6       read single block
                  18,   // CMD18        7       read multiple block
                  12,   // CMD12        8       stop transmission
                  24,   // CMD24        9       Write single block
                  25}; // CMD25        10      Write multiple block        

uint8_t cmd_crc[] = {0x95,0xFF};
uint8_t cmd_arg[] = {0x00,0xFF};

uint8_t msg[] = {0xFF};
uint8_t rcv[] = {0x00};

uint8_t error_token = 0x00;     // Becomes 1 when reading block error happens
uint8_t stop_tran_token[] = {0xFD};

/* Private function prototypes -----------------------------------------------*/
void sd_command(uint8_t cmd, unsigned long arg);

void sd_read_single_block(BYTE *buffer, uint32_t arg);
void sd_read_multiple_block(BYTE *buffer, uint32_t arg, UINT count);

void sd_write_single_block(const BYTE *buffer, uint32_t block_address);
void sd_write_multiple_block(const BYTE *buffer, uint32_t arg, UINT count);

uint8_t response_R7();
uint8_t response_R1();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat;
  
  stat = disk.drv[pdrv]->disk_status(disk.lun[pdrv]);
  
  if(pdrv != 0)
    return STA_NOINIT;
  
  return stat;
}

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat = RES_OK;
  
  uint8_t response[8];
  uint8_t card_type = 0;
  uint8_t count = 18;    // Try command this many time

/*
  if(disk.is_initialized[pdrv] == 0)
  { 
    disk.is_initialized[pdrv] = 1;
    stat = disk.drv[pdrv]->disk_initialize(disk.lun[pdrv]);
  }     */

    // Disable CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    // send 74 clock cycles or more
    for(int i=0; i<15; i++)
    {
            HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    }

    // Enable CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    
    do
    {
        HAL_Delay(1);
      
        // Send CMD0 = 0x00
        sd_command(cmd[0], 0x00000000);
        
        // Send dummy 0xFF
        HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);

        HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    } while(rcv[0] !=0x01);
    
    // It is expected to send extra clock cycles to card so that card can finish it work
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    
    // every command transcation should end with CS disable
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  
  HAL_Delay(1);
  
    // Decide Card type
    {
      // Enable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      
      //HAL_Delay(1);       // Provide no clock and MISO line is HIGH
      sd_command(cmd[1], 0x000001AA);     // CMD8 =0x08
      
      for(uint8_t i=0; i<8;i++)
      {
        response[i] = response_R7();
      }
      
      if(rcv[0] == 0x05 )
      {
        // card is of type SDC v1 or MMVC v3
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);       // Red LED on
      }
      else if(response[5]== 0xAA)
      {
        // check if response is R1 + 32 bits
        // check the lower 12 bits. That should equal to 0x1AA
        // if it is then the card type is SDC v2 and can work at voltage range 2.7-3.6
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);       // Ob board LED
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        card_type = SDHC;
      }
      // Disable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
    
  HAL_Delay(1);
  
    // Send ACMD41 with HCS bit 1
    do
    {
      count = count - 1;
      
      HAL_Delay(1);
      
      // Enable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      
      // Send CMD55 first and then ACMD41
      sd_command(cmd[2],0x00000000);    // CMD55 55=0x77

      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
            
      sd_command(cmd[3],0x40000000);    // ACMD41 41=0x69

      for(uint8_t i=0; i<8;i++)
      {
        if(response_R1() == 0x00)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);     // Red LED ON = Response 0x00 rcvd;
          count = 0;      // WHEN THE RESPONSE R1 CHANGES FROM 0X01 TO 0X00, WE SHOULD BREAK THE DO WHILE LOOP
          break;          // AS WELL. SO WHEN THAT HAPPENS, SET THE COUNT = 0. THIS WILL MAKE THE DO WHILE LOOP INVALID
        }
      }
      
      // Send extra 0xFF before ending
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      
      // Disable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }while(count > 0);
    
  HAL_Delay(1);
  
    // CMD58 OCR Register. Response is R1 + 32 bits.
    // [----R1---] [Version] ----- [-Rsv-] ---- [Voltage] ---- [check_pattern]
    // [39-----31] [30---28] ----- [27-12] ---- [11----8] ---- [7-----------0]
    // Check bit 30 in the response. bit 30 is Card capacity status CCS
    // if CCS = 0 => card type is SDSC. 
    // if CCS = 1 => card stype is SDHC.
    {
      // Enable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      
      // Send CMD58 0x7A
      sd_command(cmd[4],0x00000000);
      
      // Get the R3 reponse
      for(uint8_t i=0; i<8;i++)
      {
        response[i] = response_R7();
      }
      
      // Check the CCS bit.
      if((response[2] & 0xC0) != 0x00)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // on board LED should be on
        card_type = SDHC;
      }
      else
      {
        card_type = SDSC;
      }
      
      // Disable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
  
  HAL_Delay(1);
    
    // Set the block size to 512 bytes
    {
      // Enable CS
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      
      // Send CMD16. Argument represents block size
      // CMD16 = 0x50 Set the block length 512 bytes
      sd_command(cmd[5], 0x00000200);
      
      // Check the Response. Should be 0x00 if successful
      for(uint8_t i=0; i<8; i++)
      {
        // Send few 0xFF and monitor response
        // when response = 0x00 => break the loop
        if(response_R1() != 0x00)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Red LED should not turn off 
        }
        else
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);       // On board LED should be off
          break;
        }
      }
      
      // Send extra FF before ending the command
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      
    }
    
  // Disbale CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  // This calls USER_initialize function and in that
  // stat variable is set to STA_NOINIT which is problem
    if(disk.is_initialized[pdrv] == 0)
  { 
    disk.is_initialized[pdrv] = 1;
    stat = disk.drv[pdrv]->disk_initialize(disk.lun[pdrv]);
  }
  
  return stat;  // keep this as it is
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	        /* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
  DRESULT res;
 
  res = disk.drv[pdrv]->disk_read(disk.lun[pdrv], buff, sector, count);
  
// Check how many sectors to read
  if(count == 1)
    sd_read_single_block(buff, sector); // CS HIGH done inside this function
  else
  {
    sd_read_multiple_block(buff, sector, count);  // CS HIGH done inside this function
    
    HAL_Delay(50);      // May be wait is required
    
    // Enable CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    
    // STOP the Transmission CMD12 = 0x4C 00 00 00 00 01
    sd_command(cmd[8], 0x00000000); // R1b response

    // Send extra FF and ignore the response
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    
    // Extra FF
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    
    /*// I should wait for MISO line to go high again
    do
    {
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    }while(rcv[0]!=0xFF);
    */
    
    // Disable CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  }
    return res;
    //return RES_OK;
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT disk_write (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count        	/* Number of sectors to write */
)
{
  DRESULT res;
  
  res = disk.drv[pdrv]->disk_write(disk.lun[pdrv], buff, sector, count);
  
  // Check how many sectors to write
  if(count == 1)
    sd_write_single_block(buff, sector);
  else
    // Stop Transaction command is included inside write block function
    sd_write_multiple_block(buff, sector, count); 
  
  return res;
  //return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  DRESULT res;

  res = disk.drv[pdrv]->disk_ioctl(disk.lun[pdrv], cmd, buff);
  
  switch(cmd)
  {
    /* flush dirty buffer if present */
  case CTRL_SYNC:
    // Enable CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    
    // Send FF to see if its ready
    // if it is then you'll get FF back
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    if(rcv[0]==0xFF)
    {
      res = RES_OK;
    }
    break;
    
  default:
    res = RES_PARERR;
    break;
    
  }
  
  // Disable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  
  return res;
}
#endif /* _USE_IOCTL == 1 */

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
__weak DWORD get_fattime (void)
{
  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/****************sd_card_api***************************************************/
void sd_command(uint8_t cmd, unsigned long arg)
{
    // Get the correct command
    uint8_t command = (cmd | 0x40);
    
    // Get the argument into bytes
    uint8_t byte1 = (arg & 0xff000000UL) >> 24;	// MSB = AA
    uint8_t byte2 = (arg & 0x00ff0000UL) >> 16;	// 01
    uint8_t byte3 = (arg & 0x0000ff00UL) >> 8;	// 00
    uint8_t byte4 = (arg & 0x000000ffUL);	// 00
    
    // CRC
    uint8_t CCRC[]= {0x95,0x87,0x01, 0x01, 0xFF};

    // Send the command 
    HAL_SPI_TransmitReceive(&hspi2, &command,(uint8_t*) rcv, 1, 2000);

    // send argument, bcoz arg is 32 bit long
    HAL_SPI_TransmitReceive(&hspi2,&byte1,(uint8_t*) rcv, 1, 2000);
    HAL_SPI_TransmitReceive(&hspi2,&byte2,(uint8_t*) rcv, 1, 2000);
    HAL_SPI_TransmitReceive(&hspi2,&byte3,(uint8_t*) rcv, 1, 2000);
    HAL_SPI_TransmitReceive(&hspi2,&byte4,(uint8_t*) rcv, 1, 2000);

    // send 0x95 for CRC for CMD0
    if(command == 0x40)         // CMD0        
      HAL_SPI_TransmitReceive(&hspi2,&CCRC[0],(uint8_t*) rcv, 1, 2000);
    else if(command == 0x48)    // CMD8
      HAL_SPI_TransmitReceive(&hspi2,&CCRC[1],(uint8_t*) rcv, 1, 2000);
    else if(command == 0x77)    // CMD55
      HAL_SPI_TransmitReceive(&hspi2,&CCRC[2],(uint8_t*) rcv, 1, 2000);
    else if(command == 0x69)    // ACMD41
      HAL_SPI_TransmitReceive(&hspi2,&CCRC[3],(uint8_t*) rcv, 1, 2000);
    else
      HAL_SPI_TransmitReceive(&hspi2,&CCRC[3],(uint8_t*) rcv, 1, 2000);
}

uint8_t response_R7()
{
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  return rcv[0];
}

uint8_t response_R1()
{
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  return rcv[0];
}

/*
 * Read Single sector
 */
void sd_read_single_block(BYTE *buffer, uint32_t block_address)
{
  // Variables
  uint32_t read_address = block_address;
  uint8_t  i=0;
  
  uint8_t rcv_token[] = {0x00};     // Receive token
  
  uint16_t j = 0;
  uint8_t  read_buffer[BUFFER_SIZE];
  
  // Enable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  
  // Check to see if the card is ready?
  do
  {
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  }while(rcv[0]!=0xFF);
    
  // Send read block command, CMD17 for single block read
  sd_command(cmd[6], read_address);     // CMD17 0x51
  
  // Command Response
  for(i=0; i<16; i++)
  {
    if(response_R1() != 0x00)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // if on board LED on = no response R1 
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);       // On board LED should stay off
      break;
    }
    
  }
  
  // Check if this FF returns data token in rcv
  // Extra 0xFF
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  
/*
 *  Once we get valid command Response R1, Data packet will follow
 *  Data token consists of Data toke, Data blocks and CRC
 *     Data Toke
 *             0xFE for CMD17/CMD18/CMD24
 *             0xFC for CMD25
 *             0xFD for stop tran token for CMD25
 *  if any Error, Error Token will be generated
 *     Error Token = 0 0 0 A B C D E
 *             A = Card is locked
 *             B = Out of Range
 *             C = Card ECC failed
 *             D = CC Error
 *             E = Error
 *  CRC must be read, 2 bytes
 *  Check data token
 */
  do
  {
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rcv_token, 1, 2000);
  }while(rcv_token[0] !=0xFE);

  // Read the block data
  for(j=0; j< BUFFER_SIZE; j++)
  {
    //HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg, &read_buffer[j], 1, 2000); // Working fine orig
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) buffer, 1, 2000);
    buffer = buffer +1; //increment buffer
  }
  
  /* Not original
  // Read CRC and ignore the response
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rcv, 1, 2000);
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rcv, 1, 2000);
  */
  
  // Disable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

/*
 * We only have to issue command 18 once
 * after that we should get response R1
 * after that repeat following
 * we should detect token FE for each data packet
 * and we should read 512 bytes followiung that FE
 */
void sd_read_multiple_block(BYTE *buffer, uint32_t block_address, UINT count)
{
  // Variables
  uint32_t read_address = block_address;
  uint8_t  i, count_block = 0;
  
  uint8_t rd_multi_token[] = {0x00};
  
  uint16_t j = 0;
  uint8_t  read_buffer[BUFFER_SIZE];
  
  count_block = count;
  
  // Enable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  
  // CMD18 = 0x52 for multiple block read
  sd_command(cmd[7], read_address);
  
  // Command Response
  for(i=0; i<16; i++)
  {
    if(response_R1() != 0x00)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // if on board LED on = no response R1 
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);       // On board LED should stay off
      break;
    }
    
  }
  
  // Extra 0xFF
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  
  for(i=0; i<count_block; i++)
  {
    // Data Token should be 0xFE
    do
    {
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rd_multi_token, 1, 2000);
    }while(rd_multi_token[0] !=0xFE);
    
    // Read the 512 bytes of sector
    for(j=0; j< BUFFER_SIZE; j++)
    {
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)read_buffer, 1, 2000);
    }
    
    /* Not original
    // Read CRC and ignore the response
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rcv, 1, 2000);
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*)rcv, 1, 2000);
    */
  }
  
  // Disable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void sd_write_single_block(const BYTE *buffer, uint32_t block_address)
{
  // Variables
  uint32_t write_address = block_address;
  uint8_t  responseR1, i=0;
  uint16_t j = 0;
  
  // Put ADC data here. For the time being, put random data
  // data represensts "This is two file"
  // uint8_t write_buffer[510] = {0x54,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x54,0x77,0x6F,0x20,0x66,0x69,0x6C,0x65};
  
  // Data token for CMD24
  uint8_t token[] = {0xFE};
  
  // Enable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  
  // Check to see if the card is ready?
  do
  {
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  }while(rcv[0]!=0xFF);
    
  // send CMD24 =0x58 Write single block
  sd_command(cmd[9],write_address);
  
  // Command Response
  for(i=0; i<16; i++)
  {
    if(response_R1() != 0x00)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // if on board LED on = no response R1 
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);       // On board LED should stay off
      break;
    }
  }     
    
  // Extra 0xFF
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  
  // Send Data Token, 0xFE for CMD24
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) token,(uint8_t*) rcv, 1, 2000);
  
  for(j=0; j<512; j++)
  {
    // Then the Data you want to write
    //HAL_SPI_TransmitReceive(&hspi2,&write_buffer[j],(uint8_t*) rcv, 1, 2000);   // Original
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) buffer,(uint8_t*) rcv, 1, 2000);
    buffer = buffer + 1;
  }
  
  // Send Any CRC. CRC is 2 bytes
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  
  for(i=0; i<16; i++)
  {
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    if(rcv[0]== 0xE5) // detect Valid data response
    {
      //wait untill MISO line goes high
      do{
          HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      }while(rcv[0]!=0xFF);

      break;
    }
  }

  // Disable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void sd_write_multiple_block(const BYTE *buffer, uint32_t block_address, UINT count)
{
  // Variables
  uint32_t write_address = block_address;
  uint8_t  responseR1, i=0;
  uint16_t j, k = 0;
  
  // Data token for CMD25
  uint8_t token[] = {0xFC};
  
  // Enable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  
  // TODO: Ready check here
  do
  {
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  }while(rcv[0]!=0xFF);
  
  
  // Send CMD25 = 0x59 Write multiple block
  sd_command(cmd[10],write_address);
  
  // Command Response
  for(i=0; i<16; i++)
  {
    if(response_R1() != 0x00)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // if on board LED on = no response R1 
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);       // On board LED should stay off
      break;
    }
    
  }
    
  for(k=0; k<count; k++)
  {
    
    // Send Data Token, 0xFC for CMD25
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) token,(uint8_t*) rcv, 1, 2000);
    
    for(j=0; j<512; j++)
    {
      // Then the Data you want to write
      HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)buffer,(uint8_t*) rcv, 1, 2000);
      buffer = buffer + 1;
    }
    
    // Send Any CRC. CRC is 2 bytes
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
    
    for(i=0; i<8; i++)
    {
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
      if(rcv[0]== 0xE5) // Valid data response
      {
        //wait untill MISO line goes high
        do{
            HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
        }while(rcv[0]!=0xFF);

        break;
      }
    }
    
  }
  
  // STOP Tran Token stop_tran_token
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) stop_tran_token,(uint8_t*) rcv, 1, 2000);

  // Send extra FF and ignore the response
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);

  // Extra FF
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  
  /* Not original*/
  //wait untill MISO line goes high
  do{
      HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) msg,(uint8_t*) rcv, 1, 2000);
  }while(rcv[0]!=0xFF);
  
  
  // Disable CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}