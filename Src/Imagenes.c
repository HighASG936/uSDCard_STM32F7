#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "ff.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"

//#define SIZEBUFFER	 0x30000

extern SD_HandleTypeDef hsd1;
extern HAL_SD_CardInfoTypedef SDCardInfo1;
extern	FIL myFile;
extern FATFS SDfatfs;
extern const int ALPHA;
	uint32_t bytesread;
//uint8_t bufferrd[SIZEBUFFER];
UINT ProcesarImagen(const BYTE *Pointer, UINT BitsToSent);

void SamplingImagen(TCHAR* myPath ,uint16_t Weight, uint16_t Height)
{
	uint32_t i, j;
	UINT BitsToSent, dmy;
	f_open(&myFile, myPath, 1);
	
	for(i = 272;i!=0 ;i--)
	{
		for(j = 0;j< Height;j++)
		{
			if(f_forward(&myFile, ProcesarImagen, BitsToSent, &dmy) != FR_OK)
			{
			__asm("NOP");
			}
		}
  }
}

UINT ProcesarImagen(const BYTE *Pointer, UINT BitsToSent)
{
	uint8_t cnt;
	if (BitsToSent == 0) cnt = 1;
	else
	{
		do {    
			Pointer++;
			cnt++;
	} while (cnt < BitsToSent);
	}
	return 0;
}
	
/*
void DrawImagen(TCHAR* myPath ,uint16_t Weight, uint16_t Height)
{
	int i,j,k=54,pixel;
	f_mount(&SDfatfs, "",1);	
	f_lseek(&myFile, f_size(&myFile));															//Expandir el expacio de trabajo ??
	f_open(&myFile,myPath, FA_OPEN_ALWAYS|FA_WRITE| FA_READ);	//Se abre el archivo
	f_read(&myFile, bufferrd, SIZEBUFFER, &bytesread);
	f_close(&myFile);
	
	for(i = 272;i!=0 ;i--)
	{
		for(j = 0;j< Height;j++)
		{
			pixel = ALPHA | (bufferrd[k]) | (bufferrd[k+1]<<8) | (bufferrd[k+2] <<16);
			BSP_LCD_DrawPixel(j,i, pixel);
			if(k> ( 50+ Weight*Height*3) ) return;
			k +=3;
		}
	}
}
*/
