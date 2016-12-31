#ifndef _IMAGENES_H
#define _IMAGENES_H

#include "stm32f7xx_hal.h"
#include "fatfs.h"

void DrawImagen(TCHAR* myPath ,uint16_t Weight, uint16_t Height);
void SamplingImagen(TCHAR* myPath ,uint16_t Weight, uint16_t Height);

#endif
