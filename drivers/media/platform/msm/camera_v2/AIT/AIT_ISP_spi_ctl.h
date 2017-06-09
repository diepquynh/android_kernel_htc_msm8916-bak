/*************************************************************************************************/
/* AIT ISP Host API                                                                              */
/* All rights reserved by Alpah Image Technology Corp                                            */
/*-----------------------------------------------------------------------------------------------*/
/* File name: AIT_ISP_spi_ctl.h                                                                  */
/* Description: ISP Host API SPI routines abstraction layer                                      */
/*                                                                                               */
/* Version 0.01   20141209                                                                       */
/*************************************************************************************************/

#ifndef _AIT_ISP_SPI_CTL_H_
#define _AIT_ISP_SPI_CTL_H_


#include "AIT_ISP_project.h"


/*************************************************************************************************/
/* SPI Interface                                                                                 */
/*************************************************************************************************/
void transmit_byte_via_SPI(uint16 addr, uint8 data);
void transmit_word_via_SPI(uint16 addr, uint16 data);
void transmit_multibytes_via_SPI(uint8* ptr, uint16 addr, uint16 length);
uint8 receive_byte_via_SPI(uint16 addr);
uint16 receive_word_via_SPI(uint16 addr);
void receive_multibytes_via_SPI(uint8* ptr, uint16 addr, uint16 length);


#endif /* _AIT_ISP_SPI_CTL_H_ */

