/*

  u8g_dev_sh1106_128x64.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  
*/

/* 初衷: 惠特128x64使用的SH1106芯片,这使得SSD1306总有些奇怪的情况.
 * 这里包含修改: 提供一个写入sh1106函数,使用[U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE)]调用
 */

#include "u8g.h"

#define WIDTH 128 //SH1106 -- 132x64, different from sh1106, But in here only use 128(col2-col129)
#define HEIGHT 64
#define PAGE_HEIGHT 8

/* init with HuiTec 1.3' SH1106 I2C OLED Module */
static const uint8_t u8g_dev_sh1106_128x64_huitec_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_RST(1),           /* do reset low pulse with (1*16)+2 milliseconds */
  U8G_ESC_CS(1),             /* enable chip */

  0x0ae,
  0x002, 0x010,
  0x040,
  0x0b0,
  0x081, 0x080,

  0x0a1,
  0x0a6,

  0x0a8, 0x03f,
  
  0x0ad, 0x08b,
  
  0x030,
  0x0c8,
  0x0d3, 0x000,
  0x0d5, 0x080,
  
  0x0d9, 0x01f,
  
  0x0da, 0x012,
  
  0x0db, 0x040,
  
  0x0af,

  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};


/* select one init sequence here */
#define u8g_dev_sh1106_128x64_init_seq u8g_dev_sh1106_128x64_huitec_init_seq

static const uint8_t u8g_dev_sh1106_128x64_data_start[] PROGMEM = {
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_CS(1),             /* enable chip */
  0x010,		/* set upper 4 bit of the col adr to 0 */
  //设置2强制初始化地址
  0x002,		/* set lower 4 bit of the col adr to 4  */
  U8G_ESC_END                /* end of sequence */
};

static const uint8_t u8g_dev_ssd13xx_sleep_on[] PROGMEM = {
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_CS(1),             /* enable chip */
  0x0ae,		/* display off */      
  U8G_ESC_CS(1),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

static const uint8_t u8g_dev_ssd13xx_sleep_off[] PROGMEM = {
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_CS(1),             /* enable chip */
  0x0af,		/* display on */      
  U8G_ESC_DLY(50),       /* delay 50 ms */
  U8G_ESC_CS(1),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

//@slboat 入口看起来是...初始化的入口功能
uint8_t u8g_dev_sh1106_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  //@slboat msg是个枚举的小玩意
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_sh1106_128x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_sh1106_128x64_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | pb->p.page); /* select current page (sh1106) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
        if ( u8g_pb_WriteBuffer(pb, u8g, dev) == 0 )
          return 0;
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_SLEEP_ON:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd13xx_sleep_on);    
      return 1;
    case U8G_DEV_MSG_SLEEP_OFF:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd13xx_sleep_off);    
      return 1;
  }
  return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_sh1106_128x64_2x_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_sh1106_128x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
	
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_sh1106_128x64_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | (pb->p.page*2)); /* select current page (sh1106) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
	u8g_WriteSequence(u8g, dev, pb->width, pb->buf); 
        u8g_SetChipSelect(u8g, dev, 0);
	
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_sh1106_128x64_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | (pb->p.page*2+1)); /* select current page (sh1106) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
	u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *)(pb->buf)+pb->width); 
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_SLEEP_ON:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd13xx_sleep_on);    
      return 1;
    case U8G_DEV_MSG_SLEEP_OFF:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd13xx_sleep_off);    
      return 1;
  }
  return u8g_dev_pb16v1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_dev_sh1106_128x64_sw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_sh1106_128x64_fn, U8G_COM_SW_SPI);
U8G_PB_DEV(u8g_dev_sh1106_128x64_hw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_sh1106_128x64_fn, U8G_COM_HW_SPI);
U8G_PB_DEV(u8g_dev_sh1106_128x64_i2c, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_sh1106_128x64_fn, U8G_COM_SSD_I2C);

uint8_t u8g_dev_sh1106_128x64_2x_buf[WIDTH*2] U8G_NOCOMMON ; 
u8g_pb_t u8g_dev_sh1106_128x64_2x_pb = { {16, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_sh1106_128x64_2x_buf}; 
u8g_dev_t u8g_dev_sh1106_128x64_2x_sw_spi = { u8g_dev_sh1106_128x64_2x_fn, &u8g_dev_sh1106_128x64_2x_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_sh1106_128x64_2x_hw_spi = { u8g_dev_sh1106_128x64_2x_fn, &u8g_dev_sh1106_128x64_2x_pb, U8G_COM_HW_SPI };
u8g_dev_t u8g_dev_sh1106_128x64_2x_i2c = { u8g_dev_sh1106_128x64_2x_fn, &u8g_dev_sh1106_128x64_2x_pb, U8G_COM_SSD_I2C };

