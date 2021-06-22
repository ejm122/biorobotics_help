/*
 *  Copyright (c) 2015-2018, Sensirion AG <joahnnes.winkelmann@sensirion.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Sensirion AG nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "i2chelper.h"
#include "sdpsensor.h"
#include "project.h"
#include "stdio.h"

//---------------------------------------------------------------------------------------------------------

//uint8_t i2c_write_SFM3xxx(uint8_t addr, uint16_t register_addr, uint8_t* data, uint16_t count)
////Basic I2C write function, sends write header and then commands
//{
//    int status;
//    int i;
//    status = I2C_1_MasterSendStart(addr, I2C_1_WRITE_XFER_MODE); //begin transmission in write-mode
//    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
//    {
//        /* Send register-address to write */
//        uint8_t reg_MSB = (register_addr >> 8) & 0xFF;
//        uint8_t reg_LSB = register_addr & 0xFF;
//        status = I2C_1_MasterWriteByte(reg_MSB);
//        status |= I2C_1_MasterWriteByte(reg_LSB);
//    }
//    if(status == I2C_1_MSTR_NO_ERROR) //If there is no error
//    {
//        for (i = 1; i < 500; ++i)
//        {
//        status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE); //continue transmission in read-mode
//        /* Read array of count bytes */
//        for(i=0; i<count; i++)
//        {
//            if(i < count-1)
//            {
//                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
//            }
//            else
//            {
//                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK */
//            }
//        }
//        uint16_t flow_raw = (data[0] << 8) | data[1];
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(n,flow,a,b,(float)data[2]);
//        }
//    }
//    I2C_1_MasterSendStop(); /* Send Stop */    
//    return status;
//}

//
int8_t i2c_read(uint8_t addr, uint8_t* data, uint16_t count)
//basic I2C read function, sends read header and saves data received
{
    int status;
    int i;

    /* Read data array */
    //status = I2C_1_MasterSendRestart(addr, I2C_1_READ_XFER_MODE);
    status = I2C_1_MasterSendStart(addr, I2C_1_READ_XFER_MODE); //send read header

    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    // if(1) /* Cheating */
    {
        /* Read array of 9 bytes */
        for(i=0; i<count; i++)
        {
            if(i < count-1)
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA); /* non-last byte send ACK */
            }
            else
            {
                data[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA); /* last byte send NACK - maybe don't to this? ACK every time? */
            }
        }
    }

    I2C_1_MasterSendStop(); /* Send Stop */

    return status;
    // return 0; /* Cheating */
}

//-----------------------------------------------------------------------------------------------------------

int8_t i2c_write(uint8_t addr, const uint8_t* data, uint16_t count, int appendCrc)
//Basic I2C write function, sends write header and then commands
{
    int status;
    int i;
    
    status = I2C_1_MasterSendStart(addr, I2C_1_WRITE_XFER_MODE); //begin transmission
    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
    {
 /* Send array of 2 bytes (commands) */
        for(i=0; i<count; i++) {
        status = I2C_1_MasterWriteByte(data[i]);
        
        if(status != I2C_1_MSTR_NO_ERROR) //If there is an error
            {
            break;
            }
        }
        if (appendCrc) { //if you are appending crc to end of command
            uint8_t crc = crc8(data, count); //make crc
            status = I2C_1_MasterWriteByte(crc);//write crc
            
      }
    }
 I2C_1_MasterSendStop(); /* Send Stop */    

return status;
}

//------------------------------------------------------------------------------------------    

uint8_t crc8(const uint8_t data[], uint8_t len)
//Generates check sum based on data received (compare to check sum sent with data)
{
  // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}
