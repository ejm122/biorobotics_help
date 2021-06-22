/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "stdio.h"
#include <sdpsensor.h>
#include <i2chelper.c>

//sfmsensor
#define sfm3300i2c 0x40

//ADC
//void print_adc_via_usbuart(void);
//int16 adc_val_int16 = 0;
//float32 adc_value = 0;
//float flow = 0;

// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
void USBUART_user_echo(void);

// Testing Function
void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num);

// System clock
uint32 sys_clock_cur_ms = 0;
float sys_clock_cur_us_in_ms = 0;
float temp;
float pressure;
void sys_clock_ms_callback(void); // 1ms callback interrupt function

//crc function
uint8_t SMF3000_CheckCrc (uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum); 

//for crc
typedef enum{
CHECKSUM_ERROR = 0x04
}etError;
typedef unsigned char u8t;

int main(void)
{
    uint8_t led_test = 0;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // USBUART Init
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    
    // I2C Init
    I2C_1_Start();
    
    /* Start the ADC components */
    //ADC_DelSig_1_Start();
    
    /* Start the ADC conversion */
    //ADC_DelSig_1_StartConvert();
    
    //Initialize for flow sensor
      //int ret = init(); //if return 0, then successful initialization
      //while (ret != 0) {
            //CyDelay(500);
            //init();
    //}
    
    // PWM Block Init
    PWM_LED_Start();
   
    /* Turn off LEDs */
    Led_Red_Write(0);
    Led_Green_Write(0);
    Led_Blue_Write(0);
    Led_Key_Write(1);
    
    CyDelay(100);
    
    // Start system 1ms tick
    CySysTickStart();
    CySysTickSetCallback(0, sys_clock_ms_callback);
    CySysTickEnableInterrupt();
    
//    Wire.beginTransmission(sfm3300i2c);
//    Wire.write(0x10); // start continuous measurement
//    Wire.write(0x00); // command 0x1000
//    Wire.endTransmission();
    
    int a = 0;
    int b = 0;
//    int i;
//    int status;
//    uint16_t register_addr = 0x1000;
//    const uint8_t DATA_LEN = 3; //Sensor sends 9 bytes (pressure differential, CRC, temp, CRC, scale factor, CRC)
//    uint8_t data[3] = { 0 };
//    uint16_t count = 3;

    const uint8_t CMD_LEN = 2;
    uint8_t cmd[2] = { 0x10, 0x00 }; //continuous measurement)
//
    if(i2c_write(sfm3300i2c, cmd, CMD_LEN, 0) != 0) {//write command
        a = 1;
    }
    CyDelay(100);   
    
    int n = 0;
    for(;;)
    {
        
        n = n+1; //for tracing sensor readouts
//        
        
//        const uint8_t CMD_LEN = 2;
//        uint8_t cmd[2] = { 0x10, 0x00 }; //continuous measurement)
//
//        if(i2c_write(sfm3300i2c, cmd, CMD_LEN, 0) != 0) {//write command
//        a = 1;
//        }
//        
        CyDelay(50); // theoretically 45ms /////////
        const uint8_t DATA_LEN = 3; //Sensor sends 9 bytes (pressure differential, CRC, temp, CRC, scale factor, CRC)
        uint8_t data[3] = { 0 };
        if(i2c_read(sfm3300i2c, data, DATA_LEN) != 0) {
            b = 1;
        }
        
        
//        const uint8_t DATA_LEN = 3; //Sensor sends 9 bytes (pressure differential, CRC, temp, CRC, scale factor, CRC)
//        uint8_t data[3] = { 0 };
//        if(i2c_write_SFM3xxx(sfm3300i2c, 0x1000, data, DATA_LEN) != 0) {
//            a = 1;
//            b = 1;
//        }
        
//    status = I2C_1_MasterSendStart(sfm3300i2c, I2C_1_WRITE_XFER_MODE); //begin transmission in write-mode
//    if(I2C_1_MSTR_NO_ERROR == status) /* Check if transfer completed without errors */
//    {
//        /* Send register-address to write */
//        uint8_t reg_MSB = (register_addr >> 8) & 0xFF;
//        uint8_t reg_LSB = register_addr & 0xFF;
//        status = I2C_1_MasterWriteByte(reg_MSB);
//        status |= I2C_1_MasterWriteByte(reg_LSB);
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(n,5,1,1,1);
//    }
//    if(status == I2C_1_MSTR_NO_ERROR) //If there is no error
//    {
//        for (i = 1; i < 500; ++i)
//        {
//        USBUART_user_check_init();
//        print_sensor_via_usbuart(1,2,3,4,5);
//        status = I2C_1_MasterSendRestart(sfm3300i2c, I2C_1_READ_XFER_MODE); //continue transmission in read-mode
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
        
        uint8_t crc_check = SMF3000_CheckCrc(data, 2, data[2]);
        uint16_t flow_raw = (data[0] << 8) | data[1];
        float flow = ((float)flow_raw - 32768 / 120);
        USBUART_user_check_init();
        print_sensor_via_usbuart(n,flow,a,b,crc_check);
//        }
//    }
//    I2C_1_MasterSendStop(); /* Send Stop */    
        
//        uint16_t flow_raw = (data[0] << 8) | data[1];
//        float flow = ((float)flow_raw - 32768 / 120);
          //VolFlow = k*sqrt(delta_p) + bias
            //int ret = readSample(); //triggered measurement
            //pressure = getDifferentialPressure();
            //temp = getTemperature();
            //while (!(ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_RETURN_STATUS)));
            //adc_value = ADC_DelSig_1_CountsTo_Volts(ADC_DelSig_1_GetResult32());
            //flow = (adc_value-0.48)*25; //ADC at 0 flow settles at 0.45-0.48V, this equation was derived from plots in the datasheet
//            USBUART_user_check_init();
//            print_sensor_via_usbuart(n,flow,a,b,(float)data[2]);
            
            led_test++;
            
            Led_Red_Write((led_test >> 0) & 0x01);
            Led_Green_Write((led_test >> 1) & 0x01);
            Led_Blue_Write((led_test >> 2) & 0x01);   

            if (Led_Key_Read() == 0)
            {
                led_test = 0;
            }
              
            
            //CyDelay(10);  
    }
}

/// USBUART Routin
void USBUART_user_check_init(void) {
    /* Host can send double SET_INTERFACE request. */
    if (0u != USBUART_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            USBUART_CDC_Init();
        }
    }
}


void USBUART_user_echo(void) {
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != USBUART_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */
            count = USBUART_GetAll(buffer);

            if (0u != count)
            {
                /* Wait until component is ready to send data to host. */
                while (0u == USBUART_CDCIsReady())
                {
                }

                /* Send data back to host. */
                USBUART_PutData(buffer, count);

                /* If the last sent packet is exactly the maximum packet 
                *  size, it is followed by a zero-length packet to assure
                *  that the end of the segment is properly identified by 
                *  the terminal.
                */
                if (USBUART_BUFFER_SIZE == count)
                {
                    /* Wait until component is ready to send data to PC. */
                    while (0u == USBUART_CDCIsReady())
                    {
                    }

                    /* Send zero-length packet to PC. */
                    USBUART_PutData(NULL, 0u);
                }
            }
        }
    }
}

void print_sensor_via_usbuart(int n, float pressure, float temp, float32 flow, uint8_t crc_check_num)
{   
    sys_clock_cur_us_in_ms = (float)CySysTickGetValue() * (1/(float)cydelay_freq_hz);

    while (0u == USBUART_CDCIsReady());

    sprintf((char *)buffer, "%d. pressure: %f temperature: %f Flow: %f CRC(4): %d\n",n, pressure, temp, flow, crc_check_num);
   
    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    USBUART_PutString((char8 *)buffer);
}

// 1ms system tick callback interrupt function
void sys_clock_ms_callback(void){
    sys_clock_cur_ms ++; // increment ms counter by 1
}

//CRC
#define POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001
//============================================================
uint8_t SMF3000_CheckCrc (uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//============================================================
//calculates checksum for n bytes of data 
//and compares it with expected checksum
//input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
// checksum expected checksum
//return: error: CHECKSUM_ERROR = checksum does not match
// 0 = checksum matches
//============================================================
{
u8t crc = 0;
u8t byteCtr;
//calculates 8-Bit checksum with given polynomial
for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
{ crc ^= (data[byteCtr]);
 for (u8t bit = 8; bit > 0; --bit)
 { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
 else crc = (crc << 1);
 }
}
if (crc != checksum) return CHECKSUM_ERROR;
else return 0;
}

/* [] END OF FILE */
