#include "adns3080.h"
#include "main.h"
#include <stdio.h>

#define ADNS3080_REG_WRITE 0x80

#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_CONFIGURATION_BITS 0x0A
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_MOTION_BURST 0x50

#define ADNS3080_BIT_MOTION 0x80
#define ADNS3080_BIT_1600IPS 0x10

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE 0x17
#define ADNS3080_FRAME_CAPTURE_START 0x83

uint8_t sbuf[16] = {0}, rbuf[16] = {0};
int8_t delta_x, delta_y;
uint8_t quality;
int32_t integral_x, integral_y;

void start_transmit(void)
{
    HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
}

void end_transmit(void)
{
    HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);
}


static void reset(void)
{
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 1, 1000);
    end_transmit();
    // set clk,nss polaryty


    HAL_Delay(1);

    HAL_GPIO_WritePin(MOUSE_RST_GPIO_Port, MOUSE_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MOUSE_RST_GPIO_Port, MOUSE_RST_Pin, GPIO_PIN_RESET);

    HAL_Delay(250); // waiting for self-test
}

bool is_connect_ADNS3080(void){
    HAL_GPIO_WritePin(LED_CURRENT_GPIO_Port, LED_CURRENT_Pin, GPIO_PIN_SET);

    reset();


    start_transmit();

    sbuf[0] = ADNS3080_PRODUCT_ID;
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
    
    end_transmit();

    printf("SPI ID : %d\n", rbuf[1]);

    if (rbuf[1] == ADNS3080_PRODUCT_ID_VALUE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void init_ADNS3080(bool ips_1600)
{
    delta_x = 0;
    delta_y = 0;
    quality = 0;
    integral_x = 0;
    integral_y = 0;
    reset();


    start_transmit();

    sbuf[0] = ADNS3080_REG_WRITE & ADNS3080_CONFIGURATION_BITS; // config write
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 1, 1000);

    //delay >75us
    for (int i = 0; i < 1000; i++)
    {
    }

    uint8_t mouse_config = rbuf[1];
    sbuf[0] = mouse_config | ADNS3080_BIT_1600IPS; // set 1600 ips
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 1, 1000);
    
    end_transmit();
}


void update_ADNS3080(void){
    
    start_transmit();

    sbuf[0] = ADNS3080_MOTION_BURST;
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 8, 1000);
    
    end_transmit();
    
    if (rbuf[1] & ADNS3080_BIT_MOTION)
    {
        delta_x = (int8_t)rbuf[2];
        delta_y = (int8_t)rbuf[3];
        integral_x += delta_x;
        integral_y += delta_y;
    }else{

    }
    quality = rbuf[4];
}


int8_t get_DeltaX_ADNS3080(void){
    return delta_x;
}
int8_t get_DeltaY_ADNS3080(void){
    return delta_y;
}
uint8_t get_Qualty_ADNS3080(void){
    return quality;
}

void clear_XY_ADNS3080(void){
    integral_x = 0;
    integral_y = 0;
}
int32_t get_X_ADNS3080(void){
    return integral_x;
}
int32_t get_Y_ADNS3080(void){
    return integral_y;
}

void frame_print_ADNS3080(void){
    char scale[] = "#987654321-,.'` ";
    sbuf[0] = ADNS3080_FRAME_CAPTURE | ADNS3080_REG_WRITE; // frame capture write
    sbuf[1] = ADNS3080_FRAME_CAPTURE_START;
    start_transmit();
    HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
    end_transmit();
    HAL_Delay(2);

    for (int pixel_x = 0; pixel_x < ADNS3080_PIXELS_X; pixel_x++)
    {
        for (int pixel_y = 0; pixel_y < ADNS3080_PIXELS_Y; pixel_y++)
        {
            sbuf[0] = ADNS3080_FRAME_CAPTURE; // frame capture
            start_transmit();
            HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
            end_transmit();

            printf("%c ", scale[(rbuf[1] % 0x3F) >> 2]);
        }
        printf("\n");
    }
    printf("\n\n");
}
