/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#define OLED_WIDTH     128
#define OLED_HEIGHT    64

#define DC_PIN         GPIO_PIN_10
#define DC_PORT        GPIOB

#define RST_PIN        GPIO_PIN_2
#define RST_PORT       GPIOB

#define CS_PIN         GPIO_PIN_12
#define CS_PORT        GPIOB

#define UP_BTN_PIN        GPIO_PIN_0
#define UP_BTN_PORT       GPIOA

#define DOWN_BTN_PIN         GPIO_PIN_1
#define DOWN_BTN_PORT        GPIOA

#define LEFT_BTN_PIN       GPIO_PIN_5
#define LEFT_BTN_PORT      GPIOA

#define RIGHT_BTN_PIN       GPIO_PIN_6
#define RIGHT_BTN_PORT      GPIOA

#define SET_BTN_PIN      GPIO_PIN_4
#define SET_BTN_PORT     GPIOA

uint8_t OLED_Buffer[1024];

/* MENU VARIABLES */
typedef enum
{
    MENU_TEMP = 0,
    MENU_TIME,
    MENU_START
} MenuItem_t;
uint8_t waitSetRelease = 0;
typedef enum
{
    MODE_NONE = 0,
    MODE_EDIT_TEMP,
    MODE_EDIT_TIME
} EditMode_t;

MenuItem_t selectedMenu = MENU_TEMP;
EditMode_t editMode = MODE_NONE;

uint8_t startFlag = 0;

uint8_t timeIndex = 2;

const uint8_t timeTable[3] =
{
    5,
    10,
    15
};

uint32_t setPressStart = 0;
uint8_t setHeld = 0;

typedef enum
{
    SCREEN_MAIN,
    SCREEN_TEMP,
    SCREEN_INCUBATE
} Screen_t;

Screen_t currentScreen = SCREEN_MAIN;

uint8_t menuIndex = 0;

float currentTemp = 37.0;

uint16_t incubateTime = 15;
uint32_t Counter = 0;
#define INCU_HEATER 1
#define PKT_SIZE 13
volatile uint8_t adc_ready_flag = 0;
#define SAMPLE_COUNT 20
#define R1 10000.0f
#define MIN_SWITCH_INTERVAL 3000  // 3 seconds minimum between ON/OFF switching
float Temperature = 0.0f;
#define MSG_SIZE   16
#define REPLY_SIZE 64
uint8_t current_incu_state = 0;
uint8_t current_read_state = 0;
volatile uint8_t waitForReleaseAfterPowerOn = 0;
const float c1=1.009249522e-03f;
const float c2=2.378405444e-04f;
const float c3=2.019202697e-07f;
#define HEATER_PWM_MAX 255
#define TEMP_TARGET 37.0f
#define PWM_START_LIMIT 128
extern volatile uint8_t usbTxReady;
volatile uint8_t adcDataReady = 0;
uint8_t rxBuffer[MSG_SIZE];
uint8_t replyBuffer[REPLY_SIZE];
char oledBuf[32];

#define R1 10000.0f

#define TEMP_SET       37.0f    // °C target
#define HYST           0.2f     // hysteresis band
#define TEMP_OVR       42.0f    // safety cutoff
#define TEMP_UNDER     5.0f     // sensor failure detection
// Added for hardware debugging
float integral_incu = 0, integral_read = 0;
float prev_error_incu = 0, prev_error_read = 0;
uint16_t adc_buffer[1];
uint16_t adc_samples[2][SAMPLE_COUNT];
uint8_t sample_index=0;
float I_Tc=25.0f, R_Tc=25.0f;
float filtered_I_Tc=25.0f, filtered_R_Tc=25.0f;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
SPI_HandleTypeDef hspi2;
/* Buffers */
uint8_t  tx_buf[] = "Welcome to STM32F401RE.\r\n";
uint8_t  rx_ch;                       // single char buffer
char     rx_buffer[100];              // line buffer
uint8_t  rx_index = 0;
uint8_t  ack_buf[150];                // reply buffer
float targetTemp = 32.0f;
uint8_t setLongPressDone = 0;
uint8_t setShortPressArmed = 0;
uint8_t systemReady = 0;
uint8_t setReleasedAfterBoot = 0;
uint32_t setBtnPressTick = 0;
uint32_t countdownSeconds = 0;
uint32_t lastCountdownTick = 0;
uint8_t blinkState = 1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
void ReadTemperature(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);

void OLED_DrawPixel(uint8_t x,
                    uint8_t y);

void OLED_DrawBigDigit(uint8_t x,
                       uint8_t y,
                       uint8_t digit);

void OLED_ShowCounter(uint32_t count);

void OLED_DrawRect(uint8_t x,
                   uint8_t y,
                   uint8_t w,
                   uint8_t h);

void OLED_DrawChar(uint8_t x,
                   uint8_t y,
                   char ch);

void OLED_DrawString(uint8_t x,
                     uint8_t y,
                     char *str);


void HandleButtons(void)
{
	static uint8_t lastSet   = 1;
	static uint8_t lastRight = 1;
	static uint8_t lastLeft  = 1;
	static uint8_t lastUp    = 1;
	static uint8_t lastDown  = 1;

    uint8_t set   = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
    uint8_t right = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
    uint8_t left  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
    uint8_t up    = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
    uint8_t down  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

    /* ---------- RIGHT ---------- */

    if(right == 0 && lastRight == 1)
    {
        if(editMode == MODE_NONE)
        {
            selectedMenu++;

            if(selectedMenu > MENU_START)
                selectedMenu = MENU_TEMP;
        }
    }

    /* ---------- LEFT ---------- */

    if(left == 0 && lastLeft == 1)
    {
        if(editMode == MODE_NONE)
        {
            if(selectedMenu == MENU_TEMP)
                selectedMenu = MENU_START;
            else
                selectedMenu--;
        }
    }

    /* ---------- SET PRESS ---------- */

    if(set == 0 && lastSet == 1)
    {
        setPressStart = HAL_GetTick();
    }

    /* ---------- LONG PRESS ---------- */

    if(set == 0 &&
       (HAL_GetTick() - setPressStart > 2000) &&
       !setLongPressDone)
    {
        setLongPressDone = 1;

        if(editMode == MODE_NONE)
        {
            switch(selectedMenu)
            {
            case MENU_TEMP:
                editMode = MODE_EDIT_TEMP;
                break;

            case MENU_TIME:
                editMode = MODE_EDIT_TIME;
                break;
            }
        }
    }

    /* ---------- SET RELEASE ---------- */

    if(set == 1 && lastSet == 0)
    {
        /* If editing -> SAVE immediately */

        if(editMode != MODE_NONE)
        {
            editMode = MODE_NONE;
            setLongPressDone = 0;
        }

        else if(selectedMenu == MENU_START)
        {
            startFlag = !startFlag;

            if(startFlag)
            {
                if(countdownSeconds == 0)
                {
                    countdownSeconds =
                        incubateTime * 60;
                }

                lastCountdownTick =
                    HAL_GetTick();
            }
        }

        else
        {
            setLongPressDone = 0;
        }
    }

    /* ---------- TEMP EDIT ---------- */

    if(editMode == MODE_EDIT_TEMP)
    {
    	if(up == 0 && lastUp == 1)
    	{
    	    targetTemp += 0.1f;
    	}

        if(down == 0 && lastDown == 1)
        {
            targetTemp -= 0.1f;
        }
    }

    /* ---------- TIME EDIT ---------- */

    if(editMode == MODE_EDIT_TIME)
    {
    	if(up == 0 && lastUp == 1)
    	{
    	    timeIndex++;

    	    if(timeIndex > 2)
    	        timeIndex = 0;

    	    incubateTime = timeTable[timeIndex];

    	    if(!startFlag)
    	    {
    	        countdownSeconds = incubateTime * 60;
    	    }
    	}

    	if(down == 0 && lastDown == 1)
    	{
    	    if(timeIndex == 0)
    	        timeIndex = 2;
    	    else
    	        timeIndex--;

    	    incubateTime = timeTable[timeIndex];

    	    if(!startFlag)
    	    {
    	        countdownSeconds = incubateTime * 60;
    	    }
    	}
    }

    /* ---------- START SCREEN ---------- */


    lastSet   = set;
    lastRight = right;
    lastLeft  = left;
    lastUp    = up;
    lastDown  = down;
}

void HandleCountdown(void)
{
    if(!startFlag)
        return;

    if(countdownSeconds == 0)
    {
        startFlag = 0;
        return;
    }

    if(HAL_GetTick() - lastCountdownTick >= 1000)
    {
        lastCountdownTick += 1000;

        countdownSeconds--;
    }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  HAL_Delay(100);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 1);
  adc_ready_flag =1;
  uint32_t lastTempTick = HAL_GetTick();
  uint32_t startTick    = lastTempTick;
    HAL_GPIO_WritePin(CS_PORT,
                      CS_PIN,
                      GPIO_PIN_SET);

    OLED_Init();

    if(countdownSeconds == 0)
    {
        countdownSeconds = incubateTime * 60;
    }

    HAL_Delay(2000);

    systemReady = 1;

    /* ================= MAIN LOOP ================= */

    while (1)
    {
        ReadTemperatures();

        HandleButtons();

        HandleCountdown();

        OLED_DrawMainScreen();

        HAL_Delay(20);
    }
}

void OLED_DrawMainScreen(void)
{
    char buf[20];

    OLED_Clear();

    /* ================= OUTER BORDER ================= */

    OLED_DrawRect(0,0,127,63);

    /* ================= DIVIDERS ================= */

    OLED_FillRect(63,0,1,48);      // vertical divider only for top 2 rows

    OLED_FillRect(0,24,128,1);     // top divider
    OLED_FillRect(0,48,128,1);     // bottom divider

    /* =================================================
       TOP LEFT : CURRENT TEMP
       ================================================= */

    OLED_DrawString(8,3,"CUR TEMP");

    int temp = (int)(I_Tc * 10);

    sprintf(buf,"%d.%01d",
            temp/10,
            abs(temp%10));

    OLED_DrawString(12,12,buf);
    OLED_DrawString(42,12,"^C");

    /* =================================================
       TOP RIGHT : INCUBATION
       ================================================= */

    OLED_DrawString(72,3,"INCUBATION");

    uint16_t minutes = countdownSeconds / 60;
    uint16_t seconds = countdownSeconds % 60;

    sprintf(buf,"%02u:%02u",minutes,seconds);

    OLED_DrawString(78,12,buf);

    /* =================================================
       BOTTOM LEFT : SET TEMP
       ================================================= */

    if(selectedMenu == MENU_TEMP)
    {
        OLED_DrawString(2,28,">");
    }

    OLED_DrawString(10,28,"SET TEMP");

    int setTemp = (int)(targetTemp * 10);

    sprintf(buf,"%d.%01d",
            setTemp/10,
            abs(setTemp%10));

    if(editMode == MODE_EDIT_TEMP)
    {
        if((HAL_GetTick()/300)%2)
        {
            OLED_DrawString(12,38,buf);
            OLED_DrawString(42,38,"^C");
        }
    }
    else
    {
        OLED_DrawString(12,38,buf);
        OLED_DrawString(42,38,"^C");
    }

    /* =================================================
       BOTTOM RIGHT : SET TIME
       ================================================= */

    if(selectedMenu == MENU_TIME)
    {
        OLED_DrawString(66,28,">");
    }

    OLED_DrawString(74,28,"SET TIME");

    sprintf(buf,"%02u MIN",incubateTime);

    if(editMode == MODE_EDIT_TIME)
    {
        if((HAL_GetTick()/300)%2)
        {
            OLED_DrawString(74,38,buf);
        }
    }
    else
    {
        OLED_DrawString(74,38,buf);
    }

    /* =================================================
       START BAR
       ================================================= */

    if(selectedMenu == MENU_START)
    {
        OLED_DrawString(28,54,">");
    }

    if(startFlag)
    {
        OLED_DrawString(52,54,"STOP");
    }
    else
    {
        OLED_DrawString(46,54,"START");
    }

    OLED_Update();
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        adcDataReady = 1;  // optional flag
    }
}

void SendTemperatureData(void)
{
	int temp_inc = (int)(I_Tc * 10);

	static char msg[256];
	snprintf(msg, sizeof(msg),
	    "Func Reader: %d.%d C | GPIOB3=%d \r\n",
	    temp_inc / 10, temp_inc % 10,
	    HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5));

	CDC_SendDebug(msg);
}

void ReadTemperatures(void)
 {
 	 // Store current ADC readings
 	    adc_samples[0][sample_index] = adc_buffer[0];  // Incubator
 	    sample_index = (sample_index + 1) % SAMPLE_COUNT;

 	   // printf("ADC Raw: Incu=%u  Reader=%u\r\n", adc_buffer[0], adc_buffer[1]);

 	    // Average samples
 	    uint32_t sum_incu = 0, sum_read = 0;
 	    for (int i = 0; i < SAMPLE_COUNT; i++) {
 	        sum_incu += adc_samples[0][i];
 	    }

 	    uint16_t Vo_incu = sum_incu / SAMPLE_COUNT;

 	    if (Vo_incu == 0) Vo_incu = 1;

 	    // Convert ADC to resistance
 	//   float R2_incu = R1 * ((float)Vo_incu /(4095.0f - (float)Vo_incu));
 	    float R2_incu = R1 * (4095.0f / (float)Vo_incu - 1.0f);

 	    // Thermistor formula to Celsius
 	    if (R2_incu >= 500 && R2_incu <= 50000) {
 	        float logR2_incu = log(R2_incu);
 	        float IT = 1.0f / (c1 + c2 * logR2_incu + c3 * pow(logR2_incu, 3));
 	        filtered_I_Tc = IT - 273.15f;
 	    }
 	   static float smoothTemp = 25.0f;

 	   /* Low pass filter */

 	   smoothTemp =
 	       (smoothTemp * 0.90f) +
 	       (filtered_I_Tc * 0.10f);

 	   I_Tc = smoothTemp;

 	  // I_Tc = filtered_I_Tc;
 }

/* ================= OLED LOW LEVEL ================= */

void OLED_Select()
{
    HAL_GPIO_WritePin(CS_PORT,
                      CS_PIN,
                      GPIO_PIN_RESET);
}

void OLED_Unselect()
{
    HAL_GPIO_WritePin(CS_PORT,
                      CS_PIN,
                      GPIO_PIN_SET);
}

void OLED_Command(uint8_t cmd)
{
    HAL_GPIO_WritePin(DC_PORT,
                      DC_PIN,
                      GPIO_PIN_RESET);

    OLED_Select();

    HAL_SPI_Transmit(&hspi2,
                     &cmd,
                     1,
                     HAL_MAX_DELAY);

    OLED_Unselect();
}

void OLED_Data(uint8_t *data,
               uint16_t size)
{
    HAL_GPIO_WritePin(DC_PORT,
                      DC_PIN,
                      GPIO_PIN_SET);

    OLED_Select();

    HAL_SPI_Transmit(&hspi2,
                     data,
                     size,
                     HAL_MAX_DELAY);

    OLED_Unselect();
}

/* ================= OLED ================= */

void OLED_Reset()
{
    HAL_GPIO_WritePin(RST_PORT,
                      RST_PIN,
                      GPIO_PIN_RESET);

    HAL_Delay(50);

    HAL_GPIO_WritePin(RST_PORT,
                      RST_PIN,
                      GPIO_PIN_SET);

    HAL_Delay(50);
}

void OLED_Clear()
{
    memset(OLED_Buffer,
           0x00,
           sizeof(OLED_Buffer));
}

void OLED_Update()
{
    for(uint8_t page = 0; page < 8; page++)
    {
        OLED_Command(0xB0 + page);

        OLED_Command(0x00);

        OLED_Command(0x10);

        OLED_Data(&OLED_Buffer[OLED_WIDTH * page],
                  OLED_WIDTH);
    }
}

void OLED_Init()
{
    OLED_Reset();

    OLED_Command(0xAE);

    OLED_Command(0x20);
    OLED_Command(0x00);

    OLED_Command(0xB0);

    OLED_Command(0xC8);

    OLED_Command(0x00);

    OLED_Command(0x10);

    OLED_Command(0x40);

    OLED_Command(0x81);
    OLED_Command(0x7F);

    OLED_Command(0xA1);

    OLED_Command(0xA6);

    OLED_Command(0xA8);
    OLED_Command(0x3F);

    OLED_Command(0xA4);

    OLED_Command(0xD3);
    OLED_Command(0x00);

    OLED_Command(0xD5);
    OLED_Command(0x80);

    OLED_Command(0xD9);
    OLED_Command(0xF1);

    OLED_Command(0xDA);
    OLED_Command(0x12);

    OLED_Command(0xDB);
    OLED_Command(0x40);

    OLED_Command(0x8D);
    OLED_Command(0x14);

    OLED_Command(0xAF);

    OLED_Clear();

    OLED_Update();
}

/* ================= DRAW ================= */

void OLED_DrawPixel(uint8_t x,
                    uint8_t y)
{
    if(x >= OLED_WIDTH || y >= OLED_HEIGHT)
        return;

    OLED_Buffer[x + (y / 8) * OLED_WIDTH]
            |= (1 << (y % 8));
}

void OLED_DrawRect(uint8_t x,
                   uint8_t y,
                   uint8_t w,
                   uint8_t h)
{
    for(uint8_t i=x;i<x+w;i++)
    {
        OLED_DrawPixel(i,y);
        OLED_DrawPixel(i,y+h);
    }

    for(uint8_t i=y;i<y+h;i++)
    {
        OLED_DrawPixel(x,i);
        OLED_DrawPixel(x+w,i);
    }
}

void OLED_FillRect(uint8_t x,
                   uint8_t y,
                   uint8_t w,
                   uint8_t h)
{
    for(uint8_t i = x; i < (x + w); i++)
    {
        for(uint8_t j = y; j < (y + h); j++)
        {
            OLED_DrawPixel(i, j);
        }
    }
}

const uint8_t font5x7[][5] =
{
    /* SPACE */
    {0x00,0x00,0x00,0x00,0x00},

    /* A-Z */
    {0x7E,0x11,0x11,0x11,0x7E}, //A
    {0x7F,0x49,0x49,0x49,0x36}, //B
    {0x3E,0x41,0x41,0x41,0x22}, //C
    {0x7F,0x41,0x41,0x22,0x1C}, //D
    {0x7F,0x49,0x49,0x49,0x41}, //E
    {0x7F,0x09,0x09,0x09,0x01}, //F
    {0x3E,0x41,0x49,0x49,0x7A}, //G
    {0x7F,0x08,0x08,0x08,0x7F}, //H
    {0x00,0x41,0x7F,0x41,0x00}, //I
    {0x20,0x40,0x41,0x3F,0x01}, //J
    {0x7F,0x08,0x14,0x22,0x41}, //K
    {0x7F,0x40,0x40,0x40,0x40}, //L
    {0x7F,0x02,0x04,0x02,0x7F}, //M
    {0x7F,0x04,0x08,0x10,0x7F}, //N
    {0x3E,0x41,0x41,0x41,0x3E}, //O
    {0x7F,0x09,0x09,0x09,0x06}, //P
    {0x3E,0x41,0x51,0x21,0x5E}, //Q
    {0x7F,0x09,0x19,0x29,0x46}, //R
    {0x46,0x49,0x49,0x49,0x31}, //S
    {0x01,0x01,0x7F,0x01,0x01}, //T
    {0x3F,0x40,0x40,0x40,0x3F}, //U
    {0x1F,0x20,0x40,0x20,0x1F}, //V
    {0x7F,0x20,0x18,0x20,0x7F}, //W
    {0x63,0x14,0x08,0x14,0x63}, //X
    {0x03,0x04,0x78,0x04,0x03}, //Y
    {0x61,0x51,0x49,0x45,0x43}  //Z
};

void OLED_DrawChar(uint8_t x,
                   uint8_t y,
                   char ch)
{
    const uint8_t digits[10][5] =
    {
        {0x3E,0x51,0x49,0x45,0x3E}, //0
        {0x00,0x42,0x7F,0x40,0x00}, //1
        {0x42,0x61,0x51,0x49,0x46}, //2
        {0x21,0x41,0x45,0x4B,0x31}, //3
        {0x18,0x14,0x12,0x7F,0x10}, //4
        {0x27,0x45,0x45,0x45,0x39}, //5
        {0x3C,0x4A,0x49,0x49,0x30}, //6
        {0x01,0x71,0x09,0x05,0x03}, //7
        {0x36,0x49,0x49,0x49,0x36}, //8
        {0x06,0x49,0x49,0x29,0x1E}  //9
    };

    uint8_t index;

    /* SPACE */

    if(ch == ' ')
    {
        return;
    }

    /* A-Z */
    /* DOT . */

    else if(ch == '.')
    {
        OLED_DrawPixel(x + 2, y + 6);
    }

    /* DEGREE SYMBOL */

    else if(ch == '^')
    {
        OLED_DrawPixel(x + 1, y + 1);
        OLED_DrawPixel(x + 2, y + 1);
        OLED_DrawPixel(x + 3, y + 1);

        OLED_DrawPixel(x + 1, y + 2);
        OLED_DrawPixel(x + 3, y + 2);

        OLED_DrawPixel(x + 1, y + 3);
        OLED_DrawPixel(x + 2, y + 3);
        OLED_DrawPixel(x + 3, y + 3);
    }

    /* COLON : */

    else if(ch == ':')
    {
        OLED_DrawPixel(x + 2, y + 2);
        OLED_DrawPixel(x + 2, y + 5);
    }

    else if(ch >= 'A' && ch <= 'Z')
    {
        index = ch - 'A' + 1;

        for(uint8_t col = 0; col < 5; col++)
        {
            uint8_t line = font5x7[index][col];

            for(uint8_t row = 0; row < 7; row++)
            {
                if(line & (1 << row))
                {
                    OLED_DrawPixel(x + col,
                                    y + row);
                }
            }
        }
    }

    /* 0-9 */

    else if(ch >= '0' && ch <= '9')
    {
        uint8_t num = ch - '0';

        for(uint8_t col = 0; col < 5; col++)
        {
            uint8_t line = digits[num][col];

            for(uint8_t row = 0; row < 7; row++)
            {
                if(line & (1 << row))
                {
                    OLED_DrawPixel(x + col,
                                    y + row);
                }
            }
        }
    }
}

void OLED_DrawString(uint8_t x,
                     uint8_t y,
                     char *str)
{
    while(*str)
    {
        OLED_DrawChar(x,y,*str++);
        x += 6;
    }
}


void OLED_DrawBigDigit(uint8_t x,
                       uint8_t y,
                       uint8_t digit)
{
    uint8_t seg[10][7] =
    {
        {1,1,1,1,1,1,0}, //0
        {0,1,1,0,0,0,0}, //1
        {1,1,0,1,1,0,1}, //2
        {1,1,1,1,0,0,1}, //3
        {0,1,1,0,0,1,1}, //4
        {1,0,1,1,0,1,1}, //5
        {1,0,1,1,1,1,1}, //6
        {1,1,1,0,0,0,0}, //7
        {1,1,1,1,1,1,1}, //8
        {1,1,1,1,0,1,1}  //9
    };

    uint8_t w = 24;
    uint8_t h = 40;
    uint8_t t = 5;

    /* TOP */
    if(seg[digit][0])
    {
        OLED_FillRect(x + t,
                      y,
                      w,
                      t);
    }

    /* TOP RIGHT */
    if(seg[digit][1])
    {
        OLED_FillRect(x + w,
                      y + t,
                      t,
                      h/2 - t);
    }

    /* BOTTOM RIGHT */
    if(seg[digit][2])
    {
        OLED_FillRect(x + w,
                      y + h/2,
                      t,
                      h/2 - t);
    }

    /* BOTTOM */
    if(seg[digit][3])
    {
        OLED_FillRect(x + t,
                      y + h - t,
                      w,
                      t);
    }

    /* BOTTOM LEFT */
    if(seg[digit][4])
    {
        OLED_FillRect(x,
                      y + h/2,
                      t,
                      h/2 - t);
    }

    /* TOP LEFT */
    if(seg[digit][5])
    {
        OLED_FillRect(x,
                      y + t,
                      t,
                      h/2 - t);
    }

    /* CENTER */
    if(seg[digit][6])
    {
        OLED_FillRect(x + t,
                      y + (h/2) - (t/2),
                      w,
                      t);
    }
}

void OLED_ShowCounter(uint32_t count)
{
    OLED_Clear();

    /* BORDER */

    OLED_DrawRect(0,0,127,63);

    /* LIMIT 0-9 */

    count %= 10;

    /* CENTER BIG DIGIT */

    OLED_DrawBigDigit(48,12,count);

    OLED_Update();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // PC5 -> ADC_CHANNEL_15 (Reader)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
