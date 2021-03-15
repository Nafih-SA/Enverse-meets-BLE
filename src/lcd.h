#pragma once
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <inttypes.h>

#ifndef __LCD_h__
#define __LCD_h__

#ifdef DEBUG
#define END 0xffff
#endif

typedef unsigned char uint8;
typedef signed char int8;
typedef unsigned int uint16;
typedef signed int int16;
typedef unsigned long uint32;
typedef signed long int32;

//extern xdata unsigned char buffer[512];
#define ROW 448 //��ʾ���С�����
#define COL 368

#define TP_TEST //���Թ��ܵ�ѡ��ѡ���������壬��Ӧ���Թ��ܿ����� TP_TEST:TP���߲��Թ���
//#define READ_REG		    //��ID CODE����
//#define SLEEP_TEST	    //���롢�˳�sleep����
//#define DEBUG			    //���Թ��ܣ���Ҫ���Ե�ָ��Ͳ���д�������ڣ�ÿ����ִ��һ��ָ��Ͳ���

//#define EVAL_PIC		    //ʹ��MCUƬ��64k Flash()�ڵ�ͼƬ��160*160�����ʾ����
//#define EXTEND_TEST	    //��չ���Թ��ܣ��磬�Ҷȣ�ɫ�ʽ���Ȼ���

//------------------------------------------------------
//#define CHAR_FONT_W8_H16	//ѡ����ʾ�ַ������壬CHAR_FONT_W8_H16��8*16��CHAR_FONT_W16_H21��16*21
#define CHAR_FONT_W16_H21

#define CONN_USB    //�ڰ����ȴ�ʱ����USB��������ͨ�š��������Դ�ʵ��USB���Թ��ܣ�
#define USE_SD_INIT //SD����ʼ��
#define SD_DEMO_PIC //��ʾSD���ڵ�ͼƬ������������չʾ
//#define AUTO_TEST			//������AUTO-TEST���ȴ�����ʱ���ذ���������ʱһ�κ��Զ�������һ���Ĳ���
//------------------------------------------------------
#define DBH P2 //�ӿڶ��岿��
#define DBL P4

#ifdef READ_REG
#define STRING_FUNCTION
#define READ_FUNCTION
#endif

#ifdef DEBUG
#define STRING_FUNCTION
#endif
#ifdef CONN_USB
#define STRING_FUNCTION
#define READ_FUNCTION
#endif
#ifdef TP_TEST
#define DRAW_FUNCTION
#define STRING_FUNCTION
#endif

#ifdef AUTO_TEST
#define AUTO_NEXT
#else
#define MANUA_NEXT
#endif

//#define STRING_FUNCTION	  //�����ַ�����������ʾ�ĺ���
//#define DRAW_FUNCTION		  //����TP���Եĺ���
//#define READ_FUNCTION		  //�����ȡIC�����ĺ���
//------------------------------------------------------
#ifdef CHAR_FONT_W8_H16
#define FONT_W 8
#define FONT_H 16
#endif
#ifdef CHAR_FONT_W16_H21
#define FONT_W 16
#define FONT_H 21
#endif
//------------------------------------------------------
#define PIC_WIDTH 160 //Ԥ����LCD��ʾ��������ͼƬ�Ĵ�С
#define PIC_HEIGHT 160

#define RED 0xF800 //������ɫ����
#define GREEN 0x07E0
#define BLUE 0x001F
#define WHITE 0xFFFF
#define BLACK 0x0000
#define GRAY 0xEF5D //0x2410
#define GRAY75 0x39E7
#define GRAY50 0x7BEF
#define GRAY25 0xADB5

#define X_min 0x0043 //TP���Է�Χ��������
#define X_max 0x07AE
#define Y_min 0x00A1
#define Y_max 0x0759
//------------------------------------------------------
// void  Delay(unsigned int dly);
// void  WaitKey(void);
// void  StopDelay(unsigned int sdly);
// void  ConnToUSB(void);
// void  Suspend_Device(void);
// void  Devices_Init(void);
// void  MCU_Init(void);

void WriteComm(uint8_t data);
void WriteData(uint8_t data);
void LCD_Init(void);
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend);
void DispColor(unsigned int color);
void DispLogo(unsigned int color);
void DispIcon(bool pic[50][50], unsigned int color, int xstart, int ystart, int height, int width);
// void DispIconHeart(unsigned int color, int xstart, int ystart, int height, int width);
// void DispIconSteps(unsigned int color, int xstart, int ystart, int height, int width);
// void DispIconTemperature(unsigned int color, int xstart, int ystart, int height, int width);
void WriteOneDot(unsigned int color);
unsigned char ToOrd(unsigned char ch);
void DispOneChar(unsigned char ord, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor); // ord:0~95
void DispStr(unsigned char *str, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);
void DispPic(unsigned int *picture);
void SetBrightness(int value);
void turnOffAllPixels();
void turnOffDisplay();
void turnOnDisplay();
void turnOnNormalDisplay();
void resetDisplay();
void setDisplay();
//void WriteDispData(unsigned char DataH,unsigned char DataL);
// void DispBand(void);
// void DispFrame(void);

// void DispPicFromSD(unsigned char PicNum);

// void DispScaleHor1(void);
// void DispScaleVer(void);
// void DispScaleVer_Red(void);
// void DispScaleVer_Green(void);
// void DispScaleVer_Blue(void);
// void DispScaleVer_Gray(void);
// void DispGrayHor16(void);
// void DispGrayHor32(void);
// void DispScaleHor2(void);
// void DispSnow(void);
// void DispBlock(void);

// void DispInt(unsigned int i,unsigned int Xstart,unsigned int Ystart,unsigned int TextColor,unsigned int BackColor);

// unsigned int ReadData(void);
// void DispRegValue(unsigned int RegIndex,unsigned char ParNum);

// void Debug(void);

// void PutPixel(unsigned int x,unsigned int y,unsigned int color);
// void DrawLine(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend,unsigned int color);
// void DrawGird(unsigned int color);

#endif
