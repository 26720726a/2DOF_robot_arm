/*
    ============================================
      RobotExp4  —  AVR main.c Full Skeleton
      (MFC <-> AVR Packet + Cascade Control)
    ============================================
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"
#include "LS7366.h"
#include "pwm.h"
#include "adc.h"
#include "dataType.h"


// ===== Global variables =====
volatile double g_Pdes = 0.0;      // MFC → 목표 위치
volatile double g_Vlimit = 0.0;    // 목표 속도
volatile double g_Climit = 0.0;    // 목표 전류

volatile double cur_pos = 0.0;
volatile double cur_vel = 0.0;
volatile double cur_cur = 0.0;

// ===== Packet Buffers =====
volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode = 0;
volatile unsigned char g_ID = 1;
volatile unsigned char checkSize = 0;

// ===== UART Ring Buffer =====
volatile unsigned char g_buf[256];
volatile unsigned char g_bufWriteCnt = 0;
volatile unsigned char g_bufReadCnt = 0;


//===============================
//  UART0 RX INTERRUPT
//===============================
ISR(USART0_RX_vect)
{
    g_buf[g_bufWriteCnt++] = UDR0;
}



//===============================
//        TIMER0 (1kHz)
//===============================
ISR(TIMER0_OVF_vect)
{
    static unsigned char cnt = 0;
    cnt++;
}



/*****************************************************
                    Init Functions
******************************************************/
void InitIO()
{
    cli();

    // UART0 9600
    UART0_Init(9600);

    // LS7366 (SPI)
    LS7366_Init();

    // ADC0 (Current Sensor)
    ADC_Init();

    // PWM Motor Driver
    PWM_Init();

    // Timer0 → 1kHz (control loop)
    TCCR0 = (1 << CS02) | (1 << CS00);     // clk/1024
    TIMSK |= (1 << TOIE0);

    sei();
}



/*****************************************************
                   PACKET PARSER
******************************************************/
void ReceivePacket()
{
    if (g_bufReadCnt != g_bufWriteCnt)
    {
        unsigned char Rx = g_buf[g_bufReadCnt++];

        switch (g_PacketMode)
        {
        case 0:     // HEADER
            if (Rx == 0xFF)
            {
                g_PacketMode = 1;
                checkSize = 0;
            }
            break;

        case 1:
            g_PacketBuffer.buffer[checkSize++] = Rx;

            if (checkSize == 4)   // header4개 수신 완료
            {
                if (g_PacketBuffer.data.id == g_ID)
                    g_PacketMode = 2;
                else
                {
                    g_PacketMode = 0;
                    checkSize = 0;
                }
            }
            break;

        case 2:     // data
            g_PacketBuffer.buffer[checkSize++] = Rx;

            if (checkSize == g_PacketBuffer.data.size)
            {
                unsigned char check = 0;

                for (int i = 0; i < g_PacketBuffer.data.size - 1; i++)
                    check += g_PacketBuffer.buffer[i];

                if (check == g_PacketBuffer.data.check)
                {
                    // ===== MFC → AVR 전달 값 =====
                    g_Pdes   = g_PacketBuffer.data.pos / 1000.0;
                    g_Vlimit = g_PacketBuffer.data.vel / 1000.0;
                    g_Climit = g_PacketBuffer.data.cur / 1000.0;
                }

                g_PacketMode = 0;
                checkSize = 0;
            }
            break;
        }
    }
}



/*****************************************************
                    PACKET SEND
******************************************************/
void SendCurrentData()
{
    static unsigned char cnt = 0;
    cnt++;

    if (cnt > 20)
    {
        cnt = 0;

        Packet_t packet;

        // Header
        packet.data.header[0] =
        packet.data.header[1] =
        packet.data.header[2] =
        packet.data.header[3] = 0xFF;

        packet.data.id = g_ID;
        packet.data.size = sizeof(Packet_data_t);
        packet.data.mode = 3;

        // 현재값 (MFC 그래프 그리기용)
        packet.data.pos = cur_pos  * 1000.0;
        packet.data.vel = cur_vel  * 1000.0;
        packet.data.cur = cur_cur  * 1000.0;

        // checksum
        packet.data.check = 0;
        for (int i = 0; i < packet.data.size - 1; i++)
            packet.data.check += packet.buffer[i];

        // 전송
        for (int i = 0; i < packet.data.size; i++)
            TransUART(packet.buffer[i]);
    }
}



/*****************************************************
                MAIN CONTROL LOOP (1kHz)
******************************************************/
void ControlLoop()
{
    static double pos_prev = 0;

    // 1) Read sensors
    long enc = LS7366_ReadCounter();
    cur_pos = enc * (2 * 3.141592) / (4000.0);    // 예: CPT 1000 * 4 = 4000

    cur_vel = (cur_pos - pos_prev) * 1000.0;
    pos_prev = cur_pos;

    cur_cur = ADC_ReadCurrent();


    // ★★ 현재는 제어기 비활성화 상태 (값만 측정해서 보내는 버전)
    // 향후 위치/속도/전류 제어기 추가 가능


    // 2) Send to PC
    SendCurrentData();
}



/*****************************************************
                    MAIN FUNCTION
******************************************************/
int main(void)
{
    Packet_t packet;

    // 패킷 헤더 초기화
    packet.data.header[0] =
    packet.data.header[1] =
    packet.data.header[2] =
    packet.data.header[3] = 0xFF;

    InitIO();

    while (1)
    {
        ReceivePacket();     // PC → AVR
        ControlLoop();       // 센서 읽기 & PC로 송신
    }
}
