#include "debug.h"
#include "usart.h"

/**
  * @brief  使用VOFA上位机, Just Float协议, 
  *         一个float为4个字节，即float = 4 x uint8_t
  */
uint8_t vofa_data[4*8] = 
{   0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
	  0,0,0,0,
    0x00,0x00,0x80,0x7F     // 四个结束帧
};
struct Debug_Param  Rx={0};


//void VOFA_Init(void)
//{
//    HAL_UART_Receive_IT(&huart3, (uint8_t*)&(Rx.tmp_data), 1);
//    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)vofa_data, sizeof(vofa_data));
//}


//void Buf_delete(uint8_t* buf, uint16_t len)
//{
//    for(uint16_t i=0; i<len; i++)
//        buf[i] = 0;
//}


///**
//  * @brief  串口数据解析
//  */
//void VOFA_DataHandler(void)
//{
//    uint8_t tmp_buf[3]={0};
//    tmp_buf[0] = Rx.buffer[0];
//    tmp_buf[1] = Rx.buffer[1];
//    
//    if(strcmp((char*)(tmp_buf), "S1") == 0)         // 设置电机启停
//    {
//        // Rx.buffer = "S1=Stop" 所以从第三个开始解析
//        char *p = (char*)(Rx.buffer+3);
//        if(strcmp((char*)(p), "Stop") == 0)
//        {
//            Power_Set_Stete(power_Stop);
//            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
//        }
//        if(strcmp((char*)(p), "Start") == 0)
//        {
//            Power_Set_Stete(power_Start);
//            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
//        }
//    }
////    else if(strcmp((char*)(tmp_buf), "S2") == 0)    // 设置电机运动模式
////    {
////        // Rx.buffer = "S2=Speed" 所以从第三个开始解析
////        char *p = (char*)(Rx.buffer+3);
////        if(strcmp((char*)(p), "Location") == 0)
////        {
////            BLDC_Set_Mode(Location_Mode);
////            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////        }
////        else if(strcmp((char*)(p), "Speed") == 0)
////        {
////            BLDC_Set_Mode(Velocity_Mode);
////            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////        }
////        else if(strcmp((char*)(p), "Current") == 0)
////        {
////            BLDC_Set_Mode(Current_Mode);
////            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////        }
////        else if(strcmp((char*)(p), "Voice") == 0)
////        {
////            BLDC_Set_Mode(Music_Mode);
//////            M0_Voice.state = Voice_Start;
////            Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////        }
////    }
//    
//    
//    /**** 获取目标值 ****/
//    if(strcmp((char*)(tmp_buf), "P1") == 0)
//    {
//        char *p = (char*)(Rx.buffer+3);
//        Power_Set_TargetId(atof(p));    // 得到目标电流
//        Buf_delete((uint8_t*)(p), Debug_BUF_Len);
//    }
////    else if(strcmp((char*)(tmp_buf), "P2") == 0)
////    {
////        char *p = (char*)(Rx.buffer+3);
////        BLDC_Set_RPMTarget(atof(p));    // 得到目标速度
////        Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////    }
////    else if(strcmp((char*)(tmp_buf), "P3") == 0)
////    {
////        char *p = (char*)(Rx.buffer+3);
////        BLDC_Set_RadTarget(atof(p));    // 得到目标位置
////        Buf_delete((uint8_t*)(p), Debug_BUF_Len);
////    }
//}


///**
//  * @brief  串口接收中断
//  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == huart2.Instance)
//    {
//        HAL_UART_Receive_IT(&huart2, (uint8_t*)&(Rx.tmp_data), 1);
//        
//        if(Rx.tmp_data == 0x23)         // 帧头 '#'
//        {
//            Rx.state = rx_Start;
//        }
//        else if(Rx.state == rx_Start)
//        {
//            if(Rx.tmp_data == 0x21)     // 帧尾 '!'
//            {
//                Rx.state = rx_Stop;
//            }
//            else
//            {
//                if(Rx.index == Debug_BUF_Len)     // 数据越线
//                {
//                    Rx.index = 0;
//                    Rx.state = rx_Stop;
//                    Buf_delete((uint8_t*)(Rx.buffer), Debug_BUF_Len);
//                } 
//                else                        // 接收数据
//                {
//                    Rx.buffer[Rx.index++] = Rx.tmp_data;
//                }
//            }
//            
//            if(Rx.state == rx_Stop)
//            {
//                Rx.flag = 1;
//                Rx.index = 0;
//                Rx.state = rx_Free;
//            }
//            
//            /**** 数据处理 ****/
//            if(Rx.flag == 1) 
//            {
//                VOFA_DataHandler();
//                
//                Rx.flag = 0;
//            }
//        }
//    }
//}

//        memcpy(vofa_data, (uint8_t*)&iiiiii, sizeof(iiiiii));
//        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)vofa_data, sizeof(vofa_data));

/**
  * @brief  printf的重定向
  */
int fputc(int c, FILE* f)
{
    uint8_t ch[1]={c};
    HAL_UART_Transmit(&huart3, ch, 1, 0xFFFF);
    return c;
}
