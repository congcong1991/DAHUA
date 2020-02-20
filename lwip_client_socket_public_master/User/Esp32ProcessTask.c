#include "main.h"
//#include "cmsis_os.h"
#include "bsp.h"
#include "app.h"
#include "bsp_esp32.h"

//#include "fftw3.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "sys.h"

//#include "fftw_mpi.h"
//#include "mpi.h"



#define PI 3.1415926


//void backFFT(double* realIn,double* imgIn,uint32_t length,double* dataOut);
//void backFFT_1(fftw_complex* in, uint32_t length, double *dataOut);


#define      macUser_Esp32_LocalID                        "192.168.0.19"                //连不上去时，备用的就是这个
#define      macUser_Esp32_LocalGATAWAY                   "192.168.0.1"           
#define      macUser_Esp32_LocalMASK                      "255.255.255.0"           

#define      macUser_Esp32_ApSsid                        "Tenda_5D9330"                //要连接的热点的名称
#define      macUser_Esp32_ApPwd                         ""           //要连接的热点的密钥

#define      macUser_Esp32_TcpServer_IP                   "192.168.0.99"//"192.168.0.112"// //     //要连接的服务器的 IP
#define      macUser_Esp32_TcpServer_Port                 "8712"  //"8712"//             //要连接的服务器的端口


/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
 extern void software_reset(void);
extern uint8_t receive_buff[IdleRxdBufSize];
extern UART_HandleTypeDef huart6;
extern volatile uint16_t esp32_ready;
extern volatile uint16_t WifiConnectedFlag;
extern volatile uint16_t WifiDisconnectedFlag;

extern void Udp_To_Pc(uint8_t *send_buf,uint32_t size);

void Esp32ProcessFunction( void *pvParameters )//用作一直发送udp数据
{ 

	for(;;)
	{
	
			if( !isTxdBufEmpty() )//!isTxdBufEmpty()
			{
			
//				SCB_InvalidateDCache_by_Addr ((uint32_t *)TXD_BUFFER_NET[TxdBufTailIndex], TXD_BUFFER_NET_Length[TxdBufTailIndex]*0.25f);
				
//				vTaskDelay(1);
				Udp_To_Pc(TXDBUF[TxdBufTailIndex],TXDBUFLength[TxdBufTailIndex]);
//				err_flag=netconn_write( tcp_client_server_conn, TXDBUF[TxdBufTailIndex], TXDBUFLength[TxdBufTailIndex], NETCONN_COPY);
//				tcp_delay_counter++;
//				if( err_flag==ERR_OK)
//				{
					Increase(TxdBufTailIndex);

//				}
//				else
//				{
//					DisCollectServer();  //链接断开标志
//					REGISTER_FLAG = 0x00;
//					vTaskDelay(1);
////					AD7606_StopRecord();
//				}

			}else
			{
				vTaskDelay(2);
			}
	}
	

}




//void backFFT_1(fftw_complex* in, uint32_t length, double *dataOut)
//{
//	uint32_t i = 0;
//    fftw_complex* out=(fftw_complex*)fftw_malloc(length*sizeof(fftw_complex));
//    fftw_plan q=fftw_plan_dft_1d(length,in,out,FFTW_BACKWARD,FFTW_ESTIMATE);
//    fftw_execute(q);
//    for(i=0;i<length;i++)
//    {
//        dataOut[i]=out[i][0] /length;
//    }
//    fftw_destroy_plan(q);
//    fftw_free(out);
//}

//void backFFT(double* realIn,double* imgIn,uint32_t length,double* dataOut)
//{
//	uint32_t i = 0;
//    fftw_complex* in=(fftw_complex*)fftw_malloc(length*sizeof(fftw_complex));
//    fftw_complex* out=(fftw_complex*)fftw_malloc(length*sizeof(fftw_complex));
//    fftw_plan q=fftw_plan_dft_1d(length,in,out,FFTW_BACKWARD,FFTW_ESTIMATE);
//    for(i=0;i<length;i++)
//    {
//        in[i][0]=realIn[i];
//        in[i][1]=imgIn[i];
//    }
//    fftw_execute(q);
//    for(i=0;i<length;i++)
//    {
//        dataOut[i]=out[i][0] /length;
//    }
//    fftw_destroy_plan(q);
//    fftw_free(in);
//    fftw_free(out);
//}

//uint32_t acceleration(uint32_t fs,double* data,uint32_t len,double* character)
//{
//	uint32_t outlen = 0;
//	        double* velocity;// = new double[outlen];
//        double* disp;// = new double[outlen];
//	           double *envelopWave;//=new double[len];
//	        double* real;//=new double[len];
//        double* img;//=new double[len];
//	
//	        double* halfband;//=new double[len];       //速度
//        double* realInt;//=new double[len];        //1次积分实部
//        double* imgInt;//=new double[len];         //1次积分虚部
//        double* halfband2;//=new double[len];      //位移
//        double* realInt2;//=new double[len];       //2次积分实部
//        double* imgInt2;//=new double[len];        //2次积分虚部
//        double* halfband3;//=new double[len];      //反fft后的加速度信号
//	
//	        double* wave;//=new double[len];
//	        fftw_complex* out;//=(fftw_complex*)fftw_malloc(len*sizeof(fftw_complex));
//        fftw_complex* tempForenv;//=(fftw_complex*)fftw_malloc(len*sizeof(fftw_complex));
//	
//				fftw_plan p;
//	
//	uint32_t i = 0;
//    if(len>0 && fs>0 )
//    {
//        double df=((double)fs/len);
//        double dw=2*PI*df;

//        int indexLow=4/df;
//        int indexHigh=fs/(2*df);
//        fftw_complex* wave1=(fftw_complex*)fftw_malloc((len*sizeof(fftw_complex)));
//        memcpy(wave,data,len*sizeof(double));
//			//这个陆工的函数
////        removeDC(wave,len);
//        out=(fftw_complex*)fftw_malloc(len*sizeof(fftw_complex));
//        tempForenv=(fftw_complex*)fftw_malloc(len*sizeof(fftw_complex));
//        p=fftw_plan_dft_1d(len,wave1,out,FFTW_FORWARD,FFTW_ESTIMATE);
//        for(i=0;i<len;i++)
//        {
//            wave1[i][0]=wave[i];
//            wave1[i][1]=0;
//        }
//        fftw_execute(p);


//        for(i=0;i<len/2;i++)
//        {
//            halfband[i]=(i)*dw;
//            halfband[len-1-i]=(i)*dw;
//            halfband2[i]=pow(i*dw,2);
//            halfband2[len-1-i]=pow(i*dw,2);
//        }
//        for(i=1;i<len/2;i++)
//        {
//            realInt[i]=out[i][1];
//            imgInt[i]=-out[i][0];
//            realInt[i]=realInt[i]/halfband[i];
//            imgInt[i]=imgInt[i]/halfband[i];
//            realInt2[i]=-out[i][0]/halfband2[i];
//            imgInt2[i]=-out[i][1]/halfband2[i];
//            if(i<indexLow || i>indexHigh )
//            {
//                realInt[i]=0;
//                imgInt[i]=0;
//                realInt2[i]=0;
//                imgInt2[i]=0;
//                out[i][0]=0;
//                out[i][1]=0;
//            }
//            realInt[len-1-i]=0;
//            imgInt[len-1-i]=0;
//            realInt2[len-1-i]=0;
//            imgInt2[len-1-i]=0;
//            out[len-1-i][0]=0;    //    temptempv[N-i]=-tempdata[i];
//            out[len-1-i][1]=0;
//        }
//        realInt[0]=0;
//        imgInt[0]=0;
//        realInt[len-1]=0;
//        imgInt[len-1]=0;
//        realInt2[0]=0;
//        imgInt2[0]=0;
//        realInt2[len-1]=0;
//        imgInt2[len-1]=0;
//        out[0][0]=0;
//        out[0][1]=0;
//        out[len-1][0]=0;
//        out[len-1][1]=0;

//        backFFT(realInt,imgInt,len,halfband);
//        backFFT(realInt2,imgInt2,len,halfband2);
//        backFFT_1(out,len,halfband3);
//        fftw_destroy_plan(p);
////        removeDC(halfband,len);
////        removeDC(halfband2,len);
////        removeDC(halfband3,len);
//        outlen=len;


//        for(i=0;i<outlen;i++)
//        {
//            velocity[i]=2*halfband[i]*1000;
//            disp[i]=2*halfband2[i]*100000;
//        }
////        character[0]= absMax(wave,len);
////        character[1]=rms(velocity,len);
////        character[2]=pp(disp+outlen/2,len/2);
////        character[3]=qiaodu(wave,len,rms(wave,len));


//        for(i=0;i<len/2;i++)
//           {
//               tempForenv[i][0]=out[i][1];
//               tempForenv[i][1]=0-out[i][0];

//               tempForenv[len/2+i][0]=0-out[len/2+i][1];
//               tempForenv[len/2+i][1]=out[len/2+i][0];
//           }
//           for(i=0;i<len;i++)
//           {
//               real[i]=tempForenv[i][0];
//               img[i]=tempForenv[i][1];
//           }
//           for(i=0;i<len;i++)
//               {
//                   real[i]=tempForenv[i][0];
//                   img[i]=tempForenv[i][1];
//               }

//           backFFT(real,img,len,envelopWave);
//           for(i=0;i<len;i++)
//               envelopWave[i]=sqrt(envelopWave[i]*envelopWave[i]+wave[i]*wave[i]);
////        character[4]=absMax(envelopWave,len);
//        fftw_free(wave1);
//        fftw_free(out);
//        fftw_free(tempForenv);
//							 
//							 
////        delete[] real;
////        delete[] img;
////        delete [] envelopWave;
////        delete[] wave;
////        delete[] halfband;
////        delete[] halfband2;
////        delete[] halfband3;
////        delete[] realInt;
////        delete[] imgInt;
////        delete[] realInt2;
////        delete[] imgInt2;
////        delete[] velocity;
////        delete[] disp;
//        return 0;
//    }
//    else return -1;
//}
