/*
 * ETH_W5500.c
 *
 *  Created on: 18 jul. 2022
 *      Author: mggarcia
 */

#include "ETH_W5500.h"
#include "C:\STM32-IDE\STM32-Library\ModBUS\ModBUS_Chelo.h"

SPI_ETH_RESET(struct W5500_SPI * x)
{
	HAL_GPIO_WritePin(x->RST_PORT, x->RST_PIN , GPIO_PIN_RESET);		//RST LOW
	HAL_Delay(100);
	HAL_GPIO_WritePin(x->RST_PORT, x->RST_PIN , GPIO_PIN_SET);			//RST HIGH
}

SPI_ETH_PHY_RESET(struct W5500_SPI * x)
{
uint8_t a;

	x->TX[0]=0x00;
	x->TX[1]=0x2E;
	x->TX[2]=0x00;
	x->TX[3]=0x00;
	SPI_ETH(x);			//Realizo la letura de PHYCFGR
	a=x->RX[3];
	a=a&0xFE;
	x->TX[0]=0x00;
	x->TX[1]=0x2E;
	x->TX[2]=0x04;
	x->TX[3]=a;
	SPI_ETH(x);
}

uint8_t  SPI_ETH(struct  W5500_SPI * x )
{
	HAL_GPIO_WritePin(x->NSS_PORT, x->NSS_PIN , GPIO_PIN_RESET);				// NSS LOW
	HAL_SPI_TransmitReceive(x->SPI, x->TX , x->RX, 4, 100);						//SPI COMM
	HAL_GPIO_WritePin(x->NSS_PORT, x->NSS_PIN , GPIO_PIN_SET);			//NSS HIGH
	return (x->RX[3]);
}

uint16_t SPI_ETH_REG(struct W5500_SPI * x,  uint16_t ph_addr, uint8_t addr,uint8_t op, uint8_t * data, uint8_t lnght)
{
 uint16_t res=0;
 uint8_t  a=op;

 x->TX[0]= (ph_addr & 0xFF00)>>8; 				//High Address Phase
 x->TX[1]= ph_addr & 0x00FF;					//Low Address Phase
 x->TX[2]= ((addr)<<3)|((op)<<2);
 x->TX[3]=0x00;
 if(op == 1)								//Write operation
 {
	 for(int i=0; i<(lnght); i++)			//Esto solo incrementa 256 ubicaciones
	 {
		x->TX[3]=data[i];					//Put data to be sent
		res=SPI_ETH(x);						//Send data to device
		x->TX[1]++;							//Address increased once
	 }
	 return(res);
 }
	 if(op == 0)							//Read operation
	 {
		 x->TX[3]=0x00;
		 if (lnght==2)
		{
		for(int i=0; i<(lnght); i++)
			{
			res|=SPI_ETH(x);
			x->TX[1]++;
			if (i==0)res=res<<8;
			}
		 return(res);
		}else
		{
			res=SPI_ETH(x);
			x->TX[1]++;
			return(res);
		}
 	 }
}

uint16_t SPI_ETH_WR_REG_16(struct W5500_SPI * x, uint16_t addr, uint16_t  data, uint8_t socket)
{
	uint16_t res=0;
	uint8_t num[2];
			x->TX[2]= ((socket)<<3)|0x04;
			num[1] = data & 0x00FF ;
			num[0] = (data & 0xFF00)>>8 ;

			 for(int i=0; i<(2); i++)
				 {
				 	x->TX[1]= addr & 0x00FF;//x->TX[2]= addr & 0x00FF;
				 	x->TX[0]=(addr & 0xFF00)>>8;//x->TX[1]=(addr & 0xFF00)>>8;
					addr++;
					x->TX[3]=num[i];
					res=SPI_ETH(x);
				 }
			 return(res);
}

uint16_t SPI_ETH_RD_REG_16(struct W5500_SPI * x, uint16_t addr, uint8_t op, uint8_t * data, uint16_t lnght, uint8_t socket )
{
		x->TX[2]= ((socket)<<3)|0x04;
	if(lnght < 2048)
	{
		if(op == SPI_READ)
		 {
			x->TX[3]=0x00;

			for(int i=0; i<(lnght); i++)
				{
				x->TX[1] = addr & 0x00FF;
				x->TX[0] = (addr & 0xFF00)>>8;
				data[i]=SPI_ETH(x);
				addr++;
				}
		 }
	 return(0);
	 }
	else
	{
	return(1);
	}
}

uint16_t SPI_ETH_RD_RCV_REG_16(struct W5500_SPI * x, uint16_t addr, uint8_t * data, uint16_t offset, uint16_t lnght, uint8_t socket )
{
		x->TX[2]= ((socket)<<3)|0x00;
	if(lnght < 2048)
	{
			x->TX[3]=0x00;
			for(int i=0; i<(lnght); i++)
				{
				x->TX[1] = addr & 0x00FF;
				x->TX[0] = (addr & 0xFF00)>>8;
				data[i+offset]=SPI_ETH(x);
				addr++;
				}
		return (0); //Retorno la dirección del puntero a la memoria
	}
	else
	{
	return(1);
	}
}

uint16_t SPI_ETH_WR_TX_REG_16(struct W5500_SPI * x, uint16_t addr, uint8_t * data, uint16_t offset, uint16_t lnght, uint8_t socket )
{
		x->TX[2]= ((socket)<<3)|0x04;
	if(lnght < 2048)
	{

			for(int i=0; i<(lnght); i++)
				{
				x->TX[1] = addr & 0x00FF;
				x->TX[0] = (addr & 0xFF00)>>8;
				x->TX[3]= x->data[i+offset];
				SPI_ETH(x);
				addr++;
				}
		return (0); //Retorno la dirección del puntero a la memoria
	}
	else
	{
	return(1);
	}
}

eth_wr_SOCKET_CMD(struct  W5500_SPI * y, uint8_t socket, uint8_t z)
{
	y->TX[0]= 0x00; 				//High Address Phase Hardcoded MR Register
	y->TX[1]= 0x01;					//Low Address Phase Command
	y->TX[2]= ((socket<<3)|0x04);	//Control Phase address + R/W + OP Mode
	y->TX[3]= z ;		//Load data to save
	SPI_ETH(y);
}

eth_wr_SOCKET_MODE(struct  W5500_SPI * y, uint8_t socket, uint8_t z)
{
		y->TX[0]= 0x00; 				//High Address Phase Hardcoded MR Register
		y->TX[1]= 0x00;					//Low Address Phase Mode
		y->TX[2]= ((socket)<<3)|0x04;	//Control Phase address + R/W + OP Mode
		y->TX[3]= z ;		//Load data to save

	SPI_ETH(y);
}


uint8_t eth_init(struct W5500_SPI * ETH)
{
  	 SPI_ETH_RESET(ETH);	//Reset W5500 por hardware
  	 HAL_Delay(800);

	 ETH->T8=0x00;
	 SPI_ETH_REG(ETH, IMR,COMM_REG	,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-INTERRUPT MASK CLEARED\r\n",strlen("\r\nETH-W5500-INTERRUPT MASK CLEARED\r\n"));
	 ETH->T8=0x0F;
	 SPI_ETH_REG(ETH, RTR,COMM_REG	,SPI_WRITE, ETH->T8,1);
	 ETH->T8=0xA0;
	 SPI_ETH_REG(ETH, RTR+1,COMM_REG,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-RETRY TIME SET\r\n",strlen("\r\nETH-W5500-RETRY TIME SET\r\n"));
	 ETH->T8=0x07;
	 SPI_ETH_REG(ETH, RCR,COMM_REG	,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-RETRY COUNT SET\r\n",strlen("\r\nETH-W5500-RETRY COUNT SET\r\n"));
	 ETH->T8=0x00;
	 SPI_ETH_REG(ETH, SIMR,COMM_REG	,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCKET INTERRUPT MASK CLEARED\r\n",strlen("\r\nETH-W5500-SOCKET INTERRUPT MASK CLEARED\r\n"));
	 SPI_ETH_REG(ETH, SHAR,COMM_REG,SPI_WRITE, ETH->SHAR,6);												//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-MAC SET\r\n",strlen("\r\nETH-W5500-MAC SET"));
	 SPI_ETH_REG(ETH, GAR,COMM_REG,SPI_WRITE, ETH->GAR,4);	//SPI_ETH_REG(ETH, GAR_ADDR_BASEH,GAR_ADDR_BASEL,SPI_WRITE, ETH->GAR,4);													//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-GATEWAY SET\r\n",strlen("\r\nETH-W5500-GATEWAY SET\r\n"));									//same for server and client
	 SPI_ETH_REG(ETH, SUBR,COMM_REG,SPI_WRITE, ETH->SUBR,4);												//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SUBNET SET\r\n",strlen("\r\nETH-W5500-SUBNET SET"));											//same for server and client
	 SPI_ETH_REG(ETH, SIPR,COMM_REG,SPI_WRITE, ETH->SIPR,4);												//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-IP SET\r\n",strlen("\r\nETH-W5500-IP SET"));

}

uint8_t eth_socket_init(struct W5500_SPI * ETH, uint8_t socket)
{

	 ETH->T8=0x02;
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,socket,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,socket,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S1_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S1_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S2_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S2_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S3_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S3_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S4_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S4_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S5_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S5_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S6_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S6_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_RXBUF_SIZE,S7_REG,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TXBUF_SIZE,S7_REG,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCKETS BUFFERS TX-RX SET\r\n",strlen("\r\nETH-W5500-SOCKETS BUFFERS TX-RX SET\r\n"));

	 ETH->T8=0x00;
	 SPI_ETH_REG(ETH, Sn_TX_WR,socket,SPI_WRITE, ETH->T8,1);
	 SPI_ETH_REG(ETH, Sn_TX_WR+1,socket,SPI_WRITE, ETH->T8,1);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCKET0 WR POINTER CLEAR\r\n",strlen("\r\nETH-W5500-SOCKET0 WR POINTER CLEAR\r\n"));

	 SPI_ETH_REG(ETH, Sn_PORT, socket,SPI_WRITE, ETH->S_PORT,2);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCK0 SOURCE PORT SET\r\n",strlen("\r\nETH-W5500-SOCK0 SOURCE PORT SET\r\n"));									// client

	 SPI_ETH_REG(ETH, Sn_DIPR,socket,SPI_WRITE, ETH->S_DIPR,4);									// client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCK0 DESTINTAION IP ADDRESS SET\r\n",strlen("\r\nETH-W5100-SOCK0 DESTINTAION IP ADDRESS SET\r\n"));

	 SPI_ETH_REG(ETH, Sn_DPORT,socket,SPI_WRITE, ETH->S_DPORT,2);
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCK0 DESTINTAION PORT SET\r\n",strlen("\r\nETH-W5100-SOCK0 DESTINTAION PORT SET\r\n"));




	 eth_wr_SOCKET_MODE(ETH,socket, MODE_TCP);																				//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-SOCK0 TCP MODE SET\r\n",strlen("\r\nETH-W5100-SOCK0 TCP MODE SET\r\n"));									//same for server and client


	 eth_wr_SOCKET_CMD(ETH,socket, OPEN);																					//same for server and client
	 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-OPEN SOCKET\r\n",strlen("\r\nETH-W5100-OPEN SOCKET\r\n"));									//same for server and client

	 if(ETH->S_ENserver == 1)
	 {
		 eth_wr_SOCKET_CMD(ETH,socket, LISTEN);																				//only for server
		 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-LISTEN SOCKET\r\n",strlen("\r\nETH-W5100-LISTEN SOCKET\r\n"));							//only for server
	 }
	 else
	 {

		 	 eth_wr_SOCKET_CMD(ETH,socket, CONNECT);																				//only for server
			 if(ETH->DBG) ITM0_Write("\r\nETH-W5500-CONNECT\r\n",strlen("\r\nETH-W5100-CONNECT\r\n"));											//only fir server
	 }

}

uint8_t eth_rd_SOCKET_STAT(struct  W5500_SPI * y, uint8_t socket)
{
			y->TX[0]= 0x00; 				//High Address Phase Hardcoded Stat Register
			y->TX[1]= 0x03;					//Low Address Phase Mode
			y->TX[2]= ((socket)<<3)|0x00;	   //Control Phase address + R/W + OP Mode
			SPI_ETH(y);
			return(y->RX[3]);

}

uint8_t eth_rd_SOCKET_CMD(struct  W5500_SPI * y, uint8_t socket)
{
			y->TX[0]= 0x00; 				//High Address Phase Hardcoded Command Register
			y->TX[1]= 0x01;					//Low Address Phase Mode
			y->TX[2]= ((socket)<<3)|0x00;	   //Control Phase address + R/W + OP Mode
			SPI_ETH(y);
			return(y->RX[3]);

}

uint16_t  eth_rd_SOCKET_DATA(struct W5500_SPI * ETH, uint8_t socket, uint16_t * mem_pointer, uint16_t sizedata)
{
	uint16_t S_bf_rcv_offset=0,
			 left_size=0,
			 upper_size=0,
			 destination_addr=0,
			 S_RX_RD=0,
			 S_get_offset=0,
			 S_get_start_address=0,
			 S_mem_pointer=0,
			 RX_MASK=0,
			 RX_BASE=0;
	uint8_t spi_Data[2];

	switch (socket)
	{
		/*case 0 :
		{
			RX_MASK=ETH->gS_RX_MASK;
			RX_BASE=ETH->gS_RX_BASE;
		}
		break;
		case 1 :
		{
			RX_MASK=ETH->gS1_RX_MASK;
			RX_BASE=ETH->gS1_RX_BASE;
		}
		break;
		case 2 :
		{
			RX_MASK=ETH->gS2_RX_MASK;
			RX_BASE=ETH->gS2_RX_BASE;
		}
		break;
		case 3 :
		{
			RX_MASK=ETH->gS3_RX_MASK;
			RX_BASE=ETH->gS3_RX_BASE;
		}
		break;*/
		default :
		{
			RX_MASK=ETH->gS_RX_MASK;
			RX_BASE=ETH->gS_RX_BASE;
		}
		break;
	}

	S_RX_RD = SPI_ETH_REG(ETH, Sn_RX_RD0 ,S0_REG ,SPI_READ, spi_Data,2);//S_RX_RD = SPI_ETH_REG(ETH, S_RX_RD_ADDR_BASEHH + socket ,S_RX_RD_ADDR_BASEHL ,SPI_READ, spi_Data,2);
	S_get_offset = S_RX_RD & RX_MASK;
	S_get_start_address  = RX_BASE + S_get_offset;
	if((S_get_offset  + sizedata )>(RX_MASK + 1))
		{
			upper_size = (RX_MASK + 1) - S_get_offset ;
			SPI_ETH_RD_RCV_REG_16(ETH , S_get_start_address , ETH->data , S_bf_rcv_offset, upper_size, socket);
			destination_addr+=upper_size;
			left_size=sizedata-upper_size;
			S_bf_rcv_offset=upper_size;
			SPI_ETH_RD_RCV_REG_16(ETH , RX_BASE , ETH->data , S_bf_rcv_offset, left_size, socket);
			*mem_pointer=S_RX_RD + sizedata;
		}
		else
			{
				SPI_ETH_RD_RCV_REG_16(ETH , S_get_start_address , ETH->data , S_bf_rcv_offset, sizedata, socket);
				*mem_pointer=S_RX_RD + sizedata;
			}
	return(mem_pointer);
}

uint16_t eth_wr_SOCKET_DATA(struct W5500_SPI * ETH, uint8_t socket, uint16_t * mem_pointer, uint16_t send_size)
{
	uint16_t S_bf_rcv_offset=0,
			 left_size=0,
			 upper_size=0,
			 source_addr=0,
			 Sn_TX_WR_local=0,
			 get_offset=0,
			 get_free_size=0,
			 get_start_address=0,
			 S_mem_pointer=0,
			 TX_MASK=0,
			 TX_BASE=0;
	uint8_t spi_Data[2];

	switch (socket)
	{
		/*case S7_REG :
		{
			TX_MASK=ETH->gS_TX_MASK;
			TX_BASE=ETH->gS_TX_BASE;
		}
		break;
		case 1 :
		{
			TX_MASK=ETH->gS1_TX_MASK;
			TX_BASE=ETH->gS1_TX_BASE;
		}
		break;
		case 2 :
		{
			TX_MASK=ETH->gS2_TX_MASK;
			TX_BASE=ETH->gS2_TX_BASE;
		}
		break;
		case 3 :
		{
			TX_MASK=ETH->gS3_TX_MASK;
			TX_BASE=ETH->gS3_TX_BASE;
		}
		break;*/
		default :
		{
			TX_MASK=ETH->gS_TX_MASK;
			TX_BASE=ETH->gS_TX_BASE;
		}
		break;
	}

	while(get_free_size<send_size)
			{
				get_free_size=SPI_ETH_REG(ETH, Sn_TX_FSR, S0_REG ,SPI_READ, spi_Data,2);//get_free_size=SPI_ETH_REG(ETH, 0x04 + socket, 0x20 ,SPI_READ, spi_Data,2); //Leo registro S_TX_FSR	=   0x420,
			}
				Sn_TX_WR_local = SPI_ETH_REG(ETH, Sn_TX_WR, S0_REG ,SPI_READ, spi_Data,2); // S_TX_RD =   0x424,Sn_TX_WR = SPI_ETH_REG(ETH, 0x04 + socket,0x24 ,SPI_READ, spi_Data,2); // S_TX_RD =   0x424,
				get_offset= Sn_TX_WR_local & TX_MASK;
				get_start_address=TX_BASE + get_offset;

				if((get_offset + send_size)>(TX_MASK + 1))
					{
						upper_size=( TX_MASK + 1) - get_offset;
						SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S_bf_rcv_offset, upper_size,socket);
						source_addr+=upper_size;
						left_size=send_size-upper_size;
						S_bf_rcv_offset=upper_size;
						SPI_ETH_WR_TX_REG_16(ETH , TX_BASE , ETH->data , S_bf_rcv_offset, left_size, socket);
						*mem_pointer=Sn_TX_WR_local + send_size;
					}
				else
					{
					SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S_bf_rcv_offset, send_size, socket);
					*mem_pointer=Sn_TX_WR_local + send_size;
					}

}

uint8_t SPI_ETH_SNIFF(struct W5500_SPY * Y,struct W5500_SPI * X)
{
	Y->MR=SPI_ETH_REG(X, 0x0000 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->GAR[0]=SPI_ETH_REG(X, 0x0001 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->GAR[1]=SPI_ETH_REG(X, 0x0002 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->GAR[2]=SPI_ETH_REG(X, 0x0003 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->GAR[3]=SPI_ETH_REG(X, 0x0004 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->SUBR[0]=SPI_ETH_REG(X, 0x0005 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SUBR[1]=SPI_ETH_REG(X, 0x0006 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SUBR[2]=SPI_ETH_REG(X, 0x0007 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SUBR[3]=SPI_ETH_REG(X, 0x0008 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->SHAR[0]=SPI_ETH_REG(X, 0x0009 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SHAR[1]=SPI_ETH_REG(X, 0x000A ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SHAR[2]=SPI_ETH_REG(X, 0x000B ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SHAR[3]=SPI_ETH_REG(X, 0x000C ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SHAR[4]=SPI_ETH_REG(X, 0x000D ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SHAR[5]=SPI_ETH_REG(X, 0x000E ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->SIPR[0]=SPI_ETH_REG(X, 0x000F ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SIPR[1]=SPI_ETH_REG(X, 0x0010 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SIPR[2]=SPI_ETH_REG(X, 0x0011 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->SIPR[3]=SPI_ETH_REG(X, 0x0012 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->INTLEVEL[0]=SPI_ETH_REG(X, 0x0013 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->INTLEVEL[1]=SPI_ETH_REG(X, 0x0014 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->IR=SPI_ETH_REG(X, 0x0015 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->IMR=SPI_ETH_REG(X, 0x0016 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->SIR=SPI_ETH_REG(X, 0x0017 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->SIMR=SPI_ETH_REG(X, 0x0018 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->RTR[0]=SPI_ETH_REG(X, 0x0019 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->RTR[1]=SPI_ETH_REG(X, 0x001A ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->RCR=SPI_ETH_REG(X, 0x001B ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PTIMER=SPI_ETH_REG(X, 0x001C ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PMAGIC=SPI_ETH_REG(X, 0x001D ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PHAR[0]=SPI_ETH_REG(X, 0x001E ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PHAR[1]=SPI_ETH_REG(X, 0x001F ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PHAR[2]=SPI_ETH_REG(X, 0x0020 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PHAR[3]=SPI_ETH_REG(X, 0x0021 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PHAR[4]=SPI_ETH_REG(X, 0x0022 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PHAR[5]=SPI_ETH_REG(X, 0x0023 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PSID[0]=SPI_ETH_REG(X, 0x0024 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PSID[1]=SPI_ETH_REG(X, 0x0025 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PMRU[0]=SPI_ETH_REG(X, 0x0026 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->PMRU[1]=SPI_ETH_REG(X, 0x0027 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->UIPR[0]=SPI_ETH_REG(X, 0x0028 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->UIPR[1]=SPI_ETH_REG(X, 0x0029 ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->UIPR[2]=SPI_ETH_REG(X, 0x002A ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->UIPR[3]=SPI_ETH_REG(X, 0x002B ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->UPORT[0]=SPI_ETH_REG(X, 0x002C ,COMM_REG,SPI_READ, Y->DUMMY,1);
	Y->UPORT[1]=SPI_ETH_REG(X, 0x002D ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->PHYCFGR=SPI_ETH_REG(X, 0x002E ,COMM_REG,SPI_READ, Y->DUMMY,1);

	Y->VERSIONR=SPI_ETH_REG(X, 0x0039 ,COMM_REG,SPI_READ, Y->DUMMY,1);

	//---------------------------port -------------------------//
	Y->Sn_MR=SPI_ETH_REG(X, 0x0000 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_CR=SPI_ETH_REG(X, 0x0001 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_IR=SPI_ETH_REG(X, 0x0002 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_SR=SPI_ETH_REG(X, 0x0003 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_PORT[0]=SPI_ETH_REG(X, 0x0004 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_PORT[1]=SPI_ETH_REG(X, 0x0005 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_DHAR[0]=SPI_ETH_REG(X, 0x0006 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DHAR[1]=SPI_ETH_REG(X, 0x0007 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DHAR[2]=SPI_ETH_REG(X, 0x0008 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DHAR[3]=SPI_ETH_REG(X, 0x0009 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DHAR[4]=SPI_ETH_REG(X, 0x000A ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DHAR[5]=SPI_ETH_REG(X, 0x000B ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_DIPR[0]=SPI_ETH_REG(X, 0x000C ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DIPR[1]=SPI_ETH_REG(X, 0x000D ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DIPR[2]=SPI_ETH_REG(X, 0x000E ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DIPR[3]=SPI_ETH_REG(X, 0x000F ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_DPORT[0]=SPI_ETH_REG(X, 0x0010 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_DPORT[1]=SPI_ETH_REG(X, 0x0011 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_MSSR[0]=SPI_ETH_REG(X, 0x0012 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_MSSR[1]=SPI_ETH_REG(X, 0x0013 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TOS=SPI_ETH_REG(X, 0x0015 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TTL=SPI_ETH_REG(X, 0x0016 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_RXBUF_SIZE=SPI_ETH_REG(X, 0x001E ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TXBUF_SIZE=SPI_ETH_REG(X, 0x001F ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TX_FSR[0]=SPI_ETH_REG(X, 0x0020 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_TX_FSR[1]=SPI_ETH_REG(X, 0x0021 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TX_RD[0]=SPI_ETH_REG(X, 0x0022 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_TX_RD[1]=SPI_ETH_REG(X, 0x0023 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_TX_WR[0]=SPI_ETH_REG(X, 0x0024 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_TX_WR[1]=SPI_ETH_REG(X, 0x0025 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_RX_RSR[0]=SPI_ETH_REG(X, 0x0026 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_RX_RSR[1]=SPI_ETH_REG(X, 0x0027 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_RX_RD[0]=SPI_ETH_REG(X, 0x0028 ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_RX_RD[1]=SPI_ETH_REG(X, 0x0029 ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_RX_WR[0]=SPI_ETH_REG(X, 0x002A ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_RX_WR[1]=SPI_ETH_REG(X, 0x002B ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_IMR=SPI_ETH_REG(X, 0x002C ,S0_REG,SPI_READ, Y->DUMMY,1);

	Y->Sn_FRAG[0]=SPI_ETH_REG(X, 0x002D ,S0_REG,SPI_READ, Y->DUMMY,1);
	Y->Sn_FRAG[1]=SPI_ETH_REG(X, 0x002E ,S0_REG,SPI_READ, Y->DUMMY,1);


	Y->Sn_KPALVTR=SPI_ETH_REG(X, 0x002F ,S0_REG,SPI_READ, Y->DUMMY,1);
};


uint8_t ETH_CORE(struct W5500_SPI * Y, UART_HandleTypeDef *PORTSER, struct MBUS * a_eth , struct MBUS * mb_wf)
{
	Y->S_status=eth_rd_SOCKET_STAT(Y,S0_REG);  //este era el bardo

		  switch(Y->S_status)	//Check Socket status
				 {
					 case SOCK_CLOSED :
						 {
							 if (Y->DBG) {ITM0_Write("\r\nS0_SOCK_CLOSED \r\n",strlen("\r\nS0_SOCK_CLOSED \r\n"));
							 TX_485_Enable(0);
							 HAL_UART_Transmit_IT(PORTSER, "\r\nS0_SOCK_CLOSED\r\n", strlen("\r\nS0_SOCK_CLOSED\r\n"));}

							 if (Y->CMD_Status==TIME_OUT)
							 {
								 eth_wr_SOCKET_CMD(Y,S0_REG, OPEN);																					//same for server and client
								 if (Y->DBG) ITM0_Write("\r\nETH-W5500-OPEN SOCKET\r\n",strlen("\r\nETH-W5100-OPEN SOCKET\r\n"));
							 }

							 if(Y->ETH_WDG >= 25000)
								 {
									 if (Y->DBG) ITM0_Write("\r\nETH_DEVICE_RESET \r\n",strlen("\r\nETH_DEVICE_RESET \r\n"));
									 SPI_ETH_RESET(Y);
									 Y->ETH_WDG=0;
								 }
							Y->CAM=0;
						 }
					 break;
					 case  SOCK_INIT :
						 {

							 if(Y->S_ENserver == 1)
							 {
								 if (Y->DBG) ITM0_Write("\r\nS0_SOCK_INIT \r\n",strlen("\r\nS0_SOCK_INIT \r\n"));
									eth_wr_SOCKET_CMD(Y, S0_REG, LISTEN );
									Y->ETH_WDG=0;
									Y->CAM=0;
							 }
							 else
							 {
								 if (Y->CMD_Status!=CONNECTING)
								 {
									eth_wr_SOCKET_CMD(Y,S0_REG, CONNECT);																				//only for server
									if (Y->DBG) {ITM0_Write("\r\nETH-W5500-CONNECT\r\n",strlen("\r\nETH-W5500-CONNECT\r\n"));
									TX_485_Enable(0);
									HAL_UART_Transmit_IT(PORTSER, "\r\nETH-W5500-CONNECT\r\n", strlen("\r\nETH-W5500-CONNECT\r\n"));}
									Y->ETH_WDG=0;
									Y->CAM=0;
									Y->CMD_Status=CONNECTING;
								 }

							 }
						 }
					 break;
					 case SOCK_LISTEN :
						 {
							 if (Y->DBG){ITM0_Write("\r\nS0_SOCK_LISTEN \r\n",strlen("\r\nS0_SOCK_LISTEN \r\n"));
							 TX_485_Enable(0);
							 HAL_UART_Transmit_IT(PORTSER, "\r\nS0_SOCK_LISTEN \r\n", strlen("\r\nS0_SOCK_LISTEN \r\n"));}
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_SYNSENT :
						 {
							 if (Y->DBG){ITM0_Write("\r\nS0_SOCK_SYNSENT \r\n",strlen("\r\nS0_SOCK_SYNSENT \r\n"));
							 TX_485_Enable(0);
							 HAL_UART_Transmit_IT(PORTSER, "\r\nS0_SOCK_SYNSENT\r\n", strlen("\r\nS0_SOCK_SYNSENT\r\n"));}
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_SYNRECV :
						 {
							 if (Y->DBG)ITM0_Write("\r\nS0_SOCK_SYNRECV \r\n",strlen("\r\nS0_SOCK_SYNRECV \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_ESTABLISHED :
						 {
							 Y->ETH_WDG=0;
							 if (Y->DBG){ITM0_Write("\r\nS0_SOCK_ESTABLISHED \r\n",strlen("\r\nS0_SOCK_ESTABLISHED \r\n"));
							 TX_485_Enable(0);
							 HAL_UART_Transmit_IT(PORTSER, "\r\nS0_SOCK_ESTABLISHED\r\n", strlen("\r\nS0_SOCK_ESTABLISHED\r\n"));}


							if (Y->S_ENserver == 1)  // Si el puerto Ethernet actúa como server (Recibe datos conexión mas pedido mbus
							{
								uint8_t local_exit=1;
								Y->S0_get_size = SPI_ETH_REG(Y, Sn_RX_RSR ,S0_REG, SPI_READ, Y->spi_Data,2);//Y->S0_get_size = SPI_ETH_REG(Y, S_RX_SZ_ADDR_BASEHH,S_RX_SZ_ADDR_BASEHL ,SPI_READ, Y->spi_Data,2);
									if(Y->S0_get_size != 0x00)
									{
										eth_rd_SOCKET_DATA(Y,S0_RX_BUFF,&Y->rx_mem_pointer,Y->S0_get_size); // read socket data
										SPI_ETH_WR_REG_16(Y,Sn_RX_RD0,Y->rx_mem_pointer,S0_REG );		// write rx memory pointer
										eth_wr_SOCKET_CMD(Y,S0_REG,RECV);
										eth_rd_SOCKET_CMD(Y,S0_REG);// write command to execute
										CopiaVector(a_eth->_MBUS_RCVD, Y->data, Y->S0_get_size, 0, 0 );
										a_eth->_n_MBUS_RCVD=Y->S0_get_size;

										if(Y->S0_get_size > 0)	{ Y->S_data_available=1;}					//Flag data received

										if(ModBUS_Check(a_eth->_MBUS_RCVD, a_eth->_n_MBUS_RCVD))		//Ckecks ModBUS type data
											{
												ModBUS(&a_eth);										//ModBUS protocol execution
												CopiaVector(Y->data, a_eth->_MBUS_2SND, a_eth->_n_MBUS_2SND, 0, 0);
											}
										else
											{
												if (Y->DBG) ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n\r\n NO MBUS \r\n\r\n"));
											}

										Y->send_size=a_eth->_n_MBUS_2SND;  //ModBUS data qty
										eth_wr_SOCKET_DATA(Y,S0_RX_BUFF, &Y->tx_mem_pointer, Y->send_size);	// write socket data
										SPI_ETH_WR_REG_16(Y,Sn_TX_WR,Y->tx_mem_pointer,S0_REG);			// write tx memory pointer//SPI_ETH_WR_REG_16(Y,0x424,tx_mem_pointer,0);			// write tx memory pointer
										eth_wr_SOCKET_CMD(Y,S0_REG,SEND);							// write command to execute
										eth_rd_SOCKET_CMD(Y,S0_REG);
									}
							}
							else	// Puerto ethernet labura como esclavo, se conecta al server para pedir datos
							{
								if (a_eth->_w_answer==0)
								{
									//Si ya envié vuelvo a enviar
									Y->data[0]=0x00;
									Y->data[1]=0x00;
									Y->data[2]=0x00;
									Y->data[3]=0x00;
									Y->data[4]=0x00;
									Y->data[5]=0x06;
									Y->data[6]=0x01;
									Y->data[7]=0x03;
									Y->data[8]=0x00;
									Y->data[9]=0x00;
									Y->data[10]=0x00;
									Y->data[11]=0x0A;
									Y->send_size=12;

									ModBUS_F03_Request(a_eth,0,16);
									CopiaVector(Y->data, a_eth->_MBUS_2SND, 12, 0, 0 );
									eth_wr_SOCKET_DATA(Y,S0_TX_BUFF, &Y->tx_mem_pointer, Y->send_size);	// write socket data
									SPI_ETH_WR_REG_16(Y,Sn_TX_WR,Y->tx_mem_pointer,S0_REG);			// write tx memory pointer
									eth_wr_SOCKET_CMD(Y,S0_REG,SEND);							// write command to execute
									uint16_t read=0;
									read=SPI_ETH_REG(Y, Sn_IR,S0_REG,SPI_READ, Y->GAR,1);
									a_eth->_w_answer=1;	// Waiting answer flag_w_answer=1;	// Waiting answer flag
									Y->MB_TOUT_ticks=0;	// restart counting
									if (Y->DBG) ITM0_Write("\r\n SENT MBUS REQ \r\n",strlen("\r\n\r\n SENT MBUS REQ \r\n\r\n"));
								}
								else
								{
								Y->S0_get_size = SPI_ETH_REG(Y, Sn_RX_RSR ,S0_REG ,SPI_READ, Y->spi_Data,2);
									if(Y->S0_get_size != 0x00)
									{
											eth_rd_SOCKET_DATA(Y,S0_RX_BUFF,&Y->rx_mem_pointer,Y->S0_get_size); // read socket data
											SPI_ETH_WR_REG_16(Y,Sn_RX_RD0,Y->rx_mem_pointer,S0_REG);		// write rx memory pointer
											eth_wr_SOCKET_CMD(Y,S0_REG,RECV);							// write command to execute
											if (Y->DBG) ITM0_Write("\r\n RCVD DATA \r\n",strlen("\r\n RCVD DATA \r\n"));
											CopiaVector(a_eth->_MBUS_RCVD, Y->data, Y->S0_get_size, 0, 0 );
											a_eth->_n_MBUS_RCVD=Y->S0_get_size;
											if(Y->S0_get_size > 0)	{ Y->S_data_available=1;}
											if(ModBUS_Check(a_eth->_MBUS_RCVD, a_eth->_n_MBUS_RCVD))		//Ckecks ModBUS type data
												{
													a_eth->_w_answer=0;  									//Si el mensaje recibido ya es modbus digo que ya recibi
													Y->MB_TOUT_ticks=0;
													ModBUS(a_eth);										//ModBUS protocol execution
													CopiaVector(Y->swap, a_eth->_MBUS_RCVD, a_eth->_n_MBUS_RCVD, 0, 0);
													CopiaVector(mb_wf->_Holding_Registers, a_eth->_Holding_Registers, 64, 0, 0);
													if (Y->DBG) ITM0_Write("\r\n RCVD MBUS REQ \r\n",strlen("\r\n\ RCVD MBUS REQ \r\n"));
												}
												else
													{
													if (Y->DBG) ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n NO MBUS \r\n"));
													}
										}

								}
							}
							Y->CAM=0;
						 }
					 break;
					 case SOCK_FIN_WAIT :
						 {
							 if (Y->DBG) ITM0_Write("\r\nS0_SOCK_FIN_WAIT \r\n",strlen("\r\nS0_SOCK_FIN_WAIT \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_CLOSING :
						 {
							 if (Y->DBG) ITM0_Write("\r\nS0_SOCK_CLOSING \r\n",strlen("\r\nS0_SOCK_CLOSING \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case  SOCK_TIME_WAIT :
						 {
							 if (Y->DBG) ITM0_Write("\r\nS0_SOCK_TIME_WAIT \r\n",strlen("\r\nS0_SOCK_TIME_WAIT \r\n"));
							eth_wr_SOCKET_CMD(Y,S0_REG, DISCON );
							SPI_ETH_REG(Y,Sn_CR ,S0_REG,SPI_READ, Y->spi_Data,100);
							Y->ETH_WDG=0;
							Y->CAM=0;
						 }
					 break;
					 case SOCK_CLOSE_WAIT :
						 {
							 if (Y->DBG) ITM0_Write("\r\nS0_SOCK_CLOSE_WAIT \r\n",strlen("\r\nS0_SOCK_CLOSE_WAIT \r\n"));
							eth_wr_SOCKET_CMD(Y,S0_REG,DISCON );
							SPI_ETH_REG(Y,Sn_CR,S0_REG,SPI_READ, Y->spi_Data,100);
							Y->ETH_WDG=0;
							Y->CAM=0;
						 }
					 break;
					 case SOCK_LAST_ACK :
						 {
							 if (Y->DBG) ITM0_Write("\r\n S0_SOCK_LAST_ACK \r\n",strlen("\r\n S0_SOCK_LAST_ACK \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_UDP :
						 {
							 if (Y->DBG) ITM0_Write("\r\n S0_SOCK_UDP \r\n",strlen("\r\n S0_SOCK_UDP \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case  SOCK_IPRAW :
						 {
							 if (Y->DBG) ITM0_Write("\r\n S0_SOCK_IPRAW \r\n",strlen("\r\n S0_SOCK_IPRAW \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case  SOCK_MACRAW :
						 {
							 if (Y->DBG) ITM0_Write("\r\n S0_SOCK_MACRAW \r\n",strlen("\r\n S0_SOCK_MACRAW \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;
					 case SOCK_PPOE :
						 {
							 if (Y->DBG) ITM0_Write("\r\n S0_SOCK_PPOE \r\n",strlen("\r\n S0_SOCK_PPOE \r\n"));
							 Y->ETH_WDG=0;
							 Y->CAM=0;
						 }
					 break;

					 default:
						 {

						 }
				 }
}

