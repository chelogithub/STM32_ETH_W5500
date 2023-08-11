/*
 * ETH_W5500.c
 *
 *  Created on: 18 jul. 2022
 *      Author: mggarcia
 */

#include "ETH_W5500.h"

SPI_ETH_RESET(struct W5500_SPI * x)
{
	HAL_GPIO_WritePin(x->RST_PORT, x->RST_PIN , GPIO_PIN_RESET);		//RST LOW
	HAL_Delay(1);
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
	uint8_t a=0;
		switch(socket)
		{
		case 0:	a=1; break;
		case 1:	a=5; break;
		case 2:	a=9; break;
		case 3:	a=13; break;
		case 4:	a=17; break;
		case 5:	a=21; break;
		case 6:	a=25; break;
		case 7:	a=29; break;

		}
			x->TX[2]= ((a)<<3)|0x04;	   //Control Phase address + write + OP Mode

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
	uint8_t a=0;
		switch(socket)
		{
			case 0:	a=1; break;
			case 1:	a=5; break;
			case 2:	a=9; break;
			case 3:	a=13; break;
			case 4:	a=17; break;
			case 5:	a=21; break;
			case 6:	a=25; break;
			case 7:	a=29; break;
		}
		x->TX[2]= ((a)<<3)|0x00;	   //Control Phase address + read + OP Mode

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
	uint8_t a=0;
		switch(socket)
		{
			case 0:	a=1; break;
			case 1:	a=5; break;
			case 2:	a=9; break;
			case 3:	a=13; break;
			case 4:	a=17; break;
			case 5:	a=21; break;
			case 6:	a=25; break;
			case 7:	a=29; break;
		}
		x->TX[2]= ((a)<<3)|0x00;	   //Control Phase address + read + OP Mode

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
	uint8_t a=0;
		switch(socket)
		{
			case 0:	a=1; break;
			case 1:	a=5; break;
			case 2:	a=9; break;
			case 3:	a=13; break;
			case 4:	a=17; break;
			case 5:	a=21; break;
			case 6:	a=25; break;
			case 7:	a=29; break;
		}
		x->TX[2]= ((a)<<3)|0x04;	   //Control Phase address + WRITE + OP Mode

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

eth_wr_SOCKET_CMD(struct  W5500_SPI * y, uint8_t s, uint8_t z)
{
	uint8_t a=0;
	switch(s)
	{
		case 0:	a=1; break;
		case 1:	a=5; break;
		case 2:	a=9; break;
		case 3:	a=13; break;
		case 4:	a=17; break;
		case 5:	a=21; break;
		case 6:	a=25; break;
		case 7:	a=29; break;
	}
	y->TX[0]= 0x00; 				//High Address Phase Hardcoded MR Register
	y->TX[1]= 0x01;					//Low Address Phase Command
	y->TX[2]= ((a)<<3)|0x04;	//Control Phase address + R/W + OP Mode
	y->TX[3]= z ;		//Load data to save
	SPI_ETH(y);
}

eth_wr_SOCKET_MODE(struct  W5500_SPI * y, uint8_t s, uint8_t z)
{
uint8_t a=0;
	switch(s)
	{
	case 0:	a=1; break;
	case 1:	a=5; break;
	case 2:	a=9; break;
	case 3:	a=13; break;
	case 4:	a=17; break;
	case 5:	a=21; break;
	case 6:	a=25; break;
	case 7:	a=29; break;

	}
		y->TX[0]= 0x00; 				//High Address Phase Hardcoded MR Register
		y->TX[1]= 0x00;					//Low Address Phase Mode
		y->TX[2]= ((a)<<3)|0x04;	//Control Phase address + R/W + OP Mode
		y->TX[3]= z ;		//Load data to save

	SPI_ETH(y);
}


uint8_t eth_init(struct W5500_SPI * ETH)
{
  	 SPI_ETH_RESET(ETH);	//Reset W5500 por hardware
  	 HAL_Delay(800);

  	 SPI_ETH_REG(ETH, GAR,COMM_REG,SPI_WRITE, ETH->GAR,4);

	 ETH->T8=0x00;
	 SPI_ETH_REG(ETH, IMR,COMM_REG	,SPI_WRITE, ETH->T8,1);

	 ETH->T8=0x0F;
	 SPI_ETH_REG(ETH, RTR,COMM_REG	,SPI_WRITE, ETH->T8,1);
	 ETH->T8=0xA0;
	 SPI_ETH_REG(ETH, RTR+1,COMM_REG,SPI_WRITE, ETH->T8,1);

	 ETH->T8=0x07;
	 SPI_ETH_REG(ETH, RCR,COMM_REG	,SPI_WRITE, ETH->T8,1);

	 ETH->T8=0x00;
	 SPI_ETH_REG(ETH, SIMR,COMM_REG	,SPI_WRITE, ETH->T8,1);

	 SPI_ETH_REG(ETH, SHAR,COMM_REG,SPI_WRITE, ETH->SHAR,6);												//same for server and client
	 ITM0_Write("\r\nETH-W5500-MAC SET\r\n",strlen("\r\nETH-W5500-MAC SET"));
	 SPI_ETH_REG(ETH, GAR,COMM_REG,SPI_WRITE, ETH->GAR,4);	//SPI_ETH_REG(ETH, GAR_ADDR_BASEH,GAR_ADDR_BASEL,SPI_WRITE, ETH->GAR,4);													//same for server and client
	 ITM0_Write("\r\nETH-W5500-GATEWAY SET\r\n",strlen("\r\nETH-W5500-GATEWAY SET\r\n"));									//same for server and client
	 SPI_ETH_REG(ETH, SUBR,COMM_REG,SPI_WRITE, ETH->SUBR,4);												//same for server and client
	 ITM0_Write("\r\nETH-W5500-SUBNET SET\r\n",strlen("\r\nETH-W5500-SUBNET SET"));											//same for server and client
	 SPI_ETH_REG(ETH, SIPR,COMM_REG,SPI_WRITE, ETH->SIPR,4);												//same for server and client
	 ITM0_Write("\r\nETH-W5500-IP SET\r\n",strlen("\r\nETH-W5500-IP SET"));





	 //same for server and client

/*-------------------TEST DE ESCRITURA DE DATOS -------------------------
uint16_t a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,S,T,U,V,W,X,Y,Z=0;

	 a=SPI_ETH_REG(ETH, GAR 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 b=SPI_ETH_REG(ETH, GAR+1 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 c=SPI_ETH_REG(ETH, GAR+2 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 d=SPI_ETH_REG(ETH, GAR+3 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 e=SPI_ETH_REG(ETH, SUBR	,COMM_REG,SPI_READ, ETH->SUBR,1);
	 f=SPI_ETH_REG(ETH, SUBR+1 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 g=SPI_ETH_REG(ETH, SUBR+2 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 h=SPI_ETH_REG(ETH, SUBR+3 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 i=SPI_ETH_REG(ETH, SHAR	,COMM_REG,SPI_READ, ETH->SHAR,1);
	 j=SPI_ETH_REG(ETH, SHAR+1 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 k=SPI_ETH_REG(ETH, SHAR+2 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 l=SPI_ETH_REG(ETH, SHAR+3 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 m=SPI_ETH_REG(ETH, SHAR+4 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 n=SPI_ETH_REG(ETH, SHAR+5 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 o=SPI_ETH_REG(ETH, SIPR	,COMM_REG,SPI_READ, ETH->SIPR,1);
	 p=SPI_ETH_REG(ETH, SIPR+1 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 q=SPI_ETH_REG(ETH, SIPR+2 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 r=SPI_ETH_REG(ETH, SIPR+3 	,COMM_REG,SPI_READ, ETH->GAR,1);


/-------------------TEST DE ESCRITURA DE DATOS -------------------------*/
}

uint8_t eth_socket_init(struct W5500_SPI * ETH, uint8_t socket)
{

	 ETH->T8=0x02;
	 SPI_ETH_REG(ETH, S_RXBUF_SIZE,S0_REG,SPI_WRITE, ETH->T8,1);
	 //HAL_Delay(100);
	 SPI_ETH_REG(ETH, S_TXBUF_SIZE,S0_REG,SPI_WRITE, ETH->T8,1);
	 //HAL_Delay(100);
	 //ITM0_Write("\r\nETH-W5500-SOCK0 TCP SET\r\n",strlen("\r\nETH-W5500-SOCK0 TCP SET"));									//same for server and client
	 SPI_ETH_REG(ETH, S_PORT, S0_REG,SPI_WRITE, ETH->S_PORT,2);									//same for server and client
	 //HAL_Delay(100);
	 ITM0_Write("\r\nETH-W5500-SOCK0 TCP REMOTE IP TO CONNECT\r\n",strlen("\r\nETH-W5500-SOCK0 TCP REMOTE IP TO CONNECT\r\n"));									// client
	 //HAL_Delay(100);

	 //-------------------TEST DE ESCRITURA DE DATOS -------------------------
	// uint16_t a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,S,T,U,V,W,X,Y,Z=0;
	 	/* SPI_ETH_REG(ETH, S_DIPR		,S0_REG,SPI_WRITE, ETH->GAR,4);
	 	 a=SPI_ETH_REG(ETH, S_PORT 		,S0_REG,SPI_READ, ETH->GAR,1);
	 	 b=SPI_ETH_REG(ETH, S_PORT+1 	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 c=SPI_ETH_REG(ETH, S_DIPR 		,S0_REG,SPI_READ, ETH->GAR,1);
	 	 d=SPI_ETH_REG(ETH, S_DIPR+1 	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 e=SPI_ETH_REG(ETH, S_DIPR+2	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 f=SPI_ETH_REG(ETH, S_DIPR+3 	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 g=SPI_ETH_REG(ETH, S_DPORT 	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 h=SPI_ETH_REG(ETH, S_DPORT+1 	,S0_REG,SPI_READ, ETH->GAR,1);
	 	 i=SPI_ETH_REG(ETH, PHYCFGR 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 	 j=SPI_ETH_REG(ETH, MR 			,S0_REG,SPI_READ, ETH->GAR,1);*/
	/* eth_wr_SOCKET_CMD(ETH,socket, OPEN);
	 a=SPI_ETH_REG(ETH, 0 ,S0_REG,SPI_READ, ETH->GAR,1);
	 b=SPI_ETH_REG(ETH, 1 ,S0_REG,SPI_READ, ETH->GAR,1);
	 c=SPI_ETH_REG(ETH, 2 ,S0_REG,SPI_READ, ETH->GAR,1);
	 d=SPI_ETH_REG(ETH, 3 ,S0_REG,SPI_READ, ETH->GAR,1);
	 e=SPI_ETH_REG(ETH, 4 ,S0_REG,SPI_READ, ETH->GAR,1);
	 f=SPI_ETH_REG(ETH, 5 ,S0_REG,SPI_READ, ETH->GAR,1);
	 g=SPI_ETH_REG(ETH, 6 ,S0_REG,SPI_READ, ETH->GAR,1);
	 h=SPI_ETH_REG(ETH, 7 ,S0_REG,SPI_READ, ETH->GAR,1);
	 i=SPI_ETH_REG(ETH, 8 ,S0_REG,SPI_READ, ETH->GAR,1);
	 j=SPI_ETH_REG(ETH, 9 ,S0_REG,SPI_READ, ETH->GAR,1);
	 k=SPI_ETH_REG(ETH, 10 ,S0_REG,SPI_READ, ETH->GAR,1);
	 l=SPI_ETH_REG(ETH, 11 ,S0_REG,SPI_READ, ETH->GAR,1);
	 m=SPI_ETH_REG(ETH, 12 ,S0_REG,SPI_READ, ETH->GAR,1);
	 n=SPI_ETH_REG(ETH, 13 ,S0_REG,SPI_READ, ETH->GAR,1);
	 o=SPI_ETH_REG(ETH, 14 ,S0_REG,SPI_READ, ETH->GAR,1);
	 p=SPI_ETH_REG(ETH, 15 ,S0_REG,SPI_READ, ETH->GAR,1);
	 q=SPI_ETH_REG(ETH, 16 ,S0_REG,SPI_READ, ETH->GAR,1);
	 r=SPI_ETH_REG(ETH, PHYCFGR 	,COMM_REG,SPI_READ, ETH->GAR,1);
	 S=SPI_ETH_REG(ETH, S_RXBUF_SIZE ,S0_REG,SPI_READ, ETH->GAR,1);
	 T=SPI_ETH_REG(ETH, S_TXBUF_SIZE ,S0_REG,SPI_READ, ETH->GAR,1);
	 U=SPI_ETH_REG(ETH, S_TX_FSR 	,S0_REG,SPI_READ, ETH->GAR,1);
	 V=SPI_ETH_REG(ETH, S_TX_FSR+1	,S0_REG,SPI_READ, ETH->GAR,1);
	 W=SPI_ETH_REG(ETH,  S_TX_RD 	,S0_REG,SPI_READ, ETH->GAR,1);
	 X=SPI_ETH_REG(ETH,  S_TX_RD+1 	,S0_REG,SPI_READ, ETH->GAR,1);
	 Y=SPI_ETH_REG(ETH, S_TX_WR 	,S0_REG,SPI_READ, ETH->GAR,1);
	 Z=SPI_ETH_REG(ETH, S_TX_WR+1 	,S0_REG,SPI_READ, ETH->GAR,1);*/

	 //-------------------TEST DE ESCRITURA DE DATOS -------------------------
/*
	 eth_wr_SOCKET_CMD(ETH,socket, OPEN);																					//same for server and client
	 ITM0_Write("\r\nETH-W5500-OPEN SOCKET\r\n",strlen("\r\nETH-W5500-OPEN SOCKET\r\n"));									//same for server and client

	if(ETH->S_ENserver == 1)
	 {
		 eth_wr_SOCKET_CMD(ETH,socket, LISTEN);																				//only for server
		 ITM0_Write("\r\nETH-W5500-LISTEN SOCKET\r\n",strlen("\r\nETH-W5500-LISTEN SOCKET\r\n"));							//only for server
	 }
	 else
	 {
		 	 eth_wr_SOCKET_CMD(ETH,socket, CONNECT);																				//only for server
			 ITM0_Write("\r\nETH-W5500-CONNECT\r\n",strlen("\r\nETH-W5500-CONNECT\r\n"));											//only fir server
	 }*/

}

uint8_t eth_rd_SOCKET_STAT(struct  W5500_SPI * y, uint8_t socket)
{
	uint8_t a=0;
		switch(socket)
		{
		case 0:	a=1; break;
		case 1:	a=5; break;
		case 2:	a=9; break;
		case 3:	a=13; break;
		case 4:	a=17; break;
		case 5:	a=21; break;
		case 6:	a=25; break;
		case 7:	a=29; break;

		}
			y->TX[0]= 0x00; 				//High Address Phase Hardcoded Stat Register
			y->TX[1]= 0x03;					//Low Address Phase Mode
			y->TX[2]= ((a)<<3)|0x00;	   //Control Phase address + R/W + OP Mode
			SPI_ETH(y);
			return(y->RX[3]);



	/*y->TX[0]=	SPI_READ;
	y->TX[1]=  S_SR_ADDR_BASEH + socket;
	y->TX[2]=  S_SR_ADDR_BASEL ;
	y->TX[3]= 0 ;		//Lo carga en la info a enviar
	SPI_ETH(y);*/

}

uint8_t eth_rd_SOCKET_CMD(struct  W5500_SPI * y, uint8_t socket)
{

	uint8_t a=0;
		switch(socket)
		{
		case 0:	a=1; break;
		case 1:	a=5; break;
		case 2:	a=9; break;
		case 3:	a=13; break;
		case 4:	a=17; break;
		case 5:	a=21; break;
		case 6:	a=25; break;
		case 7:	a=29; break;

		}
			y->TX[0]= 0x00; 				//High Address Phase Hardcoded Command Register
			y->TX[1]= 0x01;					//Low Address Phase Mode
			y->TX[2]= ((a)<<3)|0x00;	   //Control Phase address + R/W + OP Mode
			SPI_ETH(y);
			return(y->RX[3]);

	/*y->TX[0]= SPI_READ;
	y->TX[1]=  S_CR_ADDR_BASEH + socket;
	y->TX[2]=  S_CR_ADDR_BASEL ;
	y->TX[3]= 0 ;		//Lo carga en la info a enviar
	SPI_ETH(y);
	return(y->RX[3]);*/
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
		case 0 :
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
		break;
		default :
		{
			RX_MASK=ETH->gS_RX_MASK;
			RX_BASE=ETH->gS_RX_BASE;
		}
		break;
	}

	S_RX_RD = SPI_ETH_REG(ETH, S_RX_RD0 ,S0_REG ,SPI_READ, spi_Data,2);//S_RX_RD = SPI_ETH_REG(ETH, S_RX_RD_ADDR_BASEHH + socket ,S_RX_RD_ADDR_BASEHL ,SPI_READ, spi_Data,2);
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
			 Sn_TX_WR=0,
			 get_offset=0,
			 get_free_size=0,
			 get_start_address=0,
			 S_mem_pointer=0,
			 TX_MASK=0,
			 TX_BASE=0;
	uint8_t spi_Data[2];

	switch (socket)
	{
		case 0 :
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
		break;
		default :
		{
			TX_MASK=ETH->gS_TX_MASK;
			TX_BASE=ETH->gS_TX_BASE;
		}
		break;
	}

	while(get_free_size<send_size)
			{
				get_free_size=SPI_ETH_REG(ETH, S_TX_FSR, S0_REG ,SPI_READ, spi_Data,2);//get_free_size=SPI_ETH_REG(ETH, 0x04 + socket, 0x20 ,SPI_READ, spi_Data,2); //Leo registro S_TX_FSR	=   0x420,
			}
				Sn_TX_WR = SPI_ETH_REG(ETH, S_TX_WR, S0_REG ,SPI_READ, spi_Data,2); // S_TX_RD =   0x424,Sn_TX_WR = SPI_ETH_REG(ETH, 0x04 + socket,0x24 ,SPI_READ, spi_Data,2); // S_TX_RD =   0x424,
				get_offset= Sn_TX_WR & TX_MASK;
				get_start_address=TX_BASE + get_offset;

				if((get_offset + send_size)>(TX_MASK + 1))
					{
						upper_size=( TX_MASK + 1) - get_offset;
						SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S_bf_rcv_offset, upper_size,socket);
						source_addr+=upper_size;
						left_size=send_size-upper_size;
						S_bf_rcv_offset=upper_size;
						SPI_ETH_WR_TX_REG_16(ETH , TX_BASE , ETH->data , S_bf_rcv_offset, left_size, socket);
						*mem_pointer=Sn_TX_WR + send_size;
					}
				else
					{
					SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S_bf_rcv_offset, send_size, socket);
					*mem_pointer=Sn_TX_WR + send_size;
					}

}

