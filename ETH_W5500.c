/*
 * ETH_W5100.c
 *
 *  Created on: 18 jul. 2022
 *      Author: mggarcia
 */

#include "ETH_W5100.h"


uint8_t  SPI_ETH(struct  W5100_SPI * x )
{
	HAL_GPIO_WritePin(x->NSS_PORT, x->NSS_PIN , GPIO_PIN_RESET);				// NSS LOW
	HAL_SPI_TransmitReceive(x->SPI, x->TX , x->RX, 4, 100);						//SPI COMM
	HAL_GPIO_WritePin(x->NSS_PORT, x->NSS_PIN , GPIO_PIN_SET);			//NSS HIGH
	return (x->RX[3]);
}

uint16_t SPI_ETH_REG(struct W5100_SPI * x, uint8_t addrh, uint8_t addrl, uint8_t op, uint8_t * data, uint8_t lnght)
{
 uint16_t res=0;

 x->TX[0]= op; //asigno lectura o escritura
 x->TX[1]= addrh;
 x->TX[2]= addrl;
 x->TX[3]=0x00;
 if(op == SPI_WRITE)
 {
	 for(int i=0; i<(lnght); i++)
	 {
		x->TX[3]=data[i];
		res=SPI_ETH(x);
		x->TX[2]++;
	 }
	 return(res);
 }
	 if(op == SPI_READ)
	 {
		 x->TX[3]=0x00;
		 if (lnght==2)
		{
		for(int i=0; i<(lnght); i++)
			{
			res|=SPI_ETH(x);
			x->TX[2]++;
			if (i==0)res=res<<8;
			}
		 return(res);
		}else
		{
			res=SPI_ETH(x);
			x->TX[2]++;
			return(res);
		}
 	 }
}

uint16_t SPI_ETH_WR_REG_16(struct W5100_SPI * x, uint16_t addr, uint16_t  data)
{
 uint16_t res=0;
 uint8_t num[2];

 x->TX[0]= SPI_WRITE; //asigno lectura o escritura

 num[1] = data & 0x00FF ;
 num[0] = (data & 0xFF00)>>8 ;

	 for(int i=0; i<(2); i++)
	 {
		x->TX[2]= addr & 0x00FF;
		x->TX[1]=(addr & 0xFF00)>>8;
		addr++;
		x->TX[3]=num[i];
		res=SPI_ETH(x);
	 }
	 return(res);
}

uint16_t SPI_ETH_RD_REG_16(struct W5100_SPI * x, uint16_t addr, uint8_t op, uint8_t * data, uint16_t lnght )
{
	if(lnght < 2048)
	{
		if(op == SPI_READ)
		 {
			x->TX[3]=0x00;
			for(int i=0; i<(lnght); i++)
				{
				x->TX[2] = addr & 0x00FF;
				x->TX[1] = (addr & 0xFF00)>>8;
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

uint16_t SPI_ETH_RD_RCV_REG_16(struct W5100_SPI * x, uint16_t addr, uint8_t * data, uint16_t offset, uint16_t lnght )
{
	x->TX[0]=0x0F; //read operation
	if(lnght < 2048)
	{
			x->TX[3]=0x00;
			for(int i=0; i<(lnght); i++)
				{
				x->TX[2] = addr & 0x00FF;
				x->TX[1] = (addr & 0xFF00)>>8;
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

uint16_t SPI_ETH_WR_TX_REG_16(struct W5100_SPI * x, uint16_t addr, uint8_t * data, uint16_t offset, uint16_t lnght )
{
	x->TX[0]=0xF0; //write operation
	if(lnght < 2048)
	{

			for(int i=0; i<(lnght); i++)
				{
				x->TX[2] = addr & 0x00FF;
				x->TX[1] = (addr & 0xFF00)>>8;
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

eth_wr_SOCKET_CMD(struct  W5100_SPI * y, uint8_t s, uint8_t z)
{
	y->TX[0]= SPI_WRITE ;
	y->TX[1]= S0_CR_ADDR_BASEH + s;
	y->TX[2]= S0_CR_ADDR_BASEL ;
	y->TX[3]= z ;		//Lo carga en la info a enviar
	SPI_ETH(y);
}

eth_wr_SOCKET_MODE(struct  W5100_SPI * y, uint8_t s, uint8_t z)
{
	y->TX[0]= SPI_WRITE ;
	y->TX[1]= S0_MR_ADDR_BASEH + s;
	y->TX[2]= S0_MR_ADDR_BASEL ;
	y->TX[3]= z ;		//Lo carga en la info a enviar
	SPI_ETH(y);
}


uint8_t eth_init(struct W5100_SPI * ETH)
{
	 SPI_ETH_REG(ETH, GAR_ADDR_BASEH,GAR_ADDR_BASEL,SPI_WRITE, ETH->GAR,4);													//same for server and client
	 ITM0_Write("\r\nETH-W5100-GATEWAY SET\r\n",strlen("\r\nETH-W5100-GATEWAY SET\r\n"));									//same for server and client
	 SPI_ETH_REG(ETH, SUBR_ADDR_BASEH,SUBR_ADDR_BASEL,SPI_WRITE, ETH->SUBR,4);												//same for server and client
	 ITM0_Write("\r\nETH-W5100-SUBNET SET\r\n",strlen("\r\nETH-W5100-SUBNET SET"));											//same for server and client
	 SPI_ETH_REG(ETH, SHAR_ADDR_BASEH,SHAR_ADDR_BASEL,SPI_WRITE, ETH->SHAR,6);												//same for server and client
	 ITM0_Write("\r\nETH-W5100-MAC SET\r\n",strlen("\r\nETH-W5100-MAC SET"));												//same for server and client
	 SPI_ETH_REG(ETH, SIPR_ADDR_BASEH,SIPR_ADDR_BASEL,SPI_WRITE, ETH->SIPR,4);												//same for server and client
	 ITM0_Write("\r\nETH-W5100-IP SET\r\n",strlen("\r\nETH-W5100-IP SET"));													//same for server and client
	 SPI_ETH_REG(ETH, RMSR_ADDR_BASEH,RMSR_ADDR_BASEL,SPI_WRITE, &ETH->RMSR,1);												//same for server and client
	 ITM0_Write("\r\nETH-W5100-DEFINE SOCKET RX MEMORY 2K\r\n",strlen("\r\nETH-W5100-DEFINE SOCKET RX MEMORY 2K\r\n")); 	//same for server and client
	 SPI_ETH_REG(ETH, TMSR_ADDR_BASEH,TMSR_ADDR_BASEL,SPI_WRITE, &ETH->TMSR,1);												//same for server and client
	 ITM0_Write("\r\nETH-W5100-DEFINE SOCKET TX MEMORY 2K\r\n",strlen("\r\nETH-W5100-DEFINE SOCKET TX MEMORY 2K\r\n"));		//same for server and client

}

uint8_t eth_socket_init(struct W5100_SPI * ETH, uint8_t socket)
{
	 eth_wr_SOCKET_MODE(ETH,socket, MODE_TCP);																				//same for server and client
	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP SET\r\n",strlen("\r\nETH-W5100-SOCK0 TCP SET"));									//same for server and client
	 SPI_ETH_REG(ETH, S0_PORT_ADDR_BASELH,S0_PORT_ADDR_BASELL,SPI_WRITE, ETH->S0_PORT,2);									//same for server and client

	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP REMOTE IP TO CONNECT\r\n",strlen("\r\nETH-W5100-SOCK0 TCP REMOTE IP TO CONNECT\r\n"));									// client
	 SPI_ETH_REG(ETH, 0x04,0x0C,SPI_WRITE, ETH->S0_DIPR,4);									// client
	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP REMOTE PORT TO CONNECT\r\n",strlen("\r\nETH-W5100-SOCK0 TCP REMOTE PORT TO CONNECT\r\n"));									// client
	 SPI_ETH_REG(ETH, 0x04,0x10,SPI_WRITE, ETH->S0_DPORT,2);									// client


	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP PORT SET\r\n",strlen("\r\nETH-W5100-SOCK0 TCP PORT SET\r\n"));						//same for server and client
	 eth_wr_SOCKET_CMD(ETH,socket, OPEN);																					//same for server and client
	 ITM0_Write("\r\nETH-W5100-OPEN SOCKET\r\n",strlen("\r\nETH-W5100-OPEN SOCKET\r\n"));									//same for server and client

	 if(ETH->S0_ENserver == 1)
	 {
		 eth_wr_SOCKET_CMD(ETH,socket, LISTEN);																				//only for server
		 ITM0_Write("\r\nETH-W5100-LISTEN SOCKET\r\n",strlen("\r\nETH-W5100-LISTEN SOCKET\r\n"));							//only for server
	 }
	 else
	 {

		 	 eth_wr_SOCKET_CMD(ETH,socket, CONNECT);																				//only for server
			 ITM0_Write("\r\nETH-W5100-CONNECT\r\n",strlen("\r\nETH-W5100-CONNECT\r\n"));											//only fir server
	 }
}

uint8_t eth_rd_SOCKET_STAT(struct  W5100_SPI * y, uint8_t socket)
{
	y->TX[0]= SPI_READ;
	y->TX[1]=  S0_SR_ADDR_BASEH + socket;
	y->TX[2]=  S0_SR_ADDR_BASEL ;
	y->TX[3]= 0 ;		//Lo carga en la info a enviar
	SPI_ETH(y);
	return(y->RX[3]);
}

uint8_t eth_rd_SOCKET_CMD(struct  W5100_SPI * y, uint8_t socket)
{
	y->TX[0]= SPI_READ;
	y->TX[1]=  S0_CR_ADDR_BASEH + socket;
	y->TX[2]=  S0_CR_ADDR_BASEL ;
	y->TX[3]= 0 ;		//Lo carga en la info a enviar
	SPI_ETH(y);
	return(y->RX[3]);
}

uint16_t  eth_rd_SOCKET_DATA(struct W5100_SPI * ETH, uint8_t socket, uint16_t * mem_pointer, uint16_t sizedata)
{
	uint16_t S0_bf_rcv_offset=0,
			 left_size=0,
			 upper_size=0,
			 destination_addr=0,
			 S0_RX_RD=0,
			 S0_get_offset=0,
			 S0_get_start_address=0,
			 S0_mem_pointer=0,
			 RX_MASK=0,
			 RX_BASE=0;
	uint8_t spi_Data[2];

	switch (socket)
	{
		case 0 :
		{
			RX_MASK=ETH->gS0_RX_MASK;
			RX_BASE=ETH->gS0_RX_BASE;
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
			RX_MASK=ETH->gS0_RX_MASK;
			RX_BASE=ETH->gS0_RX_BASE;
		}
		break;
	}

	S0_RX_RD = SPI_ETH_REG(ETH, S0_RX_RD_ADDR_BASEHH + socket ,S0_RX_RD_ADDR_BASEHL ,SPI_READ, spi_Data,2);
	S0_get_offset = S0_RX_RD & RX_MASK;
	S0_get_start_address  = RX_BASE + S0_get_offset;
	if((S0_get_offset  + sizedata )>(RX_MASK + 1))
		{
			upper_size = (RX_MASK + 1) - S0_get_offset ;
			SPI_ETH_RD_RCV_REG_16(ETH , S0_get_start_address , ETH->data , S0_bf_rcv_offset, upper_size);
			destination_addr+=upper_size;
			left_size=sizedata-upper_size;
			S0_bf_rcv_offset=upper_size;
			SPI_ETH_RD_RCV_REG_16(ETH , RX_BASE , ETH->data , S0_bf_rcv_offset, left_size);
			*mem_pointer=S0_RX_RD + sizedata;
		}
		else
			{
				SPI_ETH_RD_RCV_REG_16(ETH , S0_get_start_address , ETH->data , S0_bf_rcv_offset, sizedata);
				*mem_pointer=S0_RX_RD + sizedata;
			}
	return(mem_pointer);
}

uint16_t eth_wr_SOCKET_DATA(struct W5100_SPI * ETH, uint8_t socket, uint16_t * mem_pointer, uint16_t send_size)
{
	uint16_t S0_bf_rcv_offset=0,
			 left_size=0,
			 upper_size=0,
			 source_addr=0,
			 Sn_TX_WR=0,
			 get_offset=0,
			 get_free_size=0,
			 get_start_address=0,
			 S0_mem_pointer=0,
			 TX_MASK=0,
			 TX_BASE=0;
	uint8_t spi_Data[2];

	switch (socket)
	{
		case 0 :
		{
			TX_MASK=ETH->gS0_TX_MASK;
			TX_BASE=ETH->gS0_TX_BASE;
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
			TX_MASK=ETH->gS0_TX_MASK;
			TX_BASE=ETH->gS0_TX_BASE;
		}
		break;
	}

	while(get_free_size<send_size)
			{
				get_free_size=SPI_ETH_REG(ETH, 0x04 + socket, 0x20 ,SPI_READ, spi_Data,2); //Leo registro S0_TX_FSR	=   0x420,
			}
				Sn_TX_WR = SPI_ETH_REG(ETH, 0x04 + socket,0x24 ,SPI_READ, spi_Data,2); // S0_TX_RD =   0x424,
				get_offset= Sn_TX_WR & TX_MASK;
				get_start_address=TX_BASE + get_offset;

				if((get_offset + send_size)>(TX_MASK + 1))
					{
						upper_size=( TX_MASK + 1) - get_offset;
						SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S0_bf_rcv_offset, upper_size);
						source_addr+=upper_size;
						left_size=send_size-upper_size;
						S0_bf_rcv_offset=upper_size;
						SPI_ETH_WR_TX_REG_16(ETH , TX_BASE , ETH->data , S0_bf_rcv_offset, left_size);
						*mem_pointer=Sn_TX_WR + send_size;
					}
				else
					{
					SPI_ETH_WR_TX_REG_16(ETH , get_start_address , ETH->data , S0_bf_rcv_offset, send_size);
					*mem_pointer=Sn_TX_WR + send_size;
					}

}
/*
 *
 *  ITM0_Write("\r\n SET-UP W5100 \r\n",strlen("\r\n SET-UP W5100 \r\n"));
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	 ETH.operacion=SPI_WRITE;
	 ETH.TX[1]= 0;
	 ETH.TX[2]= 1;
	 ETH.TX[3]= 192;

	 spi_Data[0]=192;
	 spi_Data[1]=168;
	 spi_Data[2]=0;
	 spi_Data[3]=1;
	 SPI_ETH_REG(&ETH, GAR_ADDR_BASEH,GAR_ADDR_BASEL,SPI_WRITE, spi_Data,4);
	 ITM0_Write("\r\nETH-W5100-GATEWAY SET\r\n",strlen("\r\nETH-W5100-GATEWAY SET\r\n"));
	 spi_Data[0]=255;
	 spi_Data[1]=255;
	 spi_Data[2]=255;
	 spi_Data[3]=0;
	 SPI_ETH_REG(&ETH, SUBR_ADDR_BASEH,SUBR_ADDR_BASEL,SPI_WRITE, spi_Data,4);
	 ITM0_Write("\r\nETH-W5100-SUBNET SET\r\n",strlen("\r\nETH-W5100-SUBNET SET"));
	 spi_Data[0]=0x00;
	 spi_Data[1]=0x08;
	 spi_Data[2]=0xDC;
	 spi_Data[3]=0x00;
	 spi_Data[4]=0x00;
	 spi_Data[5]=0x01;
	 SPI_ETH_REG(&ETH, SHAR_ADDR_BASEH,SHAR_ADDR_BASEL,SPI_WRITE, spi_Data,6);
	 ITM0_Write("\r\nETH-W5100-MAC SET\r\n",strlen("\r\nETH-W5100-MAC SET"));
	 spi_Data[0]=192;
	 spi_Data[1]=168;
	 spi_Data[2]=0;
	 spi_Data[3]=3;
	 SPI_ETH_REG(&ETH, SIPR_ADDR_BASEH,SIPR_ADDR_BASEL,SPI_WRITE, spi_Data,4);
	 ITM0_Write("\r\nETH-W5100-IP SET\r\n",strlen("\r\nETH-W5100-IP SET"));
	 spi_Data[0]=0x01;
	 SPI_ETH_REG(&ETH, S0_MR_ADDR_BASEH,S0_MR_ADDR_BASEL,SPI_WRITE, spi_Data,1);
	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP SET\r\n",strlen("\r\nETH-W5100-SOCK0 TCP SET"));
	 spi_Data[0]=0x01;
	 spi_Data[1]=0xF6;  //Puerto 502 0x1F6
	 SPI_ETH_REG(&ETH, S0_PORT_ADDR_BASELH,S0_PORT_ADDR_BASELL,SPI_WRITE, spi_Data,2);
	 ITM0_Write("\r\nETH-W5100-SOCK0 TCP PORT SET\r\n",strlen("\r\nETH-W5100-SOCK0 TCP PORT SET\r\n"));
	 spi_Data[0]=0x55;	//2K Memory socket
	 SPI_ETH_REG(&ETH, RMSR_ADDR_BASEH,RMSR_ADDR_BASEL,SPI_WRITE, spi_Data,1);
	 ITM0_Write("\r\nETH-W5100-DEFINE SOCKET RX MEMORY 2K\r\n",strlen("\r\nETH-W5100-DEFINE SOCKET RX MEMORY 2K\r\n"));
	 spi_Data[0]=0x55;	//2K Memory socket
	 SPI_ETH_REG(&ETH, TMSR_ADDR_BASEH,TMSR_ADDR_BASEL,SPI_WRITE, spi_Data,1);
	 ITM0_Write("\r\nETH-W5100-DEFINE SOCKET TX MEMORY 2K\r\n",strlen("\r\nETH-W5100-DEFINE SOCKET TX MEMORY 2K\r\n"));
	 SPI_ETH_PORT_CMD(&ETH, OPEN);
	 ITM0_Write("\r\nETH-W5100-OPEN SOCKET\r\n",strlen("\r\nETH-W5100-OPEN SOCKET\r\n"));
	 SPI_ETH_PORT_CMD(&ETH, LISTEN);
	 ITM0_Write("\r\nETH-W5100-LISTEN SOCKET\r\n",strlen("\r\nETH-W5100-LISTEN SOCKET\r\n"));



SPI_READ_EN=1;
ETH.operacion=SPI_READ;
ETH.TX[1]= 0;
ETH.TX[2]= 1;
ETH.TX[3]= 0;

 ----------------------------------------------------------------------------------------------------


	  if(spi_no_debug)
	  	  {
	  if(SPI_READ_EN)
	  {


		 W5100_socket0_STATUS = SPI_ETH_REG(&ETH, S0_SR_ADDR_BASEH,S0_SR_ADDR_BASEL ,SPI_READ, spi_Data,1);
	     switch(W5100_socket0_STATUS)
	     {
			 case SOCK_CLOSED :
				 {
					ITM0_Write("\r\nS0_SOCK_CLOSED \r\n",strlen("\r\nS0_SOCK_CLOSED \r\n"));
					SPI_ETH_PORT_CMD(&ETH,  OPEN );
				 }
			 break;
			 case  SOCK_INIT :
				 {
					ITM0_Write("\r\nS0_SOCK_INIT \r\n",strlen("\r\nS0_SOCK_INIT \r\n"));
					SPI_ETH_PORT_CMD(&ETH,  LISTEN );
				 }
			 break;
			 case SOCK_LISTEN :
				 {
					ITM0_Write("\r\nS0_SOCK_LISTEN \r\n",strlen("\r\nS0_SOCK_LISTEN \r\n"));
				 }
			 break;
			 case SOCK_SYNSENT :
				 {
					ITM0_Write("\r\nS0_SOCK_SYNSENT \r\n",strlen("\r\nS0_SOCK_SYNSENT \r\n"));
				 }
			 break;
			 case SOCK_SYNRECV :
				 {
					ITM0_Write("\r\nS0_SOCK_SYNRECV \r\n",strlen("\r\nS0_SOCK_SYNRECV \r\n"));
				 }
			 break;
			 case SOCK_ESTABLISHED :
				 {
					ITM0_Write("\r\nS0_SOCK_ESTABLISHED \r\n",strlen("\r\nS0_SOCK_ESTABLISHED \r\n"));
					S0_get_size = SPI_ETH_REG(&ETH, S0_RX_SZ_ADDR_BASEHH,S0_RX_SZ_ADDR_BASEHL ,SPI_READ, spi_Data,2);
					if(S0_get_size != 0x00)
					{
						ITM0_Write("\r\nS0_DATA RECEIVED \r\n",strlen("\r\nS0_DATA RECEIVED \r\n"));
						S0_RX_RD = SPI_ETH_REG(&ETH, S0_RX_RD_ADDR_BASEHH,S0_RX_RD_ADDR_BASEHL ,SPI_READ, spi_Data,2);
						S0_get_offset = S0_RX_RD & gS0_RX_MASK;
						S0_get_start_address  = gS0_RX_BASE + S0_get_offset;
						S0_bf_rcv_offset=0;
						if((S0_get_offset  + S0_get_size )>(gS0_RX_MASK + 1))
						{
							upper_size = (gS0_RX_MASK + 1) - S0_get_offset ;
							SPI_ETH_RD_RCV_REG_16(&ETH , S0_get_start_address , SPI_READ , ETH.data , S0_bf_rcv_offset, upper_size);
							destination_addr+=upper_size;
							left_size=S0_get_size-upper_size;
							S0_bf_rcv_offset=upper_size;
							SPI_ETH_RD_RCV_REG_16(&ETH , gS0_RX_BASE , SPI_READ , ETH.data , S0_bf_rcv_offset, left_size);
							S0_mem_pointer=S0_RX_RD + S0_get_size;
						}
						else
						{

							SPI_ETH_RD_RCV_REG_16(&ETH , S0_get_start_address , SPI_READ , ETH.data , S0_bf_rcv_offset, S0_get_size);
							S0_mem_pointer=S0_RX_RD + S0_get_size;
						}
						SPI_ETH_WR_REG_16(&ETH, S0_RX_RD0 , SPI_WRITE, S0_mem_pointer );
						SPI_ETH_PORT_CMD(&ETH, RECV );
						while( SPI_ETH_REG(&ETH, S0_CR_ADDR_BASEH,S0_CR_ADDR_BASEL ,SPI_READ, spi_Data,1))
						{}

					}



				 }
			 break;
			 case SOCK_FIN_WAIT :
				 {
					ITM0_Write("\r\nS0_SOCK_FIN_WAIT \r\n",strlen("\r\nS0_SOCK_FIN_WAIT \r\n"));
				 }
			 break;
			 case SOCK_CLOSING :
				 {
					ITM0_Write("\r\nS0_SOCK_CLOSING \r\n",strlen("\r\nS0_SOCK_CLOSING \r\n"));
				 }
			 break;
			 case  SOCK_TIME_WAIT :
				 {
					ITM0_Write("\r\nS0_SOCK_TIME_WAIT \r\n",strlen("\r\nS0_SOCK_TIME_WAIT \r\n"));
					SPI_ETH_PORT_CMD(&ETH, CLOSE );
				 }
			 break;
			 case SOCK_CLOSE_WAIT :
				 {
					ITM0_Write("\r\nS0_SOCK_CLOSE_WAIT \r\n",strlen("\r\nS0_SOCK_CLOSE_WAIT \r\n"));
					SPI_ETH_PORT_CMD(&ETH, CLOSE );
				 }
			 break;
			 case SOCK_LAST_ACK :
				 {
					ITM0_Write("\r\nS0_SOCK_LAST_ACK \r\n",strlen("\r\nS0_SOCK_LAST_ACK \r\n"));
				 }
			 break;
			 case SOCK_UDP :
				 {
					ITM0_Write("\r\nS0_SOCK_UDP \r\n",strlen("\r\nS0_SOCK_UDP \r\n"));
				 }
			 break;
			 case  SOCK_IPRAW :
				 {
					ITM0_Write("\r\nS0_SOCK_IPRAW \r\n",strlen("\r\nS0_SOCK_IPRAW \r\n"));
				 }
			 break;
			 case  SOCK_MACRAW :
				 {
					ITM0_Write("\r\nS0_SOCK_MACRAW \r\n",strlen("\r\nS0_SOCK_MACRAW \r\n"));
				 }
			 break;
			 case SOCK_PPOE :
				 {
					ITM0_Write("\r\nS0_SOCK_PPOE \r\n",strlen("\r\nS0_SOCK_PPOE \r\n"));
				 }
			 break;

			 default:
				 {

				 }
	     }
	  }
	  }else
	  	  {
		  ETH.operacion=SPI_READ;
		  ETH.TX[3]=0x00;
		  SPI_ETH(&ETH);
	  	  }

*/

