/*
 * ETH_W5100.h
 *
 *  Created on: 18 jul. 2022
 *      Author: mggarcia
 */

//#ifndef ETH_W5100_H_
//#define ETH_W5100_H_
#include "main.h"

// ****** Begin COMMON Registers Address ****** //
enum
{
 MR				=   0x000,	//Mode
 GAR			=   0x001,	//Gateway address
 SUBR			=   0x005,	//Subnet mask
 SHAR			=   0x009,	//Source hardware address
 SIPR			=   0x00F,	//Source IP address
 IR				=   0x015,	//Interrupt
 IMR			=   0x016,	//Interrupt Mask
 RTR			=   0x017,	//Retry time
 RCR			=   0x019,	//Retry count
 RMSR	 		=   0x01A,	//RX Memory size
 TMSR 			=   0x01B,	//TX Memory size
 PAT 			=   0x01C,	//Authentication type in PPPoE
 PPP_PTIMER		=   0x028,	//PPP LCP Request timer
 PPP_PMAGIC		=   0x029,	//PPP LCP ]Magic number
 UIPR			=   0x02A,	//Unreachable IP
 UPORT			=   0x02E,	//Unreachable PORT

 GAR_ADDR_BASEH 		=	0x00,
 GAR_ADDR_BASEL   	 	=	0x01,
 SUBR_ADDR_BASEH 		=	0x00,
 SUBR_ADDR_BASEL   	 	=	0x05,
 SHAR_ADDR_BASEH 		=	0x00,
 SHAR_ADDR_BASEL   	 	=	0x09,
 SIPR_ADDR_BASEH 		=	0x00,
 SIPR_ADDR_BASEL   	 	=	0x0F,
 IR_ADDR_BASEH 		    =	0x00,
 IR_ADDR_BASEL   	 	=	0x15,
 IMR_ADDR_BASEH 		=	0x00,
 IMR_ADDR_BASEL   	 	=	0x16,
 RMSR_ADDR_BASEH 		=	0x00,
 RMSR_ADDR_BASEL   	 	=	0x1A,
 TMSR_ADDR_BASEH 		=	0x00,
 TMSR_ADDR_BASEL   	 	=	0x1B,
};
// ****** End COMMON Registers Address ****** //

// ****** Begin SOCKET Registers Address ****** //
enum
{

 S0_MR			=   0x400,	//Mode
 S0_CR			=   0x401,	//Command
 S0_IR			=   0x402,	//Interrupt
 S0_SR			=   0x403,	//Status
 S0_PORT		=   0x404,	//Source Port
 S0_DHAR		=   0x406,	//Destiantion Address
 S0_DIPR		=   0x40C,	//Destination IP Address
 S0_DPORT		=   0x410,	//Destination Port
 S0_MSSR		=   0x412,	//Maximum segment size
 S0_PROT	 	=   0x414,	//Socket 0 in IP RAW Mode
 S0_TOS 		=   0x415,	//Socket 0 TOS
 S0_TTL 		=   0x416,	//Socket 0 TTL
 S0_TX_FSR		=   0x420,	//Socket 0 TX Free size
 S0_TX_RD		=   0x422,	//Socket 0 TX Read pointer
 S0_TX_WR		=   0x424,	//Socket 0 TX Write pointer
 S0_RX_RSR		=   0x426,	//Socket 0 RX Received size
 S0_RX_RD0      =   0x428,  //Socket 0 RX Read pointer

 S1_MR			=   0x500,	//Mode
 S1_CR			=   0x501,	//Command
 S1_IR			=   0x502,	//Interrupt
 S1_SR			=   0x503,	//Status
 S1_PORT		=   0x504,	//Source Port
 S1_DHAR		=   0x506,	//Destiantion Address
 S1_DIPR		=   0x50C,	//Destination IP Address
 S1_DPORT		=   0x510,	//Destination Port
 S1_MSSR		=   0x512,	//Maximum segment size
 S1_PROT	 	=   0x514,	//Socket 1 in IP RAW Mode
 S1_TOS 		=   0x515,	//Socket 1 TOS
 S1_TTL 		=   0x516,	//Socket 1 TTL
 S1_TX_FSR		=   0x520,	//Socket 1 TX Free size
 S1_TX_RD		=   0x522,	//Socket 1 TX Read pointer
 S1_TX_WR		=   0x524,	//Socket 1 TX Write pointer
 S1_RX_RSR		=   0x526,	//Socket 1 RX Received size
 S1_RX_RD0      =   0x528,  //Socket 1 RX Read pointer

 S2_MR			=   0x600,	//Mode
 S2_CR			=   0x601,	//Command
 S2_IR			=   0x602,	//Interrupt
 S2_SR			=   0x603,	//Status
 S2_PORT		=   0x604,	//Source Port
 S2_DHAR		=   0x606,	//Destiantion Address
 S2_DIPR		=   0x60C,	//Destination IP Address
 S2_DPORT		=   0x610,	//Destination Port
 S2_MSSR		=   0x612,	//Maximum segment size
 S2_PROT	 	=   0x614,	//Socket 2 in IP RAW Mode
 S2_TOS 		=   0x615,	//Socket 2 TOS
 S2_TTL 		=   0x616,	//Socket 2 TTL
 S2_TX_FSR		=   0x620,	//Socket 2 TX Free size
 S2_TX_RD		=   0x622,	//Socket 2 TX Read pointer
 S2_TX_WR		=   0x624,	//Socket 2 TX Write pointer
 S2_RX_RSR		=   0x626,	//Socket 2 RX Received size
 S2_RX_RD0      =   0x628,  //Socket 2 RX Read pointer

 S3_MR			=   0x700,	//Mode
 S3_CR			=   0x701,	//Command
 S3_IR			=   0x702,	//Interrupt
 S3_SR			=   0x703,	//Status
 S3_PORT		=   0x704,	//Source Port
 S3_DHAR		=   0x706,	//Destiantion Address
 S3_DIPR		=   0x70C,	//Destination IP Address
 S3_DPORT		=   0x710,	//Destination Port
 S3_MSSR		=   0x712,	//Maximum segment size
 S3_PROT	 	=   0x714,	//Socket 3 in IP RAW Mode
 S3_TOS 		=   0x715,	//Socket 3 TOS
 S3_TTL 		=   0x716,	//Socket 3 TTL
 S3_TX_FSR		=   0x720,	//Socket 3 TX Free size
 S3_TX_RD		=   0x722,	//Socket 3 TX Read pointer
 S3_TX_WR		=   0x724,	//Socket 3 TX Write pointer
 S3_RX_RSR		=   0x726,	//Socket 3 RX Received size
 S3_RX_RD0      =   0x728,  //Socket 3 RX Read pointer


 S0_MR_ADDR_BASEH 		=	0x04,
 S0_MR_ADDR_BASEL   	=	0x00,
 S0_CR_ADDR_BASEH 		=	0x04,
 S0_CR_ADDR_BASEL   	=	0x01,
 S0_IR_ADDR_BASEH 		=	0x04,
 S0_IR_ADDR_BASEL   	=	0x02,
 S0_SR_ADDR_BASEH 		=	0x04,
 S0_SR_ADDR_BASEL   	=	0x03,
 S0_PORT_ADDR_BASEHH 	=	0x04,
 S0_PORT_ADDR_BASEHL   	=	0x05,
 S0_PORT_ADDR_BASELH 	=	0x04,
 S0_PORT_ADDR_BASELL  	=	0x04,

 S0_RX_SZ_ADDR_BASEHH 	=	0x04,
 S0_RX_SZ_ADDR_BASEHL  	=	0x26,
 S0_RX_SZ_ADDR_BASELH 	=	0x04,
 S0_RX_SZ_ADDR_BASELL  	=	0x27,

 S0_RX_RD_ADDR_BASEHH 	=	0x04, //S0_RX_RD0H
 S0_RX_RD_ADDR_BASEHL  	=	0x28, //S0_RX_RD0L
 S0_RX_RD_ADDR_BASELH 	=	0x04, //S0_RX_RD1H
 S0_RX_RD_ADDR_BASELL  	=	0x29  //S0_RX_RD1L

};
// ****** End SOCKET0  Registers Address ****** //
// ****** Begin Socket COMMANDS Sn_MR ****** //
enum
{
 MODE_CLOSED		=	0x00,
 MODE_TCP 			=	0x01,
 MODE_UDP 			=	0x02,
 MODE_IPRAW  		=	0x04,
 MODE_MACRAW 		=	0x08,
 MODE_PPOE 			=	0x10,
 MODE_ND			=	0x20,
 MODE_MAC_FILTER  	=	0x40,
 MODE_MULTICASTING	=	0x80
};

// ****** end Socket COMMANDS Sn_MR ****** //
// ****** Begin Socket COMMANDS Sn_CR ****** //
enum
{
 OPEN 			=	0x01,
 LISTEN 		=	0x02,
 CONNECT 		=	0x04,
 DISCON  		=	0x08,
 CLOSE 			=	0x10,
 SEND 			=	0x20,
 SEND_MAC		=	0x21,
 SEND_KEEP  	=	0x22,
 RECV 			=	0x40
};

// ****** end Socket COMMANDS Sn_CR ****** //

// ****** Begin Socket STATUS Sn_SR ****** //
enum
{
 SOCK_CLOSED 		=	0x00,
 SOCK_ARP   	 	=	0x01,
 SOCK_INIT			=	0x13,
 SOCK_LISTEN		=	0x14,
 SOCK_SYNSENT 		=	0x15,
 SOCK_SYNRECV		=	0x16,
 SOCK_ESTABLISHED	=	0x17,
 SOCK_FIN_WAIT		=	0x18,
 SOCK_CLOSING  		=	0x1A,
 SOCK_TIME_WAIT		=	0x1B,
 SOCK_CLOSE_WAIT	=	0x1C,
 SOCK_LAST_ACK		=	0x1D,
 SOCK_UDP			=	0x22,
 SOCK_IPRAW			=	0x32,
 SOCK_MACRAW		=	0x42,
 SOCK_PPOE			=	0x5F,
};
// ****** End Socket STATUS Sn_SR ****** //

// Begin Socket SPI
enum
{
 SPI_WRITE			=	0xF0,
 SPI_READ   	 	=	0x0F,
};

struct W5100_SPI
{
	SPI_HandleTypeDef *SPI;			//Hardware SPI tp implement
	GPIO_TypeDef  *NSS_PORT;		//Port for NSS
	uint16_t NSS_PIN;				//Pin number
	uint8_t operacion;				//Define operation read /write


uint16_t	ETH_WDG;
uint8_t     S0_status,
			S1_status,
			S2_status,
			S3_status,
			S0_data_available,
			S0_data_readed,
			TX[4],					//Vector for TX SPI commands
			RX[4],					//Vector for RX SPI commands
			data[2048],				//Data readed from SPI
			swap[2048],				//VECTOR DE INTERCAMBIO A DEFINIR
			GAR[4],
			SUBR[4],
			SHAR[6],
			SIPR[4],
			RMSR,
			TMSR,
			PATR[2],
			PTIMER,
			PMAGIC,
			UIPR[4],
			UPORT[2],

			S0_PORT[2],
			S0_DIPR[4],
			S0_DPORT[2],


			S0_ENserver,
			S1_ENserver,
			S2_ENserver,
			S3_ENserver;


uint16_t    gS0_RX_BASE ,
			gS0_RX_MASK ,
			gS1_RX_BASE ,
			gS1_RX_MASK ,
			gS2_RX_BASE ,
			gS2_RX_MASK ,
			gS3_RX_BASE ,
			gS3_RX_MASK ,

			gS0_TX_BASE ,
			gS0_TX_MASK ,
			gS1_TX_BASE ,
			gS1_TX_MASK ,
			gS2_TX_BASE ,
			gS2_TX_MASK ,
			gS3_TX_BASE ,
			gS3_TX_MASK ;


};




// End Socket SPI
/****************************************************************************
 * Función para el comunicación SPI.
 ****************************************************************************/

uint8_t SPI_ETH(struct W5100_SPI *);
/******************************************************************************
	SPI_ETH Lee o escribe un registro por SPI generando NSS

Primero se debe definir el registro y la operación

estructura.TX[0]=0x0F; for reading 0xF0 for writing ops
estructura.TX[1]= high address ;
estructura.TX[2]= low address;
estructura.TX[3]= data;

function returns readed value

example

raded_value = SPI_ETH(&ETH);

/******************************************************************************/


eth_wr_SOCKET_CMD(struct W5100_SPI *, uint8_t, uint8_t);
/******************************************************************************
	eth_wr_SOCKET_CMD Execute commands on Sn_CR

First define structure to use "instance SPI"
Insert command to execute refer to list "Begin Socket COMMANDS Sn_CR
Insert Socket to apply command

Example:    eth_wr_SOCKET_CMD(&ETH, 0, OPEN );

/******************************************************************************/

eth_wr_SOCKET_MODE(struct W5100_SPI *, uint8_t, uint8_t);
/******************************************************************************
	eth_wr_SOCKET_MODE Execute commands on Sn_MR

First define structure to use "instance SPI"
Insert mode to define refer to list "Begin Socket COMMANDS Sn_MR
Insert Socket to apply mode

Example:    eth_wr_SOCKET_MODE(&ETH, 0, MODE_TCP );

/******************************************************************************/
uint8_t eth_rd_SOCKET_CMD(struct W5100_SPI *, uint8_t);
/******************************************************************************
	eth_rd_SOCKET_CMD Read commands on Sn_CR
After command execution, register is cleared, use this function to check it.
First define structure to use "instance SPI"
Insert Socket number to read command from

Example:    read_command=eth_rd_SOCKET_CMD(&ETH, 0 );

/******************************************************************************/

uint8_t eth_rd_SOCKET_STAT(struct W5100_SPI *, uint8_t);
/******************************************************************************
	eth_rd_SOCKET_STAT Read Socket status REG Sn_SR

First define structure to use "instance SPI"
Insert Socket to apply command

Example:    socket_status=eth_rd_SOCKET_STAT(&ETH, 0 );

/******************************************************************************/

uint16_t eth_rd_SOCKET_DATA(struct W5100_SPI *, uint8_t, uint16_t *, uint16_t);
/******************************************************************************
	eth_rd_SOCKET_DATA Read Socket data received

First define structure to use "instance SPI"
Insert Socket to read
Insert pointer variable that will return read pointer
Insert amount of data to be read
Read data points to struct."data" and struct."qty" for amount of data.

Example:    eth_rd_SOCKET_DATA(&ETH, 0 ,&mem_pointer, data_qty);

/******************************************************************************/

uint16_t eth_wr_SOCKET_DATA(struct W5100_SPI *, uint8_t,uint16_t *, uint16_t);
/******************************************************************************
	eth_wr_SOCKET_DATA Write socket to send data

First define structure to use "instance SPI"
Insert Socket to read
Insert pointer variable that will return read pointer
Insert amount of data to be writen
Read data points to struct."data" and struct."qty" for amount of data.

Example:    eth_wr_SOCKET_DATA(&ETH, 0 ,&data_vector, data_qty);

/******************************************************************************/

uint8_t eth_init(struct W5100_SPI *);
/******************************************************************************
	eth_init Initializes modules

First define structure to use "instance SPI"


Example:    eth_init(&ETH);

/******************************************************************************/
uint8_t eth_socket_init(struct W5100_SPI *, uint8_t);
/******************************************************************************
	eth_socket_init Initializes socket

First define structure to use "instance SPI"
Insert socket number


Example:    eth_socket_init(&ETH, uint8_t);

/******************************************************************************/



uint16_t SPI_ETH_REG(struct W5100_SPI *, uint8_t ,uint8_t ,uint8_t , uint8_t * , uint8_t );
uint16_t SPI_ETH_WR_REG_16(struct W5100_SPI *  , uint16_t  , uint16_t );
uint16_t SPI_ETH_RD_REG_16(struct W5100_SPI * , uint16_t , uint8_t , uint8_t * , uint16_t);
uint16_t SPI_ETH_RD_RCV_REG_16(struct W5100_SPI * , uint16_t, uint8_t * , uint16_t, uint16_t);
uint16_t SPI_ETH_WR_TX_REG_16(struct W5100_SPI * , uint16_t, uint8_t * , uint16_t, uint16_t);
void setVar_ETH(void);
//#endif /* ETH_W5100_H_ */
