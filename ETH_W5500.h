/*
 * ETH_W5500.h
 *
 *  Created on: 18 jul. 2022
 *      Author: mggarcia
 */

//#ifndef ETH_W5500_H_
//#define ETH_W5500_H_
#include "main.h"

// ****** Begin COMMON Registers Address ****** //
enum
{
 MR				=   0x0000,	//Mode
 GAR			=   0x0001,	//Gateway address
 SUBR			=   0x0005,	//Subnet mask
 SHAR			=   0x0009,	//Source hardware address
 SIPR			=   0x000F,	//Source IP address
 ILLT			=   0x0013,	//Interrupt Low Level Timer
 IR				=   0x0015,	//Interrupt
 IMR			=   0x0016,	//Interrupt Mask
 SIR			=   0x0017,	//Socket Interrupt
 SIMR			=   0x0018,	//Socket Interrupt Mask
 RTR			=   0x0019,	//Retry time
 RCR			=   0x001B,	//Retry count
 PTIMER			=   0x001C,	//PPP LCP Request timer
 PMAGIC			=   0x001D,	//PPP LCP Magic number
 PHAR			= 	0x001E,	//PPP Destination MAC Address
 PSID			= 	0x0024,	//PPP Session ID
 PMRU			= 	0x0026,	//PPP Max Segment Size
 UIPR_			= 	0x0028,	//Unreachable IP Address
 UPORTR			= 	0x002C,	//Unreachable Port
 PHYCFGR 		=   0x002E,	//PHY Configuration
 VERSIONR		=   0x0039,	//Chip version
/*
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
 IMR_ADDR_BASEL   	 	=	0x16,*/

};
// ****** End COMMON Registers Address ****** //

// ****** Begin Selection Registers Address ****** //
enum{

	COMM_REG	= 0x00,
	S0_REG		= 0x01,
	S0_TX_BUFF	= 0x02,
	S0_RX_BUFF	= 0x03,
	S1_REG		= 0x05,
	S1_TX_BUFF	= 0x06,
	S1_RX_BUFF	= 0x07,
	S2_REG		= 0x09,
	S2_TX_BUFF	= 0x0A,
	S2_RX_BUFF	= 0x0B,
	S3_REG		= 0x0D,
	S3_TX_BUFF	= 0x0E,
	S3_RX_BUFF	= 0x0F,
	S4_REG		= 0x11,
	S4_TX_BUFF	= 0x12,
	S4_RX_BUFF	= 0x13,
	S5_REG		= 0x15,
	S5_TX_BUFF	= 0x16,
	S5_RX_BUFF	= 0x17,
	S6_REG		= 0x19,
	S6_TX_BUFF	= 0x1A,
	S6_RX_BUFF	= 0x1B,
	S7_REG		= 0x1D,
	S7_TX_BUFF	= 0x1E,
	S7_RX_BUFF	= 0x1F,

};

// ****** End Selection Registers Address ****** //

// ****** Begin SOCKET Registers Address ****** //
enum
{

 Sn_MR			=   0x0000,	//Mode
 Sn_CR			=   0x0001,	//Command
 Sn_IR			=   0x0002,	//Interrupt
 Sn_SR			=   0x0003,	//Status
 Sn_PORT		=   0x0004,	//Source Port
 Sn_DHAR		=   0x0006,	//Destiantion Address
 Sn_DIPR		=   0x000C,	//Destination IP Address
 Sn_DPORT		=   0x0010,	//Destination Port
 Sn_MSSR		=   0x0012,	//Maximum segment size
 Sn_TOS 		=   0x0015,	//Socket  TOS
 Sn_TTL 		=   0x0016,	//Socket  TTL
 Sn_RXBUF_SIZE	=	0x001E,	//Socket Resceive Buffer
 Sn_TXBUF_SIZE	=	0x001F,	//Socket Transmitt Buffer
 Sn_TX_FSR		=   0x0020,	//Socket  TX Free size
 Sn_TX_RD		=   0x0022,	//Socket  TX Read pointer
 Sn_TX_WR		=   0x0024,	//Socket  TX Write pointer
 Sn_RX_RSR		=   0x0026,	//Socket  RX Received size
 Sn_RX_RD0     	=   0x0028, //Socket  RX Read pointer
 Sn_RX_WR		=   0x002A,	//Socket  RX Write Pointer
 Sn_IMR			=   0x002C,	//Socket  Interrupt Mask
 Sn_FRAG		=   0x002D,	//Socket  Fragment Offset in IP Header
 Sn_KPALVTR     =   0x002F, //Socket  Keep Alive Timer


 S_MR_ADDR_BASEH 		=	0x00,
 S_MR_ADDR_BASEL   		=	0x00,
 S_CR_ADDR_BASEH 		=	0x00,
 S_CR_ADDR_BASEL   		=	0x01,
 S_IR_ADDR_BASEH 		=	0x00,
 S_IR_ADDR_BASEL   		=	0x02,
 S_SR_ADDR_BASEH 		=	0x00,
 S_SR_ADDR_BASEL   		=	0x03,
 S_PORT_ADDR_BASEHH 	=	0x00,
 S_PORT_ADDR_BASEHL   	=	0x05,
 S_PORT_ADDR_BASELH 	=	0x00,
 S_PORT_ADDR_BASELL  	=	0x04,

 S_RX_SZ_ADDR_BASEHH 	=	0x00,
 S_RX_SZ_ADDR_BASEHL  	=	0x26,
 S_RX_SZ_ADDR_BASELH 	=	0x00,
 S_RX_SZ_ADDR_BASELL  	=	0x27,

 S_RX_RD_ADDR_BASEHH 	=	0x00, //S_RX_RD0H
 S_RX_RD_ADDR_BASEHL  	=	0x28, //S_RX_RD0L
 S_RX_RD_ADDR_BASELH 	=	0x00, //S_RX_RD1H
 S_RX_RD_ADDR_BASELL  	=	0x29  //S_RX_RD1L

};
// ****** End SOCKET0  Registers Address ****** //
// ****** Begin Socket COMMANDS Sn_MR ****** //
enum
{
 MODE_CLOSED			=	0x00,
 MODE_TCP				=	0x01,
 MODE_UDP 				=	0x02,
 MODE_IPRAW  			=	0x04,
 MODE_MACRAW 			=	0x08,
 MODE_PPOE 				=	0x10,
 MODE_ND				=	0x20,
 MODE_MAC_FILTER  		=	0x40,
 MODE_MULTICASTING		=	0x80
};

// ****** end Socket COMMANDS Sn_MR ****** //
// ****** Begin Socket COMMANDS Sn_CR ****** //
enum
{
 OPEN 			=	0x01,
 LISTEN 		=	0x02,
 CONNECT 		=	0x04,
 DISCON  		=	0x08,
 CLOSE			=	0x10,
 SEND 			=	0x20,
 SEND_MAC		=	0x21,
 SEND_KEEP  	=	0x22,
 RECV 			=	0x40
};




// ****** end Socket COMMANDS Sn_CR ****** //

// ****** Begin Socket COMMANDS Status for programm ****** //
enum
{
 READY 				=	0x00,
 TIME_OUT			=	0X01,
 CONNECTING 		=	0x02
};

// ****** end Socket COMMANDS Status for programm ****** //

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


// ****** Begin Socket STATUS STATE MACHINE****** //
enum
{ 
NO_OP		   = 	0x00,
IDLE           = 	0x01,
SENT           = 	0x02,
SENDING		   =	0x03,
RECEIPT		   =	0x04,
RECEIVING	   =	0x05,
};
// ****** End Socket STATUS STATE MACHINE ****** //


// Begin Socket SPI
enum
{
 SPI_WRITE			=	1,
 SPI_READ   	 	=	0,
};

struct W5500_SPI
{
	SPI_HandleTypeDef *SPI;			//Hardware SPI tp implement
	GPIO_TypeDef  *NSS_PORT;		//Port for NSS
	uint16_t NSS_PIN;				//Pin number
	GPIO_TypeDef  *RST_PORT;		//Port for NSS
	uint16_t RST_PIN;				//Pin number
	uint8_t operacion;				//Define operation read /write

uint16_t	ETH_WDG,
			ETH_CMD_WDG;
uint8_t     CMD_Status,
			PHY_status,
			S_status,
			S1_status,
			S2_status,
			S3_status,
			S_data_available,
			S_data_readed,
			TX[4],					//Vector for TX SPI commands
			RX[4],					//Vector for RX SPI commands
			data[2048],				//Data readed from SPI
			swap[2048],				//VECTOR DE INTERCAMBIO A DEFINIR
			TX_data[2048],
			spi_Data[64],
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

			S_PORT[2],
			S_DIPR[4],
			S_DPORT[2],

			S_ENserver,
			S1_ENserver,
			S2_ENserver,
			S3_ENserver,
			T8,
			DBG,
			CAM,
			STATUS,
			COMMAND;

uint16_t    gS_RX_BASE ,
			gS_RX_MASK ,
			gS1_RX_BASE ,
			gS1_RX_MASK ,
			gS2_RX_BASE ,
			gS2_RX_MASK ,
			gS3_RX_BASE ,
			gS3_RX_MASK ,

			gS_TX_BASE ,
			gS_TX_MASK ,
			gS1_TX_BASE ,
			gS1_TX_MASK ,
			gS2_TX_BASE ,
			gS2_TX_MASK ,
			gS3_TX_BASE ,
			gS3_TX_MASK ,
			T16,
			tx_mem_pointer,
			rx_mem_pointer,
			S0_get_size,
			send_size;

uint32_t    MB_TOUT_ticks;
};

struct W5500_SPY
{
uint8_t	MR,
		GAR[4],
		SUBR[4],
		SHAR[6],
		SIPR[4],
		INTLEVEL[2],
		IR,IMR,SIR,SIMR,
		RTR[2],
		RCR, PTIMER, PMAGIC,
		PHAR[6],
		PSID[2],
		PMRU[2],
		UIPR[4],
		UPORT[2],
		PHYCFGR,VERSIONR;

uint8_t	Sn_MR,
		Sn_CR,
		Sn_IR,
		Sn_SR,
		Sn_PORT[2],
		Sn_DHAR[6],
		Sn_DIPR[4],
		Sn_DPORT[2],
		Sn_MSSR[2],
		Sn_TOS,
		Sn_TTL,
		Sn_RXBUF_SIZE,
		Sn_TXBUF_SIZE,
		Sn_TX_FSR[2],
		Sn_TX_RD[2],
		Sn_TX_WR[2],
		Sn_RX_RSR[2],
		Sn_RX_RD[2],
		Sn_RX_WR[2],
		Sn_IMR,
		Sn_FRAG[2],
		Sn_KPALVTR,
		RXREG[2048],
		TXREG[2048],
		DUMMY[2];
};
// End Socket SPI
/****************************************************************************
 * Función para el comunicación SPI.
 ****************************************************************************/

uint8_t SPI_ETH(struct W5500_SPI *);
/******************************************************************************
	SPI_ETH Lee o escribe un registro por SPI generando NSS

Primero se debe definir el registro y la operación

estructura.TX[0]= High offset address;
estructura.TX[1]= Low offset address ;
estructura.TX[2]= Control phase;  (5 bits reg selection + R/Wneg + operation mode =00)
estructura.TX[3]= data;

function returns readed value

example

raded_value = SPI_ETH(&ETH);

/******************************************************************************/
SPI_ETH_RESET(struct W5500_SPI * x);
/******************************************************************************
	SPI_ETH_RESET Resets module by hardware

First define structure to use "instance SPI"
Port and pin of RST must be defined

Example:    SPI_ETH_RESET(&ETH );

/******************************************************************************/

SPI_ETH_PHY_RESET(struct W5500_SPI * x);
/******************************************************************************
	SPI_ETH_PHY_RESET Resets module by hardware

First define structure to use "instance SPI"


Example:    SPI_ETH_PHY_RESET(&ETH );

/******************************************************************************/

eth_wr_SOCKET_CMD(struct W5500_SPI *, uint8_t, uint8_t);
/******************************************************************************
	eth_wr_SOCKET_CMD Execute commands on Sn_CR

First define structure to use "instance SPI"
Insert command to execute refer to list "Begin Socket COMMANDS Sn_CR
Insert Socket to apply command

Example:    eth_wr_SOCKET_CMD(&ETH, 0, OPEN );

/******************************************************************************/

eth_wr_SOCKET_MODE(struct W5500_SPI *, uint8_t, uint8_t);
/******************************************************************************
	eth_wr_SOCKET_MODE Execute commands on Sn_MR

First define structure to use "instance SPI"
Insert mode to define refer to list "Begin Socket COMMANDS Sn_MR
Insert Socket to apply mode

Example:    eth_wr_SOCKET_MODE(&ETH, 0, MODE_TCP );

/******************************************************************************/
uint8_t eth_rd_SOCKET_CMD(struct W5500_SPI *, uint8_t);
/******************************************************************************
	eth_rd_SOCKET_CMD Read commands on Sn_CR
After command execution, register is cleared, use this function to check it.
First define structure to use "instance SPI"
Insert Socket number to read command from

Example:    read_command=eth_rd_SOCKET_CMD(&ETH, 0 );

/******************************************************************************/

uint8_t eth_rd_SOCKET_STAT(struct W5500_SPI *, uint8_t);
/******************************************************************************
	eth_rd_SOCKET_STAT Read Socket status REG Sn_SR

First define structure to use "instance SPI"
Insert Socket to apply command

Example:    socket_status=eth_rd_SOCKET_STAT(&ETH, 0 );

/******************************************************************************/

uint16_t eth_rd_SOCKET_DATA(struct W5500_SPI *, uint8_t, uint16_t *, uint16_t);
/******************************************************************************
	eth_rd_SOCKET_DATA Read Socket data received

First define structure to use "instance SPI"
Insert Socket to read
Insert pointer variable that will return read pointer
Insert amount of data to be read
Read data points to struct."data" and struct."qty" for amount of data.

Example:    eth_rd_SOCKET_DATA(&ETH, 0 ,&mem_pointer, data_qty);

/******************************************************************************/

uint16_t eth_wr_SOCKET_DATA(struct W5500_SPI *, uint8_t, uint16_t *);
/******************************************************************************
	eth_wr_SOCKET_DATA Write socket to send data

First define structure to use "instance SPI"
Insert Socket to read
Insert pointer variable that will return read pointer
Insert amount of data to be writen
Read data points to struct."data" and struct."qty" for amount of data.

Example:    eth_wr_SOCKET_DATA(&ETH, 0 ,&data_vector, data_qty);

/******************************************************************************/

uint8_t eth_init(struct W5500_SPI *);
/******************************************************************************
	eth_init Initializes modules

First define structure to use "instance SPI"


Example:    eth_init(&ETH);

/******************************************************************************/
uint8_t eth_socket_init(struct W5500_SPI *, uint8_t);
/******************************************************************************
	eth_socket_init Initializes socket

First define structure to use "instance SPI"
Insert socket number


Example:    eth_socket_init(&ETH, uint8_t);

/******************************************************************************/


/******************************************************************************/
uint16_t SPI_ETH_REG(struct W5500_SPI *, uint16_t ,uint8_t ,uint8_t , uint8_t * , uint8_t );
/******************************************************************************
	SPI_ETH_REG Read / Write register

Read: Reads 8 bit or 16bit registers (length = 1 8bit, length = 2 16bit)

First define structure to use "instance SPI"
Insert socket number
Address Phase
Control Phase
Data
Data length


Example:    eth_socket_init(&ETH, uint16_t, uint8_t, uint8_t, uint8_t *, uint8_t);

/******************************************************************************/
uint16_t SPI_ETH_WR_REG_16(struct W5500_SPI *  , uint16_t  , uint16_t , uint8_t);
uint16_t SPI_ETH_RD_REG_16(struct W5500_SPI * , uint16_t , uint8_t , uint8_t * , uint16_t, uint8_t);
uint16_t SPI_ETH_RD_RCV_REG_16(struct W5500_SPI * , uint16_t, uint8_t * , uint16_t, uint16_t, uint8_t);
uint16_t SPI_ETH_WR_TX_REG_16(struct W5500_SPI * , uint16_t, uint8_t * , uint16_t, uint16_t, uint8_t);
void setVar_ETH(void);
uint8_t SPI_ETH_SNIFF(struct W5500_SPY *,struct W5500_SPI *);

//#endif /* ETH_W5500_H_ */
