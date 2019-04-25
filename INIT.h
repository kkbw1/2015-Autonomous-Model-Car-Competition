#include "MPC5606B.h"

#define _INIT_H_
//////////////////////////////////////////////////////////////////////////////////////////////////
#define LINIBRR_9600 	416		// 416.6666666666
#define LINFBRR_9600	11		// 10.6666666666

#define LINIBRR_14400 	277		// 277.7777777777
#define LINFBRR_14400	12		// 12.444444444444

#define LINIBRR_19200 	208		
#define LINFBRR_19200	5

#define LINIBRR_38400 104		// 104.166666666666
#define LINFBRR_38400	3		// 2.666666666666

#define LINIBRR_57600	
#define LINFBRR_57600	

#define LINIBRR_115200 	34
#define LINFBRR_115200	12

#define LINIBRR_128000 	31		// 31.25
#define LINFBRR_128000	4		// 4 

#define LINIBRR_230400 	17		// 31.25
#define LINFBRR_230400	6		// 4 

#define BCR	0x0D
#define BLF	0x0A

#define UART_POLL		0
#define UART_CHK		1
//////////////////////////////////////////////////////////////////////////////////////////////////
#define SDA 		10
#define SCL 		11

#define I2C_ACK 	0
#define I2C_NACK	1
#define	I2C_W		0
#define I2C_R		1

#define SDA_LOW		SIU.GPDO[SDA].B.PDO = 0
#define SDA_HIGH	SIU.GPDO[SDA].B.PDO = 1
#define SCL_LOW		SIU.GPDO[SCL].B.PDO = 0
#define SCL_HIGH	SIU.GPDO[SCL].B.PDO = 1
//////////////////////////////////////////////////////////////////////////////////////////////////

// Delay fucntion
void Delay(uint32_t count);
// Init
void initModesAndClock(void);
void initPeriClkGen(void);
void disableWatchdog(void);
void initPads (void);
// ADC
void initADC0(void);
void initADC1(void);
// PIT
void initPIT(int8_t Channel, uint32_t Clock_Freq, double ExpectedTimeBase);
// EMIOS
void initEMIOS_0(void);
void initEMIOS_0_MCB(uint8_t u8Channel, uint16_t u16Period);
void initEMIOS_0_OPWM(uint8_t u8Channel, uint8_t u8CntBus, uint16_t u16Duty);
void initEMIOS_0_IPWM(uint8_t u8Channel, uint8_t u8CntBus, uint8_t u8Pol);
void EMIOS_0_OPWM_Duty(uint8_t u8Channel, uint16_t u16Duty);
// UART
void initLINFlex_0(void);
void UART0_putChar(uint8_t ch);
void initLINFlex_1(void);
void UART1_putChar(uint8_t ch);
void UART1_putStr(char *str);
char UART1_getChar(unsigned char mode);
// DSPI 
void initDSPI(void);
unsigned char writeDataDSPI_0(unsigned char data, unsigned char cs, unsigned char cont_cs);
// I2C
void initI2C(void);
void I2C_Delay(void);
void I2C_Master_Start(void);
void I2C_Master_Stop(void);
uint8_t I2C_Master_TX(uint8_t _data);
uint8_t I2C_Master_RX(uint8_t _ack);
//Interrupt
void enableIrq(void);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Delay(uint32_t count)
{
	volatile uint32_t dly;
	
	for(dly=0;dly<count;dly++) {}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initModesAndClock(void) 
{
	ME.MER.R = 0x0000001D;          	/* Enable DRUN, RUN0, SAFE, RESET modes */
	
	/* Initialize PLL before turning it on: */
	/* Use 1 of the next 2 lines depending on crystal frequency: */
	CGM.FMPLL_CR.R = 0x02400100;    	/* 8 MHz xtal: Set PLL0 to 64 MHz */   

	ME.RUN[0].R = 0x001F0074;       	/* RUN0 cfg: 16MHzIRCON,OSC0ON,PLL0ON,syclk=PLL */
	ME.RUNPC[1].R = 0x000000F0; 	  	/* Peri. Cfg. 1 settings: only run in RUN0 mode */
	
	ME.PCTL[4].R = 0x01;            	/* MPC56xxB/P/S DSPI0:  select ME.RUNPC[1] */	
	ME.PCTL[32].R = 0x01;       		/* MPC56xxB ADC 0: 			select ME.RUNPC[1] */
	ME.PCTL[44].R = 0x01;				// I2C
	ME.PCTL[48].R = 0x01;           	/* MPC56xxB/P/S LINFlex 0: 	select ME.RUNPC[1] */
	ME.PCTL[49].R = 0x01;           	/* MPC56xxB/P/S LINFlex 1: 	select ME.RUNPC[1] */
	ME.PCTL[68].R = 0x01;           	/* MPC56xxB/S SIUL:  		select ME.RUNPC[1] */
	ME.PCTL[72].R = 0x01;           	/* MPC56xxB/S EMIOS 0:  	select ME.RUNPC[1] */
	ME.PCTL[92].R = 0x01;           	/* PIT, RTI: 				select ME.RUNPC[1] */	  
	
	/* Mode Transition to enter RUN0 mode: */
	ME.MCTL.R = 0x40005AF0;         	/* Enter RUN0 Mode & Key */
	ME.MCTL.R = 0x4000A50F;         	/* Enter RUN0 Mode & Inverted Key */  
	while (ME.GS.B.S_MTRANS) {}     	/* Wait for mode transition to complete */    
	/* Note: could wait here using timer and/or I_TC IRQ */
	while(ME.GS.B.S_CURRENTMODE != 4) {} /* Verify RUN0 is the current mode */
}

void initPeriClkGen(void) 
{
	CGM.SC_DC0.R = 0x80;         		/* MPC56xxB: Enable peri set 1 sysclk divided by 1 */
	CGM.SC_DC1.R = 0x80;         		/* MPC56xxB: Enable peri set 2 sysclk divided by 1 */
	CGM.SC_DC2.R = 0x80;         		/* MPC56xxB: Enable peri set 3 sysclk divided by 1 */
}

void disableWatchdog(void) 
{
	SWT.SR.R = 0x0000c520;     			/* Write keys to clear soft lock bit */
	SWT.SR.R = 0x0000d928; 
	SWT.CR.R = 0x8000010A;     			/* Clear watchdog enable (WEN) */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initPads (void) 
{
	//PORTA	0~15
	SIU.PCR[0].B.OBE = 1;	// PA0 CLK1
	SIU.PCR[1].B.OBE = 1;	// PA1 SI1
	SIU.PCR[2].B.OBE = 1;	// PA2 CLK2
	SIU.PCR[3].B.OBE = 1;	// PA3 SI2
	
	SIU.PCR[4].B.PA = 0x1;	// PA4 Steering Servo
	
	SIU.PCR[6].B.IBE = 1;
	SIU.PCR[6].B.WPE = 1;
	SIU.PCR[6].B.WPS = 1;
	
	SIU.PCR[12].R = 0x0103; /* MPC56xxB: Config pad PA12 as DSPI_0 MISO input */
	SIU.PCR[13].R = 0x0604; /* MPC56xxB: Config pad PA13 as DSPI_0 MOSI output */
	SIU.PCR[14].R = 0x0604; /* MPC56xxB: Config pad PA14 as DSPI_0 SCK output */
	
	SIU.PCR[15].B.PA = 1;   /* MPC56xxB: Config pad PA15 as DSPI_0 CS0_0 output */
	SIU.PCR[28].B.PA = 3;   /* MPC56xxB: Config pad PB12 as DSPI_0 CS1_0 output */
	SIU.PCR[29].B.PA = 3;   /* MPC56xxB: Config pad PB13 as DSPI_0 CS2_0 output */	
	
	// PORTB 16~31
	SIU.PCR[20].B.APC = 1;	// PB4  ADC0_0_P  POT
	SIU.PCR[21].B.APC = 1;	// PB5  ADC0_1_P  
	SIU.PCR[22].B.APC = 1;	// PB6  ADC0_2_P  Cam1
	SIU.PCR[23].B.APC = 1;	// PB7  ADC0_3_P  Cam2
	
	// PORTC 32~47
	
	// PORTD 48~63     
	SIU.PCR[48].B.APC = 1;	// PD0 ADC0_4_P  IR1
	SIU.PCR[49].B.APC = 1;	// PD1 ADC0_5_P  IR2
	SIU.PCR[50].B.APC = 1;	// PD2 ADC0_6_P  IR3
	SIU.PCR[51].B.APC = 1;	// PD3 ADC0_7_P  IR4
	
	SIU.PCR[52].B.IBE = 1;	// PD4 External Switch
	SIU.PCR[53].B.IBE = 1;	// PD5
	SIU.PCR[54].B.IBE = 1;	// PD6
	SIU.PCR[55].B.IBE = 1;	// PD7
	SIU.PCR[56].B.IBE = 1;	// PD8
	SIU.PCR[57].B.IBE = 1;	// PD9
	SIU.PCR[58].B.IBE = 1;	// PD10
	SIU.PCR[59].B.IBE = 1;	// PD11
	
	// PORTE 64~79
	SIU.PCR[64].B.IBE = 1;	// PE0 Internal Switch			
	SIU.PCR[65].B.IBE = 1;				
	SIU.PCR[66].B.IBE = 1;				
	SIU.PCR[67].B.IBE = 1;
	SIU.PCR[68].B.OBE = 1;	// PE4 Internal LED			
	SIU.PCR[69].B.OBE = 1;				
	SIU.PCR[70].B.OBE = 1;				
	SIU.PCR[71].B.OBE = 1;		
	
	// PORTF 80~95
	SIU.PCR[80].B.OBE = 1;	// PF0 DC Enable
	
	// PORTG 96~111
	SIU.PCR[96].B.OBE = 1; 	// PG0 RS LCD
	SIU.PCR[97].B.OBE = 1;	// PG1 E
	SIU.PCR[98].B.OBE = 1;	// PG2 4
	SIU.PCR[99].B.OBE = 1;	// PG3 5
	SIU.PCR[100].B.OBE = 1;	// PG4 6
	SIU.PCR[101].B.OBE = 1;	// PG5 7
	
	SIU.PCR[102].B.IBE = 1;	// PG6 Internal Toggle
	SIU.PCR[103].B.IBE = 1;
	SIU.PCR[104].B.IBE = 1;
	SIU.PCR[105].B.IBE = 1;
	
	SIU.PCR[106].B.PA = 0x1; // PG10 DC1 PWM 
	SIU.PCR[107].B.PA = 0x1; // PG11 DC1 PWM
	SIU.PCR[108].B.PA = 0x1; // PG12 DC2 PWM
	SIU.PCR[109].B.PA = 0x1; // PG13 DC2 PWM
	
	// PORTH 112~127
	
	
	// PORTI 128~143
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initADC0(void) 
{	
	ADC_0.MCR.R = 0x80000000;         				/* Initialize ADC0, Overwritten Enable, One shot mode */ 
	
	//ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 0);	/* Channel 0 */
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 2);	/* Channel 2 */
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 3);	/* Channel 3 */
	
	ADC_0.CTR0.R = 0x00008606;       				/* precision channel Conversion times for 32MHz AD Clock */
}

void initADC1(void) 
{	
	ADC_1.MCR.R = 0x00000000;         				/* Initialize ADC0 for one shot mode */ 

	ADC_1.NCMR0.R |= (uint32_t)(0x00000001 << 0);	/* Channel 0 */
	ADC_1.NCMR0.R |= (uint32_t)(0x00000001 << 2);	/* Channel 2 */
	ADC_1.NCMR0.R |= (uint32_t)(0x00000001 << 3);	/* Channel 3 */
	
	ADC_1.CTR0.R = 0x00008606;       				/* Conversion times for 32MHz AD Clock */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initPIT(int8_t Channel, uint32_t Clock_Freq, double ExpectedTimeBase)  
{
	PIT.PITMCR.R = 0x00000001;       				/* Enable PIT and configure timers to stop in debug mode */
	PIT.CH[Channel].LDVAL.R = (uint32_t) (ExpectedTimeBase*Clock_Freq-1);	// Fix the load value for CH[0]
	PIT.CH[Channel].TCTRL.R = 0x000000003; 			/* Enable PIT interrupt and make PIT active to count */ 
}

void initEMIOS_0(void)
{
	EMIOS_0.MCR.B.GPRE= 63;   							/* Divide 64 MHz sysclk by 63+1 = 64 for 1MHz eMIOS clk*/
  	EMIOS_0.MCR.B.GTBE = 1;   							/* Enable global time base */
  	EMIOS_0.MCR.B.GPREN = 1;  							/* Enable eMIOS clock, start counting */
  	EMIOS_0.MCR.B.FRZ = 1;    							/* Enable stopping channels when in debug mode */	
}

void initEMIOS_0_MCB(uint8_t u8Channel, uint16_t u16Period)
{
  	EMIOS_0.CH[u8Channel].CADR.R      = u16Period;	    /* Period will be u16Period clocks (usec) */
  	EMIOS_0.CH[u8Channel].CCR.B.MODE  = 0x50; 			/* Set as Modulus Up Counter Buffered (MCB) */
  	EMIOS_0.CH[u8Channel].CCR.B.BSL   = 0x3;			/* Use Internal Counter */
 	EMIOS_0.CH[u8Channel].CCR.B.UCPRE = 0;    			/* Set channel prescaler divide by 1 (to 1Mhz) */
  	EMIOS_0.CH[u8Channel].CCR.B.UCPEN = 1;    			/* Enable prescaler*/
  	EMIOS_0.CH[u8Channel].CCR.B.FREN  = 1; 				/* Freeze channel counting when in debug mode */	
}

void initEMIOS_0_OPWM(uint8_t u8Channel, uint8_t u8CntBus, uint16_t u16Duty)
{
  	EMIOS_0.CH[u8Channel].CADR.R = 0;        			/* Leading edge when channel counter bus=u16A */
  	EMIOS_0.CH[u8Channel].CBDR.R = u16Duty;        		/* Trailing edge when channel's counter bus=u16B */
  	EMIOS_0.CH[u8Channel].CCR.B.BSL = u8CntBus;			/* Use counter bus A,B,C,D,or E */
  	EMIOS_0.CH[u8Channel].CCR.B.EDPOL = 1;				/* Polarity-leading edge sets output/trailing clears */
  	EMIOS_0.CH[u8Channel].CCR.B.MODE = 0x60; 			/* Mode is OPWM Buffered */	
}

void initEMIOS_0_IPWM(uint8_t u8Channel, uint8_t u8CntBus, uint8_t u8Pol)
{
	// EDPOL => 1 is Trigger on a rising edge
	// EDSEL => 1 is Both edge triggering
	// EDSEL => 0 is Single Edge defined by the EDPOL bit
	EMIOS_0.CH[u8Channel].CCR.B.BSL = u8CntBus;			/* Use counter bus A,B,C,D,or E */
	EMIOS_0.CH[u8Channel].CCR.B.EDPOL = u8Pol;			/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[u8Channel].CCR.B.EDSEL = 1;				/* Single edges triggering*/
//	EMIOS_0.CH[u8Channel].CCR.B.FCK = 1;	    		/* Input filter bit clock reference is the system clock*/
//	EMIOS_0.CH[u8Channel].CCR.B.IF = 1;				    /* Input filter bit of 2 clock cycles*/ 
	EMIOS_0.CH[u8Channel].CCR.B.MODE = 0b0000100; 		/* Mode is IPWM Input Pulse Width Measurement */
	EMIOS_0.CH[u8Channel].CCR.B.FREN = 1;	    		/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[u8Channel].CCR.B.DMA = 0;	  			/* Flag/overrun assigned to interrupt request instead of CTU*/
	EMIOS_0.CH[u8Channel].CCR.B.FEN = 0;	    		/* Enables Unified Channel FLAG bit to generate an interrupt signal*/
}  

void EMIOS_0_OPWM_Duty(uint8_t u8Channel, uint16_t u16Duty)
{
	EMIOS_0.CH[u8Channel].CBDR.R = u16Duty;        		/* Trailing edge when channel's counter bus=u16B */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initLINFlex_0(void)
{
	/* enter INIT mode */
	LINFLEX_0.LINCR1.R = 0x0081; 			/* SLEEP=0, INIT=1 */
	
	/* wait for the INIT mode */
	while (0x1000 != (LINFLEX_0.LINSR.R & 0xF000)) {}	

	SIU.PCR[18].R = 0x0604;     			/* Configure pad PB2 for AF1 func: LIN0TX */
	SIU.PCR[19].R = 0x0100;     			/* Configure pad PB3 for LIN0RX */
	
	/* configure for UART mode */
	LINFLEX_0.UARTCR.R = 0x0001; 			/* set the UART bit first to be able to write the other bits */
	LINFLEX_0.UARTCR.R = 0x0033; 			/* 8bit data, no parity, Tx and Rx enabled, UART mode */
								 			/* Transmit buffer size = 1 (TDFL = 0 */
								 			/* Receive buffer size = 1 (RDFL = 0) */
	
	LINFLEX_0.LINFBRR.R = LINFBRR_230400;
	LINFLEX_0.LINIBRR.R = LINIBRR_230400;
	
	LINFLEX_0.LINIER.R = 0x0004; 			// Data Reception Interrupt Enable 
	
	/* enter NORMAL mode */
	LINFLEX_0.LINCR1.R = 0x0080; 			/* INIT=0 */
}

void UART0_putChar(uint8_t ch)
{
	LINFLEX_0.BDRL.B.DATA0 = ch;  			/* write character to transmit buffer */
	while (1 != LINFLEX_0.UARTSR.B.DTF) {}  /* Wait for data transmission completed flag */
	LINFLEX_0.UARTSR.B.DTF = 1;				/* clear the DTF flag and not the other flags */
}

void initLINFlex_1(void)
{
	/* enter INIT mode */
	LINFLEX_1.LINCR1.R = 0x0081; 		/* SLEEP=0, INIT=1 */
	
	/* wait for the INIT mode */
	while (0x1000 != (LINFLEX_1.LINSR.R & 0xF000)) {}
	
	/* configure pads */
	SIU.PCR[38].R = 0x0604;     		/* Configure pad PC6 for AF1 func: LIN1TX */
	SIU.PCR[39].R = 0x0100;     		/* Configure pad PC7 for LIN1RX */	
	
	/* configure for UART mode */
	LINFLEX_1.UARTCR.R = 0x0001; 		/* set the UART bit first to be able to write the other bits */
	LINFLEX_1.UARTCR.R = 0x0033; 		/* 8bit data, no parity, Tx and Rx enabled, UART mode */
								 		/* Transmit buffer size = 1 (TDFL = 0 */
								 		/* Receive buffer size = 1 (RDFL = 0) */
	
	LINFLEX_1.LINFBRR.R = LINFBRR_115200;
	LINFLEX_1.LINIBRR.R = LINIBRR_115200;
	
	LINFLEX_1.LINIER.R = 0x0004; 		// Data Reception Interrupt Enable 
	
	/* enter NORMAL mode */
	LINFLEX_1.LINCR1.R = 0x0080; 		/* INIT=0 */
}

char UART1_getChar(unsigned char mode)
{
	uint8_t ch;
	
	if(mode == UART_CHK)
	{
		if(1 == LINFLEX_1.UARTSR.B.DRF && 1 == LINFLEX_1.UARTSR.B.RMB)
		{
			/* get the data */
			ch = (uint8_t)LINFLEX_1.BDRM.B.DATA4;
			/* clear the DRF and RMB flags by writing 1 to them */
			LINFLEX_1.UARTSR.B.DRF = 1;
			LINFLEX_1.UARTSR.B.RMB = 1;			
		}
		else
		{
			ch = 0;
		}
	}
	else if(mode == UART_POLL)
	{
		/* wait for DRF */
		while (1 != LINFLEX_1.UARTSR.B.DRF) {}  /* Wait for data reception completed flag */
		/* wait for RMB */
		while (1 != LINFLEX_1.UARTSR.B.RMB) {}  /* Wait for Release Message Buffer */
		
		/* get the data */
		ch = (uint8_t)LINFLEX_1.BDRM.B.DATA4;
		
		/* clear the DRF and RMB flags by writing 1 to them */
		LINFLEX_1.UARTSR.B.DRF = 1;
		LINFLEX_1.UARTSR.B.RMB = 1;	
	}
	
	return ch;
}

void UART1_putChar(uint8_t ch)
{
	LINFLEX_1.BDRL.B.DATA0 = ch;  			/* write character to transmit buffer */
	while (1 != LINFLEX_1.UARTSR.B.DTF) {}  /* Wait for data transmission completed flag */
	LINFLEX_1.UARTSR.B.DTF = 1;				/* clear the DTF flag and not the other flags */
}

void UART1_putStr(char *str)
{
	uint8_t len = 0;
	
	while(str[len] != 0)
	{
		UART1_putChar(str[len]);
		len++;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initDSPI(void)
{
	DSPI_0.MCR.B.MSTR = 1;				// SPI Master Mode
	DSPI_0.MCR.B.CONT_SCKE = 0;			// Continuous SCK Disable
	DSPI_0.MCR.B.DCONF = 0;				// DSPI Configuration = SPI
	DSPI_0.MCR.B.FRZ = 0;				// Debug Mode => Do not halt Transfer
	DSPI_0.MCR.B.MTFE = 0;				// Modified SPI transfer Formate Disable
	DSPI_0.MCR.B.PCSIS0 = 1;			// CS0_0 is high => chip deselect
	DSPI_0.MCR.B.PCSIS1 = 1;			// CS1_0 is high => chip deselect
	DSPI_0.MCR.B.PCSIS2 = 1;			// CS1_0 is high => chip deselect
	DSPI_0.MCR.B.MDIS = 0;				// Module Enable
	DSPI_0.MCR.B.DIS_TXF = 1; 			// FIFO Transmit Disable
	DSPI_0.MCR.B.DIS_RXF = 1;			// FIFO Receive Disable
	DSPI_0.MCR.B.CLR_TXF = 0;
	DSPI_0.MCR.B.CLR_RXF = 0;
	DSPI_0.MCR.B.SMPL_PT = 0;
	DSPI_0.MCR.B.HALT = 1;				// Stop Transfer
	
	DSPI_0.CTAR[0].B.DBR = 0;			// Double BaudRate Disable
	DSPI_0.CTAR[0].B.FMSZ = 7;			// Frame Size 8
	DSPI_0.CTAR[0].B.CPOL = 0;			// the inactive state value of SCK is low
	DSPI_0.CTAR[0].B.CPHA = 0;			// Data is captured on the leading edge of SCK and changed on the following edge
	DSPI_0.CTAR[0].B.LSBFE = 0;			// LSB First Disable
	DSPI_0.CTAR[0].B.PCSSCK = 0b0;		// prescaler Value = 1
	DSPI_0.CTAR[0].B.PASC = 0b0;		// prescaler value = 1
	DSPI_0.CTAR[0].B.PDT = 0b10;		// prescaler value = 1
	DSPI_0.CTAR[0].B.PBR = 0b00;		// prescaler value = 2
	DSPI_0.CTAR[0].B.CSSCK = 0b0111;	// prescaler value 256
	DSPI_0.CTAR[0].B.ASC = 0b0111;		// prescaler value 256
	DSPI_0.CTAR[0].B.DT = 0b10;			// prescaler value 8
	DSPI_0.CTAR[0].B.BR = 0b0011;		// prescaler value 8	SCK Baudrate = (Fsys * DBR Value) / (BR Value * PBR Value)
	
	DSPI_0.MCR.B.HALT = 0x0;			// Start Transfer
	
	SIU.PCR[12].R = 0x0103;        		/* MPC56xxB: Config pad PA12 as DSPI_0 MISO input */
	SIU.PCR[13].R = 0x0604;        		/* MPC56xxB: Config pad PA13 as DSPI_0 MOSI output */
	SIU.PCR[14].R = 0x0604;        		/* MPC56xxB: Config pad PA14 as DSPI_0 SCK output */
	
	SIU.PCR[15].B.PA = 1;        		/* MPC56xxB: Config pad PA15 as DSPI_0 CS0_0 output */
	SIU.PCR[28].B.PA = 3;        		/* MPC56xxB: Config pad PB12 as DSPI_0 CS1_0 output */
	SIU.PCR[29].B.PA = 3;        		/* MPC56xxB: Config pad PB13 as DSPI_0 CS2_0 output */	
}

unsigned char writeDataDSPI_0(unsigned char data, unsigned char cs, unsigned char cont_cs) 
{
	unsigned char data_rx;
	
	if(cs == 0 && cont_cs == 0)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x08010000 | data);	
	}
	else if(cs == 0 && cont_cs == 1)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x88010000 | data);	
	}
	
	if(cs == 1 && cont_cs == 0)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x08020000 | data);	
	}
	else if(cs == 1 && cont_cs == 1)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x88020000 | data);	
	}
	
	if(cs == 2 && cont_cs == 0)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x08040000 | data);	
	}
	else if(cs == 2 && cont_cs == 1)
	{
		DSPI_0.PUSHR.R = (uint32_t)(0x88040000 | data);	
	}
	
	while (DSPI_0.SR.B.RFDF != 1){} /* Wait for Receive FIFO Drain Flag = 1 */
	data_rx = (uint8_t)DSPI_0.POPR.R;   	/* Read data received by master SPI */
	DSPI_0.SR.R = 0x90020000;       /* Clear TCF, RDRF, EOQ flags by writing 1 */
	
	return data_rx;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initI2C(void)
{		
//	SIU.PCR[10].B.SRC = 0x1;	// PB2 SDA
//	SIU.PCR[11].B.SRC = 0x1;	// PB3 SCL
	
//	SIU.PCR[10].B.WPE = 0x1;	// PB2 SDA
//	SIU.PCR[11].B.WPE = 0x1;	// PB3 SCL
//	
//	SIU.PCR[10].B.WPS = 0x1;	// PB2 SDA
//	SIU.PCR[11].B.WPS = 0x1;	// PB3 SCL
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	SIU.PCR[SDA].B.OBE = 1;
	SIU.PCR[SCL].B.OBE = 1;
}

void I2C_Delay(void)
{
	Delay(1);
}

void I2C_Master_Start(void)
{
	SIU.PCR[SDA].B.IBE = 0;
	SIU.PCR[SDA].B.OBE = 1;

	SIU.PCR[SCL].B.IBE = 0;
	SIU.PCR[SCL].B.OBE = 1;
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	SIU.GPDO[SDA].B.PDO = 1;
	SIU.GPDO[SCL].B.PDO = 1;
	I2C_Delay();
	I2C_Delay();
	SIU.GPDO[SDA].B.PDO = 0;
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 0;
	I2C_Delay();
}

void I2C_Master_Stop(void)
{
	SIU.PCR[SDA].B.IBE = 0;
	SIU.PCR[SDA].B.OBE = 1;	
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 1;
	I2C_Delay();
	SIU.GPDO[SDA].B.PDO = 1;
	I2C_Delay();
	I2C_Delay();
}

uint8_t I2C_Master_TX(uint8_t _data)
{
	uint8_t Data = _data;
	uint8_t Ack = 0x00;
	uint8_t i = 0;
	
	SIU.PCR[SDA].B.IBE = 0;
	SIU.PCR[SDA].B.OBE = 1;
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	// 1~8th Clock
	for(i = 8; i >= 1; i--)
	{
		SIU.GPDO[SDA].B.PDO = (Data >> (i - 1)) & 0x01;
		I2C_Delay();
		SIU.GPDO[SCL].B.PDO = 1;
		I2C_Delay();
		I2C_Delay();
		SIU.GPDO[SCL].B.PDO = 0;
		I2C_Delay();
	}
	
	// 9th Clock, Ack Rx
	SIU.PCR[SDA].B.OBE = 0;
	SIU.PCR[SDA].B.IBE = 1;
	
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 1;
	I2C_Delay();	
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 0;
	Ack = SIU.GPDI[SDA].B.PDI;
	I2C_Delay();
	
	SIU.PCR[SDA].B.IBE = 0;
	SIU.PCR[SDA].B.OBE = 1;
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	return Ack;
}

uint8_t I2C_Master_RX(uint8_t _ack)
{
	uint8_t Data = 0x00;
	uint8_t Ack = _ack;
	uint8_t i = 0;
	
	SIU.PCR[SDA].B.OBE = 0;
	SIU.PCR[SDA].B.IBE = 1;
	
	// 1~8th Clock
	for(i = 1; i <= 8; i++)
	{
		I2C_Delay();
		SIU.GPDO[SCL].B.PDO = 1;
		I2C_Delay();
		I2C_Delay();
		Data = Data << 1;
		Data |= (SIU.GPDI[SDA].B.PDI & 0x01);
		SIU.GPDO[SCL].B.PDO = 0;
		I2C_Delay();
	}
	
	//9th Clock, Ack Tx
	SIU.PCR[SDA].B.IBE = 0;		
	SIU.PCR[SDA].B.OBE = 1;
	
	SIU.PCR[SDA].B.ODE = 1;
	SIU.PCR[SCL].B.ODE = 1;
	
	SIU.GPDO[SDA].B.PDO = Ack;
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 1;
	I2C_Delay();
	I2C_Delay();
	SIU.GPDO[SCL].B.PDO = 0;
	I2C_Delay();
	
	SIU.GPDO[SDA].B.PDO = 0;
	
	return Data;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void enableIrq(void) 
{
	INTC.CPR.B.PRI = 0;         	 		/* Single Core: Lower INTC's current priority */
//	asm(" wrteei 1");	    	   			/* Enable external interrupts */
}
