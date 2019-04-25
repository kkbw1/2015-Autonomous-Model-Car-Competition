#include "MPC5606B.h"
#include "IntcInterrupts.h"

#include "INIT.h"
#include "CLCD.h"
#include "LS7366.h"

#define OFF				0
#define ON				1
//////////////////////////////////////////////////////////////////////
#define CNT_CAM			1280	//640		//1280
#define CNT_CTRL		76800	//76800		//96000		//64000		//128000	//192000	//320000	
//////////////////////////////////////////////////////////////////////
#define CLK1			0
#define SI1				1
#define CLK2			2
#define SI2				3
//////////////////////////////////////////////////////////////////////
#define LIDAR			190
//////////////////////////////////////////////////////////////////////
#define THRES_OFFSET	20

#define LINE_B	0
#define LINE_R	1
#define LINE_L	2
#define LINE_N	3
//////////////////////////////////////////////////////////////////////	
#define SERVO_CENTER	1170
//////////////////////////////////////////////////////////////////////
#define DC_DIFF

#define DC_MAX			997
/////////////////////// CLCD Page ///////////////////////////////
#define PAGE_DISP		4
////////////////////// for variable  i->PIT, j->main//////////////////////////////////
int i, j;
/////////////////////////// flags ////////////////////////////////////////////////
volatile uint8_t flagDisp;
volatile uint8_t flagPIT;
volatile uint8_t flagServo;
volatile uint8_t flagThres;
volatile uint8_t flagDC;
//////////////////////////// School Zone /////////////////////////////////////
volatile uint16_t dataIR[4][10];
volatile uint8_t cntIR;

volatile uint16_t flagIR;
volatile uint8_t flagSchool;
volatile uint16_t flagSchoolInt;
///////////////////////////// AEB /////////////////////////////////////////////
uint8_t flagAEB;
volatile uint16_t distance;
/////////////////////////// UART, Bluetooth ///////////////////////////////////////
uint8_t flagBT;
uint8_t flagUART;

uint8_t data_btrx;
uint8_t data_uart;
///////////////////////// PIT ////////////////////////////////////////////
volatile uint8_t timerRest = 0;
volatile uint16_t flagTimer = 0;
volatile uint32_t CPeriod;
volatile uint32_t periodCtrl;
//////////////////////////// potentiometer on board /////////////////////////////////
volatile uint16_t dataPot = 0;			// pot adc value
////////////////////////// distance sensor //////////////////////////////////////////
int16_t a, b, b_old;
uint32_t measure, input_t;

volatile uint8_t cntLaser;
volatile uint16_t valLaser[10];
volatile uint32_t avgLaser;
volatile uint32_t medLaser;
/////////////////////////////////////////////////////////////////////////////////
volatile uint8_t px_num;
volatile uint16_t cam_temp;
//////////////////////// Cam Upper ///////////////////////////////////////////
volatile uint16_t cam_rawR[130];
volatile uint8_t cam_binR[128];
volatile uint8_t cntBinR;

volatile uint16_t minR;
volatile uint16_t maxR;
volatile uint8_t idxMaxR;
volatile uint8_t idxMinR;
volatile uint16_t cam_thrR;
//volatile uint16_t cam_maxR[128];	// right cam pixel max value
//volatile uint16_t cam_thrPxR[128];	// right cam pixel threshold
//volatile int cam_diffH[130];

volatile uint8_t idxRRCL;			// right cam right center line
volatile uint8_t idxRLCL;			// right cam left center line

volatile uint8_t idxRRCS;			// right cam right start
volatile uint8_t idxRLCE;			// right cam left end
//////////////////////// Cam Bottom ///////////////////////////////////////////
volatile uint16_t cam_rawL[130];
volatile uint8_t cam_binL[128];
volatile uint8_t cntBinL;

volatile uint16_t minL;
volatile uint16_t maxL;
volatile uint8_t idxMaxL;
volatile uint8_t idxMinL;
volatile uint16_t cam_thrL;
//volatile uint16_t cam_maxL[128];	// left cam pixel max value
//volatile uint16_t cam_thrPxL[128];	// left cam pixel threshold
//volatile int cam_diffB[130];

volatile uint8_t idxLRCL;			// left cam right center line
volatile uint8_t idxLLCL;			// left cam left center line

volatile uint8_t idxLRCS;			// left cam right start
volatile uint8_t idxLLCE;			// left cam left end
////////////////////////// Cam Unite ////////////////////////////////////////
volatile uint8_t idx1;
volatile uint8_t idx2;

volatile uint16_t cam_rawU[240];
volatile uint8_t cam_binU[240];
volatile uint8_t cntBinU;

volatile uint8_t idxRCS;			// Right Cam Start
volatile uint8_t idxLCE;			// Left Cam End

volatile uint8_t idxRCC = 127;		// Right Cam Center
volatile uint8_t idxLCC = 0;		// Left Cam Cetner

volatile uint8_t idxURLC;			// Right Left Cam Cross Point => idxRCC - idxRCS

volatile uint8_t idxUC;
volatile uint8_t idxURCL = 7;		// Unite array right lane
volatile uint8_t idxULCL = 162;		// Unite array left lane

volatile uint8_t diffRL;			// Right Left Cam Differnce
volatile uint16_t sizeUnite;
/////////////////////////////////////////////////////////////////////////////////////
//volatile uint8_t idxStartLineRL;
//volatile uint8_t idxEndLineRL;
//volatile uint8_t cntLineRL;
//volatile uint8_t idxSLineRL[10];
//
//volatile uint8_t idxStartLineRR;
//volatile uint8_t idxEndLineRR;
//volatile uint8_t cntLineRR;
//volatile uint8_t idxSLineRR[10];
/////////////////////////////////////////////////////////////////////////////////////
//volatile uint8_t idxStartLineLL;
//volatile uint8_t idxEndLineLL;
//volatile uint8_t cntLineLL;
//volatile uint8_t idxSLineLL[10];
//
//volatile uint8_t idxStartLineLR;
//volatile uint8_t idxEndLineLR;
//volatile uint8_t cntLineLR;
//volatile uint8_t idxSLineLR[10];
/////////////////////////// Lane position //////////////////////////////////////////
volatile uint8_t idxStartLineUL;
volatile uint8_t idxEndLineUL;
volatile uint8_t cntLineUL;
volatile uint8_t idxSLineUL[10];

volatile uint8_t idxStartLineUR;
volatile uint8_t idxEndLineUR;
volatile uint8_t cntLineUR;
volatile uint8_t idxSLineUR[10];
////////////////////////// Car Position ////////////////////////////////////////////
volatile uint8_t flagLine[10];
volatile int MyPos[10];
volatile int MyPosAvg;
volatile long MyPosSum;
///////////////////////// Sampling Time ///////////////////////////////////////////
volatile double Ts;
//////////////////////// Steering Servo ///////////////////////////////////////////
volatile double errServo_now;
volatile double errServo_pre;
volatile double errServo_sum;

volatile double P_Servo;
volatile double I_Servo;
volatile double Isat_Servo;
volatile double D_Servo;

volatile double Servo_Pterm;
volatile double Servo_Iterm;
volatile double Servo_Dterm;
volatile double Servo_Dterm_old;

volatile int16_t dutyServo_Center;
volatile int16_t dutyServo_Max;
volatile int16_t dutyServo_Min;

volatile double dutyServo_old;
volatile double dutyServo;
/////////////////////// DC Velocity /////////////////////////////////////////////
int cntDesire;
int cntDesire_def;

double DiffValue1;
double DiffValue2;
////////////////////// DC1 -> Right ///////////////////////////////////////////////
uint16_t cntTgt1;

uint32_t cntDC1_now;
uint32_t cntDC1_pre;

long cntDC1;
long cntDC1_arr[10];
long cntDC1_avg;

double errDC1_now;
double errDC1_pre;
double errDC1_sum;

double P_DC1;
double I_DC1;
double Isat_DC1;
double D_DC1;
double N_DC1;

double DC1_Pterm;
double DC1_Iterm;
double DC1_Dterm;
double DC1_Dterm_old;
//double DC1_Dsum;
////////////////////// DC2 -> LEft /////////////////////////////////////////////////
uint16_t cntTgt2;

uint32_t cntDC2_now;
uint32_t cntDC2_pre;
long cntDC2;
long cntDC2_arr[10];
long cntDC2_avg;

double errDC2_now;
double errDC2_pre;
double errDC2_sum;

double P_DC2;
double I_DC2;
double Isat_DC2;
double D_DC2;
double N_DC2;

double DC2_Pterm;
double DC2_Iterm;
double DC2_Dterm;
double DC2_Dterm_old;
//double DC2_Dsum;
//////////////////////// DC PWM Duty ///////////////////////////////
double dutyDiff;
double dutyRatio1;
double dutyRatio2;

long cntDCAvg;

int dutyMax;

int dutyDC1_old;
int dutyDC1;

int dutyDC2_old;
int dutyDC2;
//////////////////////////////////////////////////////////////////////////

double LPF(double alpha, double yPre, double xNow);
uint16_t MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData);
uint16_t MDF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData);

void ISR_LIN0_UART_RECV(void);
void ISR_LIN1_UART_RECV(void);
void ISR_PIT0(void);

void GetPixeldata(void);

void GetVelocity(void);
void GetEtcADC(void);
void GetLidar(void);

void CalcMedianVelocity(void);

void DetectObject(void);

void TransformImage(void);
void DetectLine(void);
void DetectSchool(void);

void SetPosition(void);

void SetServo(void);
void DiffDC(void);
void SetDC(void);

void ControlActuator(void);

void BTSendData(void);
void UART0SendData(void);

void initVar(void);
void initGain(void);
void MainLoop(void);
void BehaviorBTData(void);
void BehaviorUART(void);
void DispData(void);
void Button(void);

// Low Pass Filter
double LPF(double alpha, double yPre, double xNow)
{
	return yPre + alpha * (xNow - yPre);
}

// Moving Average Filter
uint16_t MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint32_t avgData = 0;	
 
	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
		avgData += arrayData[i];
	}
	
	// insert new data
	arrayData[0] = newData;
	avgData += arrayData[0];
 
	// calculate average
	avgData = avgData / arraySize;
 
	return (uint16_t)avgData;
}

// Median Filter
uint16_t MDF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint16_t sort[16];
	uint8_t n, m = 0;
	uint16_t temp = 0;
	uint16_t resData = 0;

	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
	}
	// insert new data
	arrayData[0] = newData;
	
	for(i = 0; i < arraySize; i++)
	{
		sort[i] = arrayData[i];
	}
	
	for(n = 0; n < arraySize-1; n++) 
	{
		for(m = n+1; m < arraySize; m++) 
		{
			if(sort[n] > sort[m])
			{
				temp = sort[n];
				sort[n] = sort[m];
				sort[m] = temp;
			}
		}
	}
	
	resData = sort[arraySize / 2];
	return resData;	
}

void ISR_LIN0_UART_RECV(void)	// Data Reception Flag is Set
{
	data_uart = (uint8_t)LINFLEX_0.BDRM.B.DATA4;
	
	/* clear the DRF and RMB flags by writing 1 to them */
	LINFLEX_0.UARTSR.B.DRF = 1;
}

void ISR_LIN1_UART_RECV(void)	// Data Reception Flag is Set
{
	data_btrx = (uint8_t)LINFLEX_1.BDRM.B.DATA4;
	
	/* clear the DRF and RMB flags by writing 1 to them */
	LINFLEX_1.UARTSR.B.DRF = 1;
}

void ISR_PIT0(void)
{	
	PIT.CH[0].TFLG.R = 1;
	
	timerRest = (uint8_t)(flagTimer % 2);
	if(timerRest == 0)			// Clock rise, 0, 2, 4, ..., 258
	{
		if(flagTimer == 0)			//SI rise
		{
			SIU.GPDO[SI1].B.PDO = 1;
			SIU.GPDO[SI2].B.PDO = 1;
//			Delay(10);
			SIU.GPDO[CLK1].B.PDO = 1;
			SIU.GPDO[CLK2].B.PDO = 1;
		}
		else if(flagTimer == 258)
		{
			SIU.GPDO[CLK1].B.PDO = 1;
			SIU.GPDO[CLK2].B.PDO = 1;
			PIT.CH[0].LDVAL.R = (uint32_t) (CNT_CTRL - 1);
		}
		else
		{
			SIU.GPDO[CLK1].B.PDO = 1;
			SIU.GPDO[CLK2].B.PDO = 1;
		}
	}
	else if(timerRest == 1)		// Clock fall, 1, 3, 5, ..., 259
	{
		GetPixeldata();
		if(flagTimer == 1)			// SI fall
		{
			SIU.GPDO[SI1].B.PDO = 0;
			SIU.GPDO[SI2].B.PDO = 0;
			SIU.GPDO[CLK1].B.PDO = 0;
			SIU.GPDO[CLK2].B.PDO = 0;
		}
		else if(flagTimer == 259)
		{
			SIU.GPDO[CLK1].B.PDO = 0;
			SIU.GPDO[CLK2].B.PDO = 0;
			PIT.CH[0].LDVAL.R = (uint32_t) (CNT_CAM - 1);
		}
		else
		{
			SIU.GPDO[CLK1].B.PDO = 0;
			SIU.GPDO[CLK2].B.PDO = 0;
		}
	}
	
	if(flagTimer == 259)		// Control Cycle
	{
		flagTimer = 0;
		px_num = 0;
		
		GetVelocity();
		GetEtcADC();
		GetLidar();
		CalcMedianVelocity();
		
		DetectObject();
		
		TransformImage();
		DetectLine();
		
		if(flagDC == ON && SIU.GPDI[105].B.PDI == 0x00)			// Toggle SW 4
		{
			DetectSchool();
		}
		
		SetPosition();
		
		if(flagServo == ON)
		{
			SetServo();
		}
		
		if(flagDC == ON)
		{
			DiffDC();
			if(flagAEB == ON)
			{
				cntTgt1 = 0;
				cntTgt2 = 0;
			}
			SetDC();
		}
		
		ControlActuator();
		
		BTSendData();
		UART0SendData();
		
		CPeriod = PIT.CH[0].CVAL.R;
		periodCtrl = (CNT_CTRL - CPeriod) / 64;
		
		// Change ADC Channel => Get CAM value
		ADC_0.NCMR0.R = 0;  
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 2);	// Channel 2 Cam Right
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 3);	// Channel 3 Cam Left	
		
		PIT.CH[0].LDVAL.R = (uint32_t) (CNT_CAM - 1);
	}
	else
	{
		flagTimer++;
	}
}

void GetPixeldata(void)
{
	ADC_0.MCR.B.NSTART = 1;
	/* Wait for last scan to complete */
	while (ADC_0.MSR.B.NSTART == 1) {};
	
	cam_rawR[px_num] = (uint16_t)(ADC_0.CDR[2].B.CDATA);		// CamRight 8bit ADC
	cam_rawL[px_num] = (uint16_t)(ADC_0.CDR[3].B.CDATA);		// CamLeft 8bit ADC
	px_num++;
}

void GetVelocity(void)
{
	cntDC1_pre = cntDC1_now;
	cntDC2_pre = cntDC2_now;
	cntDC1_now = GetEncoderPulse(1);
	cntDC2_now = GetEncoderPulse(2);
	
	cntDC1 =  -(int)(cntDC1_now - cntDC1_pre);
	cntDC2 =  -(int)(cntDC2_pre - cntDC2_now);
}

void GetEtcADC(void)
{
	// Rearrange IR sensor from old 10 Cycle
	for(i = 8; i >= 0; i--)
	{
		dataIR[0][i + 1] = dataIR[0][i];
		dataIR[1][i + 1] = dataIR[1][i];
		dataIR[2][i + 1] = dataIR[2][i];
		dataIR[3][i + 1] = dataIR[3][i];
	}
	
	// Get Pot & IR value
	ADC_0.NCMR0.R = 0;
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 0);	// Channel 0 Pot
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 4);	// Channel 4 IR1
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 5);	// Channel 5 IR2
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 6);	// Channel 6 IR3
	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 7);	// Channel 7 IR4
	
	ADC_0.MCR.B.NSTART = 1;
	/* Wait for last scan to complete */
	while (ADC_0.MSR.B.NSTART == 1) {};
	
	dataPot = (uint16_t)(ADC_0.CDR[0].B.CDATA);		// POT value
	dataIR[0][0] = (uint16_t)(ADC_0.CDR[4].B.CDATA);	// IR1 value
	dataIR[1][0] = (uint16_t)(ADC_0.CDR[5].B.CDATA);	// IR1 value
	dataIR[2][0] = (uint16_t)(ADC_0.CDR[6].B.CDATA);	// IR1 value
	dataIR[3][0] = (uint16_t)(ADC_0.CDR[7].B.CDATA);	// IR1 value
}

void GetLidar(void)
{
	if(EMIOS_0.CH[9].CSR.B.FLAG == 1)
	{
		a = EMIOS_0.CH[9].CADR.R;	
		b = EMIOS_0.CH[9].CBDR.R;
		EMIOS_0.CH[9].CSR.B.FLAG = 1;				/*Clear Flag*/
	
		// b & b_old Period Calculation
		if(b < b_old)
		{
			input_t = (65534 - b_old) + b;	
		}
		else
		{
			input_t = b - b_old;
		}
		
		// Pulse width by subtracting the value in B1 from A2
		if(a < b)
		{
			measure = (65534 - b) + a;
		}
		else
		{
			measure = a - b;
		}
		
		measure = measure / 10;
		b_old = b;	
		
		//avgLaser = MAF(10, valLaser, measure);
		medLaser = MDF(5, valLaser, measure);
		
		if(medLaser > 450)
		{
			medLaser = 9999;
		}
	}
}

void CalcMedianVelocity(void)
{
	long sort1[16];
	long sort2[16];
	uint8_t arraySize = 5;
	uint8_t n, m = 0;
	long temp = 0;

	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		cntDC1_arr[i] = cntDC1_arr[i - 1];
		cntDC2_arr[i] = cntDC2_arr[i - 1];
	}
	// insert new data
	cntDC1_arr[0] = cntDC1;
	cntDC2_arr[0] = cntDC2;
	
	// Copy array data
	for(i = 0; i < arraySize; i++)
	{
		sort1[i] = cntDC1_arr[i];
		sort2[i] = cntDC2_arr[i];
	}
	
	// Sorting array data
	for(n = 0; n < arraySize-1; n++) 
	{
		for(m = n+1; m < arraySize; m++) 
		{
			if(sort1[n] > sort1[m])
			{
				temp = sort1[n];
				sort1[n] = sort1[m];
				sort1[m] = temp;
			}
		}
	}
	
	for(n = 0; n < arraySize-1; n++) 
	{
		for(m = n+1; m < arraySize; m++) 
		{
			if(sort2[n] > sort2[m])
			{
				temp = sort2[n];
				sort2[n] = sort2[m];
				sort2[m] = temp;
			}
		}
	}
	
	// Get median value
	cntDC1_avg = sort1[arraySize / 2];
	cntDC2_avg = sort2[arraySize / 2];	
}

void DetectObject(void)
{
	if(medLaser < distance) 	// Toggle SW 3
	{
		flagAEB = ON;
//		if(medLaser < 70)
//		{
//			SIU.GPDO[80].B.PDO = 0;		//PF0 DC Enable
//		}
	}
	else
	{
		flagAEB = OFF;
//		if(flagDC == ON)
//		{
//			SIU.GPDO[80].B.PDO = 1;		//PF0 DC Enable
//		}
	}
}

void TransformImage(void)
{	
	// Get Max & Min Value
	maxR = cam_rawR[0];
	minR = cam_rawR[0];
	maxL = cam_rawL[0];
	minL = cam_rawL[0];
	for(i = 0; i < 128; i++)
	{
		if(cam_rawR[i] > maxR)
		{
			maxR = cam_rawR[i];
			idxMaxR = i;
		}

		if(cam_rawR[i] < minR)
		{
			minR = cam_rawR[i];
			idxMinR = i;
		}
		
		if(cam_rawL[i] > maxL)
		{
			maxL = cam_rawL[i];
			idxMaxL = i;
		}

		if(cam_rawL[i] < minL)
		{
			minL = cam_rawL[i];
			idxMinL = i;
		}
	}
	
	// Calculate Theshold
	cam_thrR = (maxR + minR) / 3;
	cam_thrL = (maxL + minL) / 3;
	
	// Binary Transform
	cntBinR = 0;
	cntBinL = 0;
	for(i = 0; i < 128; i++)
	{
		if(cam_rawR[i] > cam_thrR)
		{
			cam_binR[i] = 255;	
		}
		else
		{
			cam_binR[i] = 0;
			if((i >= idxRCS) && (i <= idxRCC))
			{
				cntBinR++;
			}
		}
		
		
		if(cam_rawL[i] > cam_thrL)
		{
			cam_binL[i] = 255;
		}
		else
		{
			cam_binL[i] = 0;
			if((i >= idxLCC) && (i <= idxLCE))
			{
				cntBinL++;
			}
		}
	}
	
	for(i = 105; i >= 10; i--)
	{
		if(cam_binR[i] == 0)
		{
			idx1 = i;
			break;
		}
	}
	
	for(i = 20; i <= 115; i++)
	{
		if(cam_binL[i] == 0)
		{
			idx2 = i;
			break;
		}
	}
	
	// Add 2 Camera Binary Data
	for(i = 0; i < 128; i++)
	{
		if(i >= idxRCS && i <= idxRCC)
		{
			cam_rawU[i - idxRCS] = cam_rawR[i];				// idxRCS ~ idxRCC
			cam_binU[i - idxRCS] = cam_binR[i];				// idxRCS ~ idxRCC
		}
		
		if(i >= idxLCC && i <= idxLCE)
		{
			cam_rawU[(i - idxRCS) + diffRL] = cam_rawL[i];	// idxLCC ~ idxLCE
			cam_binU[(i - idxRCS) + diffRL] = cam_binL[i];	// idxLCC ~ idxLCE
		}
	}
}

void DetectLine(void)
{
	idxStartLineUL = 255;
	idxEndLineUL = 255;
	cntLineUL = 0;
	
	idxStartLineUR = 255;
	idxEndLineUR = 255;
	cntLineUR = 0;
	
	// Unite Cam right side line detect
	for(i = idxUC; i >= 0; i--)
	{
		// Find 1st value = 0 index
		if(idxStartLineUR == 255 && cam_binU[i] == 0)
		{
			idxStartLineUR = (uint8_t)i;
			cntLineUR++;
			
			break;
		}
	}
	// Unite Cam left side line detect
	for(i = idxUC + 1; i < sizeUnite; i++)
	{
		// Find 1st value = 0 index
		if(idxStartLineUL == 255 && cam_binU[i] == 0)
		{
			idxStartLineUL = (uint8_t)i;
			cntLineUL++;
			
			break;
		}
	}
	
	// except process
	// Both Line finded & any lines positioned center index
	if(idxStartLineUR == idxUC && idxStartLineUL == (idxUC + 1))
	{
		if(flagLine[0] == LINE_R)			// previous line detect is Right => ignore left line
		{
			cntLineUL = 0;
			
			idxStartLineUL = 255;
			idxStartLineUR = idxUC;
		}
		else if(flagLine[0] == LINE_L)		// previous line detect is Left => ignore right line
		{
			cntLineUR = 0;
			
			idxStartLineUL = idxUC;
			idxStartLineUR = 255;
		}
		else if(flagLine[0] == LINE_B)
		{
			
		}
	}
	// Right line finded & previous line is left line & previous, now idx diff is small
	else if(cntLineUR != 0 && cntLineUL == 0 && flagLine[0] == LINE_L)
	{
		if(idxSLineUL[0] - idxStartLineUR < 70)
		{
			idxStartLineUR = 255;
			idxStartLineUL = idxUC;		// Center index

			cntLineUR = 0;
			cntLineUL = 1;
		}
	}
	// Left line finded & previous line is right line & previous, now idx diff is small
	else if(cntLineUR == 0 && cntLineUL != 0 && flagLine[0] == LINE_R)
	{
		if(idxStartLineUL - idxSLineUR[0] < 70)
		{
			idxStartLineUR = idxUC;		// Center index
			idxStartLineUL = 255;	
			
			cntLineUR = 1;
			cntLineUL = 0;	
		}
	}
	else if(cntLineUR == 0 && cntLineUL == 0)
	{
		idxStartLineUL = idxSLineUL[0];
		idxStartLineUR = idxSLineUR[0];
		
		if(idxStartLineUL != 255)
		{
			cntLineUL = 1;
		}
		
		if(idxStartLineUR != 255)
		{
			cntLineUR = 1;
		}
	}
	
	// Rearrange Line index from old 10 Cycle
	for(i = 8; i >= 0; i--)
	{
		idxSLineUR[i + 1] = idxSLineUR[i];
		idxSLineUL[i + 1] = idxSLineUL[i];
	}
	
	// Right line add to array
	if(cntLineUR != 0)
	{
		idxSLineUR[0] = idxStartLineUR;
	}
	else
	{
		idxSLineUR[0] = 255;
	}
	
	// Left line add to array
	if(cntLineUL != 0)
	{
		idxSLineUL[0] = idxStartLineUL;
	}
	else
	{
		idxSLineUL[0] = 255;
	}
}

void DetectSchool(void)
{
	if(flagIR < 330)	// Start Delay 2sec
	{
		flagIR++;
	}
	
	cntIR = 0;
	for(i = 0; i < 4; i++)
	{
		if(dataIR[i][0] < dataPot)		// now
		{
			cntIR++;
		}
		if(dataIR[i][1] < dataPot)		// prev
		{
			cntIR++;
		}
		if(dataIR[i][2] < dataPot)		// old
		{
			cntIR++;
		}
	}
	
	if(cntIR >= 9 && flagIR >= 320)
	{
		if(flagSchool == 0)									// School Zone Start
		{
			flagSchool = 1;
			cntDesire_def = cntDesire;
			cntDesire = 9;
		}
		else if(flagSchool == 1 & flagSchoolInt >= 490)		// School Zone End, Delay 3sec
		{
			flagSchool = 2;
			flagSchoolInt = 0;
			cntDesire = cntDesire_def;
		}
		else if(flagSchool == 2 & flagSchoolInt >= 490)		// School Zone flag initialize, Delay 3sec
		{
			flagSchool = 0;
			flagSchoolInt = 0;
		}
	}
	
	if(flagSchool == 1 || flagSchool == 2)					// School Line Detect Delay
	{
		flagSchoolInt++;
	}
}

void SetPosition(void)
{
	// Rearrange line Flag & Position from old 10 Cycle
	for(i = 8; i >= 0; i--)
	{
		flagLine[i + 1] = flagLine[i];
		MyPos[i + 1] = MyPos[i];
	}
	
	if(cntLineUL != 0 && cntLineUR != 0)			// Detect Both Side Line
	{
		flagLine[0] = LINE_B;
//		if(flagSchool == 1)
//		{
//				if(dutyServo < dutyServo_Center)		// Previous Steering Servo Turn Left
//				{
//					MyPos[0] = idxSLineUR[0] - idxURCL;
//				}
//				else if(dutyServo > dutyServo_Center)	// Previous Steering Servo Turn Right
//				{
//					MyPos[0] = idxSLineUL[0] - idxULCL;
//				}
//				else
//				{
//					MyPos[0] = ((idxSLineUL[0] - idxULCL) + (idxSLineUR[0] - idxURCL)) / 2;
//				}
//				flagLine[0] = LINE_B;
//		}
			
		if((idxStartLineUL - idxStartLineUR) < 120)
		{
			if(flagSchool != 1)
			{
				// Average My Position from old 10 cycle
				MyPosSum = 0;
				for(i = 0; i < 10; i++)
				{
					MyPosSum += MyPos[i];
				}
				
				MyPosAvg = MyPosSum / 10;
				MyPos[0] = MyPosAvg;
			}
			else if(flagSchool == 1)
			{
				if(dutyServo < dutyServo_Center)		// Previous Steering Servo Turn Left
				{
					MyPos[0] = idxSLineUR[0] - idxURCL;
				}
				else if(dutyServo > dutyServo_Center)	// Previous Steering Servo Turn Right
				{
					MyPos[0] = idxSLineUL[0] - idxULCL;
				}
				else
				{
					MyPos[0] = ((idxSLineUL[0] - idxULCL) + (idxSLineUR[0] - idxURCL)) / 2;
				}
				flagLine[0] = LINE_B;
			}
		}
		else
		{
			MyPos[0] = ((idxSLineUL[0] - idxULCL) + (idxSLineUR[0] - idxURCL)) / 2;
		}
	}
	else if(cntLineUL != 0 && cntLineUR == 0)		// Detect Left Side Line
	{
		MyPos[0] = idxSLineUL[0] - idxULCL;
		flagLine[0] = LINE_L;
	}
	else if(cntLineUL == 0 && cntLineUR != 0)		// Detect Right Side Line
	{
		MyPos[0] = idxSLineUR[0] - idxURCL;
		flagLine[0] = LINE_R;			
	}
	else if(cntLineUL == 0 && cntLineUR == 0)		// Detect no Line
	{	
		MyPos[0] = 0;
		
//		MyPosSum = 0;
//		for(i = 0; i < 10; i++)
//		{
//			MyPosSum += MyPos[i];
//		}
//		
//		MyPosAvg = MyPosSum / 10;
//		MyPos[0] = MyPosAvg;
		
		flagLine[0] = LINE_N;
	}
}

void SetServo(void)
{	
	int posDesired = 0;
	
//	if(flagDC == ON)
//	{
//		MyPosSum += MyPos[0];
//	}
	
	errServo_pre = errServo_now;
	errServo_now = posDesired - MyPos[0];
//	errServo_now = 0 - MyPosAvg;
	
	// if DC ON, error Summming
//	if(flagDC == ON)
//	{
//		errServo_sum += errServo_now;
//	}
//	// error Summing Saturation
//	if((int)errServo_sum > (int)Isat_Servo)
//	{
//		errServo_sum = Isat_Servo;
//	}
//	else if((int)errServo_sum < (int)(Isat_Servo * -1))
//	{
//		errServo_sum = Isat_Servo * -1;
//	}
	
	// PID Control	
	Servo_Dterm_old = Servo_Dterm;
	Servo_Pterm = P_Servo * errServo_now;
	Servo_Iterm = I_Servo * errServo_sum;
	Servo_Dterm = D_Servo * (double)(errServo_now - errServo_pre);
//	Servo_Dterm = LPF(0.8, Servo_Dterm_old, Servo_Dterm);
	
	dutyServo_old = dutyServo;
	dutyServo = (int16_t)(dutyServo_Center + Servo_Pterm + Servo_Iterm + Servo_Dterm);
	dutyServo = (int16_t)(LPF(0.85, dutyServo_old, dutyServo));
	if(dutyServo > dutyServo_Max)
	{
		dutyServo = dutyServo_Max;
	}
	else if(dutyServo < dutyServo_Min)
	{
		dutyServo = dutyServo_Min;
	}
}

void DiffDC(void)
{
#ifdef DC_DIFF
	if(dutyServo > dutyServo_Center && flagSchool != 1)		// Right Handle
	{
		dutyDiff = dutyServo - dutyServo_Center;

		// 1st Order Differentier
		// Both Dec & Inc
//		dutyRatio1 = dutyDiff / 2000.0;
//		dutyRatio2 = dutyDiff / 1000.0;
//		cntTgt1 = cntDesire * (1.0 + dutyRatio1);
//		cntTgt2 = cntDesire * (1.0 - dutyRatio2);

		// One Dec
//		dutyRatio1 = dutyDiff / COFF_DIFF1;
//		dutyRatio2 = dutyDiff / COFF_DIFF1;
//		cntTgt1 = cntDesire;
//		cntTgt2 = cntDesire * (1.0 - dutyRatio2);
		
		// Both Dec
		// => speed 190 Optimization
//		dutyRatio1 = dutyDiff / 1400.0;
//		dutyRatio2 = dutyDiff / 1000.0;
		
		dutyRatio1 = dutyDiff / 850;
		dutyRatio2 = dutyDiff / 750;
		
//		dutyRatio1 = dutyDiff / 1100;
//		dutyRatio2 = dutyDiff / 1000;
		
		cntTgt1 = cntDesire * (1.0 - dutyRatio1);
		cntTgt2 = cntDesire * (1.0 - dutyRatio2);
		
		// 2nd Diff
//		dutyRatio1 = dutyDiff / 1500.0;		// inc
//		dutyRatio2 = dutyDiff / 1200.0;		// dec
//		cntTgt1 = cntDesire * (1.0 - dutyRatio1) * (1.0 - dutyRatio1);
//		cntTgt2 = cntDesire * (1.0 - dutyRatio2) * (1.0 - dutyRatio2);		
	}
	else if(dutyServo < dutyServo_Center && flagSchool != 1)	// Left Handle
	{
		dutyDiff = dutyServo_Center - dutyServo;

		// 1st Order Differentier		
		// Both Dec & Inc
//		dutyRatio1 = dutyDiff / 1000.0;
//		dutyRatio2 = dutyDiff / 2000.0;
//		cntTgt1 = cntDesire * (1.0 - dutyRatio1);
//		cntTgt2 = cntDesire * (1.0 + dutyRatio2);

		// One Dec
//		dutyRatio1 = dutyDiff / COFF_DIFF1;
//		dutyRatio2 = dutyDiff / COFF_DIFF1;
//		cntTgt1 = cntDesire * (1.0 - dutyRatio1);
//		cntTgt2 = cntDesire;
		
		// Both Dec
		// => speed 190 Optimization
//		dutyRatio1 = dutyDiff / 1000.0;
//		dutyRatio2 = dutyDiff / 1400.0;
		
		dutyRatio1 = dutyDiff / 750.0;
		dutyRatio2 = dutyDiff / 850.0;
		
//		dutyRatio1 = dutyDiff / 1000;
//		dutyRatio2 = dutyDiff / 1100;
		
		cntTgt1 = cntDesire * (1.0 - dutyRatio1);
		cntTgt2 = cntDesire * (1.0 - dutyRatio2);
		
		// 2nd Diff
//		dutyRatio1 = dutyDiff / 1200.0;		// dec
//		dutyRatio2 = dutyDiff / 1500.0;		// inc
//		cntTgt1 = cntDesire * (1.0 - dutyRatio1) * (1.0 - dutyRatio1);
//		cntTgt2 = cntDesire * (1.0 - dutyRatio2) * (1.0 - dutyRatio2);	
	}
	else
	{
		cntTgt1 = cntDesire;
		cntTgt2 = cntDesire;
	}
#endif

#ifndef DC_DIFF		// NO Diff Control
	cntTgt1 = cntDesire;
	cntTgt2 = cntDesire;
#endif	
}

void SetDC(void)
{
	double Pterm;
	double Iterm;
	double Dterm;
	
	////////////////// DC1 //////////////////////////	
	errDC1_pre = errDC1_now;
	errDC1_now = cntTgt1 - cntDC1_avg;
	errDC1_sum += errDC1_now;
	if(errDC1_sum > Isat_DC1)
	{
		errDC1_sum = Isat_DC1;
	}
	else if(errDC1_sum < Isat_DC1 * -1.0)
	{
		errDC1_sum = Isat_DC1 * -1.0;
	}
	
	Pterm = P_DC1 * errDC1_now;
	Iterm = I_DC1 * errDC1_sum;
	Dterm = D_DC1 * (errDC1_now - errDC1_pre);
//	Dterm = (D_DC1 * errDC1_now - DC1_Dsum) * N_DC1;
//	DC1_Dsum += Dterm;
	
//	Pterm = P_DC1 * errDC1_now;
//	Iterm = I_DC1 * errDC1_sum * Ts;
//	Dterm = D_DC1 * (errDC1_now - errDC1_pre) / Ts;
////	Dterm = (D_DC1 * errDC1_now - DC1_Dsum) * N_DC1 * Ts;
//	DC1_Dsum += Dterm;
	
//	dutyDC1 += (int)(Pterm + Iterm + Dterm);	
//	dutyDC1 = (int)(Pterm + Iterm + Dterm);
	
	if(flagAEB == OFF)
	{
		dutyDC1 = LPF(0.8, dutyDC1, (int)(Pterm + Iterm + Dterm));
	}
	else if(flagAEB == ON)
	{
		dutyDC1 = (int)(Pterm + Iterm + Dterm);
	}
	
	if(dutyDC1 > dutyMax)
	{
		dutyDC1 = dutyMax;
	}
	else if(dutyDC1 < -dutyMax)
	{
		dutyDC1 = -dutyMax;
	}
	////////////////// DC2 //////////////////////////	
	errDC2_pre = errDC2_now;
	errDC2_now = cntTgt2 - cntDC2_avg;
	errDC2_sum += errDC2_now;
	if(errDC2_sum > Isat_DC2)
	{
		errDC2_sum = Isat_DC2;
	}
	else if(errDC2_sum < Isat_DC2 * -1.0)
	{
		errDC2_sum = Isat_DC2 * -1.0;
	}
	
	Pterm = P_DC2 * errDC2_now;
	Iterm = I_DC2 * errDC2_sum;
	Dterm = D_DC2 * (errDC2_now - errDC2_pre);
//	Dterm = (D_DC2 * errDC2_now - DC2_Dsum) * N_DC1;
//	DC2_Dsum += Dterm;
	
//	Pterm = P_DC2 * errDC2_now;
//	Iterm = I_DC2 * errDC2_sum * Ts;
//	Dterm = D_DC2 * (errDC2_now - errDC2_pre) / Ts;
////	Dterm = (D_DC2 * errDC2_now - DC2_Dsum) * N_DC2 * Ts;
////	DC2_Dsum += Dterm;
	
//	dutyDC2 += (int)(Pterm + Iterm + Dterm);
//	dutyDC2 = (int)(Pterm + Iterm + Dterm);
	
	if(flagAEB == OFF)
	{
		dutyDC2 = LPF(0.8, dutyDC2, (int)(Pterm + Iterm + Dterm));
	}
	else if(flagAEB == ON)
	{
		dutyDC2 = (int)(Pterm + Iterm + Dterm);
	}
	
	if(dutyDC2 > dutyMax)
	{
		dutyDC2 = dutyMax;
	}
	else if(dutyDC2 < -dutyMax)
	{
		dutyDC2 = -dutyMax;
	}
}

void ControlActuator(void)
{
	EMIOS_0_OPWM_Duty(4, (uint16_t)(dutyServo));
/////////////////////////////////////////////////
	if(flagDC == ON)
	{
		// DC1 Control
		if(dutyDC1 >= 0)	// Forward Direction
		{
			EMIOS_0_OPWM_Duty(24, (uint16_t)dutyDC1);
			EMIOS_0_OPWM_Duty(25, 0);
		}
		else				// Reverse Direction
		{
			EMIOS_0_OPWM_Duty(24, 0);
			EMIOS_0_OPWM_Duty(25, (uint16_t)(-dutyDC1));	
		}
		// DC2 Control
		if(dutyDC2 >= 0)	// Forward Direction
		{
			EMIOS_0_OPWM_Duty(26, (uint16_t)(dutyDC2));	
			EMIOS_0_OPWM_Duty(27, 0);
		}
		else				// Reverse Direction
		{
			EMIOS_0_OPWM_Duty(26, 0);	
			EMIOS_0_OPWM_Duty(27, (uint16_t)(-dutyDC2));		
		}
	}
}

void BTSendData(void)
{
	if(flagBT == 1)
	{
		UART1_putChar((uint8_t)((errServo_now - errServo_pre) + 128));
	}
	else if(flagBT == 2)
	{
//		UART1_putChar((uint8_t)cntLineUR);
//		UART1_putChar((uint8_t)cntLineUL);
		
		UART1_putChar((uint8_t)cntBinR);
		UART1_putChar((uint8_t)cntBinL);
	}
	else if(flagBT == 3)
	{
		UART1_putChar((uint8_t)idxSLineUR[0]);
		UART1_putChar((uint8_t)idxSLineUL[0]);
	}
	else if(flagBT == 4)
	{
		UART1_putChar((uint8_t)(cntDC1));
		UART1_putChar((uint8_t)(cntDC2));
	}
	else if(flagBT == 5)
	{
//		UART1_putChar((uint8_t)(dutyDC1 >> 2));
//		UART1_putChar((uint8_t)(dutyDC2 >> 2));	
		
//		UART1_putChar((uint8_t)(MyPosSum));

		UART1_putChar((uint8_t)(dutyServo / 10));
		UART1_putChar((uint8_t)(MyPos[0] + (sizeUnite / 2)));
	}
	else if(flagBT == 6)
	{
		UART1_putChar((uint8_t)((dataIR[0][0]) >> 2));
		UART1_putChar((uint8_t)((dataIR[3][0]) >> 2));
	}
	else if(flagBT == 7)
	{
		for(i = 0; i < 128; i++)
		{
			UART1_putChar(cam_rawR[i] >> 2);
		}
		for(i = 0; i < 128; i++)
		{
			//UART1_putChar(cam_maxR[i] >> 2);
		}
		flagBT = 0;
	}
	else if(flagBT == 8)
	{
		for(i = 0; i < 128; i++)
		{
			UART1_putChar(cam_rawL[i] >> 2);
		}
		for(i = 0; i < 128; i++)
		{
			//UART1_putChar(cam_maxL[i] >> 2);
		}
		flagBT = 0;
	}
	else if(flagBT == 9)
	{
		for(i = 0; i < sizeUnite; i++)
		{
			UART1_putChar(cam_rawU[i] >> 2);
		}
		flagBT = 0;
	}
	// flagBT == 0  -> Stop Send Data
}

void UART0SendData(void)
{
	if(flagUART == 1)
	{
		UART0_putChar((uint8_t)(cntDC1));
		UART0_putChar((uint8_t)(cntDC2));
	}
	else if(flagUART == 2)
	{
//		UART0_putChar((uint8_t)(cam_thrR >> 2));
//		UART0_putChar((uint8_t)(cam_thrL >> 2));	
		
		UART0_putChar((uint8_t)(MyPosSum));
	}
	else if(flagUART == 3)
	{
		for(i = 0; i < 128; i++)
		{
			UART0_putChar(cam_rawR[i] >> 2);		// B
		}
		for(i = 0; i < 128; i++)
		{
			UART0_putChar(cam_rawL[i] >> 2);		// B
		}
//		for(i = 0; i < 128; i++)
//		{
//			UART0_putChar(cam_maxR[i] >> 2);		// R
//		}
//		for(i = 0; i < 128; i++)
//		{
//			UART0_putChar(cam_maxL[i] >> 2);		// R
//		}
//		for(i = 0; i < 128; i++)
//		{
//			UART0_putChar(cam_thrPxR[i] >> 2);		// G
//		}
//		for(i = 0; i < 128; i++)
//		{
//			UART0_putChar(cam_thrPxL[i] >> 2);		// G
//		}
		flagUART = 0;
	}
	else if(flagUART == 4)
	{
		for(i = 0; i < sizeUnite; i++)
		{
			UART0_putChar(cam_rawU[i] >> 2);
		}
		for(i = 0; i < sizeUnite; i++)
		{
			UART0_putChar(cam_binU[i] >> 2);
		}
		UART0_putChar((uint8_t)(cam_thrR >> 2));
		UART0_putChar((uint8_t)(cam_thrL >> 2));	
		flagUART = 0;
	}
	else if(flagUART == 5)
	{
		if(medLaser < 500)
		{
			UART0_putChar(medLaser);
		}		
	}
	else if(flagUART == 6)
	{
		UART0_putChar(periodCtrl / 10);
	}
	else if(flagUART == 7)
	{
		UART0_putChar((uint8_t)(idxStartLineUR));
		UART0_putChar((uint8_t)(idxStartLineUL));
	}
	else if(flagUART == 8)
	{
		
	}
	else if(flagUART == 9)
	{
		
	}
}

void initVar(void)
{
	///////////////////////// Flags ///////////////////////////////
	flagDisp = 0;
	flagPIT = OFF;
	flagThres = OFF;
	flagDC = OFF;
	///////////////////////// School Zone /////////////////////////////
	flagIR = 0;
	cntIR = 0;
	flagSchool = 0;
	flagSchoolInt = 0;
	//////////////////////////// AEB //////////////////////////////////
	flagAEB = OFF;
	distance = LIDAR;
	///////////////////////// UART & BT ///////////////////////////////
	flagBT = 0;
	flagUART = 0;

	data_btrx = 0;
	data_uart = 0;
	///////////////////////// PIT /////////////////////////////
	timerRest = 0;
	flagTimer = 0;
	CPeriod = 0;
	periodCtrl = 0;
	///////////////////////// CAM ///////////////////////////////
	px_num = 0;
	cam_temp = 0;
	///////////////////////// Cam ////////////////////////////////
//	idxRLCL = 118;
//	idxRRCL = 11;
//
//	idxRLOFF = 127 - 3;
//	idxRROFF = 3;

	idxRCS = 10;			// Right Cam Start
	idxLCE = 127-10;		// Left Cam End
	idxRCC = 84;			// Right Cam Center
	idxLCC = 44;			// Left Cam Cetner
	sizeUnite = (idxRCC - idxRCS + 1) + (idxLCE - idxLCC + 1);	// Unite array data length
	
	idxURLC = idxRCC - idxRCS;		// Unite Cam array Diff point
	idxUC = idxURLC;				// Unite Cam array Center point
	
	idxURCL = 7;			// Unite array right lane from center position
	idxULCL = 141;			// Unite array left lane from center position
	
	diffRL = idxRCC - idxLCC + 1;
	
	for(j = 0; j < 10; j++)
	{
		flagLine[j] = LINE_B;
		MyPos[j] = 0;
	}
	/////////////////////////////////////////
	Ts = 0.00512 + 0.0012;		// 20us * 256 = 5.12msec, calc period = 1msec
	/////////////////////////////////////////
	errServo_now = 0;
	errServo_pre = 0;
	errServo_sum = 0;
	
	dutyServo_Center = SERVO_CENTER;
	dutyServo_Min = dutyServo_Center - 300;
	dutyServo_Max = dutyServo_Center + 300;
	/////////////////////////////////////////
	cntDesire = 150;
	/////////////////////////////////////////
	dutyMax = DC_MAX;
	dutyDC1 = 0;
	dutyDC2 = 0;
}

void initGain(void)
{
/////////////////////////////////////////	
	// Best Setting
	P_Servo = 6.5;
	I_Servo = 0.0000;
	Isat_Servo = 2000.0;			// Original 100
	D_Servo = 9.0;

//	P_Servo = 7.0;
//	I_Servo = 0.0;
//	Isat_Servo = 200.0;			// Original 100
//	D_Servo = 15.0;
///////////////////////////////////////////	
	// normal pid gain
	// Rise = 0.0122, Settling = 0.0673, Overshoot = 1.044
//	P_DC1 = 6.7726;
//	I_DC1 = 65.5294;				// *Ts = 0.39491
//	Isat_DC1 = 200000.0;
//	D_DC1 = 0.042;
//	
//	P_DC2 = 6.7726;
//	I_DC2 = 65.5294;				// *Ts = 0.39491
//	Isat_DC2 = 200000.0;
//	D_DC2 = 0.042;
	
	// Ts skip pid gain
	P_DC1 = 15.0;
	I_DC1 = 0.06;		// 0.04
	Isat_DC1 = 4000.0;
	D_DC1 = 0.0;
	
	P_DC2 = 15.0;
	I_DC2 = 0.06;
	Isat_DC2 = 4000.0;
	D_DC2 = 0.0;
	
	//integral pid gain
//	P_DC1 = 0.1;
//	I_DC1 = 0.0008;
//	Isat_DC1 = 2000.0;
//	D_DC1 = 8.0;
//	
//	P_DC2 = 0.1;
//	I_DC2 = 0.0008;
//	Isat_DC2 = 2000.0;
//	D_DC2 = 8.0;
/////////////////////////////////////////
}

void MainLoop(void)
{
	BehaviorBTData();
	BehaviorUART();
	
	DispData();
	Button();
}

void BehaviorBTData(void)
{
	////////////////////// Free Stop, Brake ///////////////////////////////
	if(data_btrx == 'q')		// Stop
	{
		cntDesire = 0;
		
		UART1_putChar('q');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'w')	// Brake
	{
		if(flagDC == ON)			// DC Motor Stop
		{
			flagDC = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			
			SIU.GPDO[80].B.PDO = 0;	//PF0 DC Disable
		}
		else if(flagDC == OFF)		// DC Motor Start
		{
			flagIR = 0;
			cntIR = 0;
			flagSchool = 0;
			flagSchoolInt = 0;
			
			flagDC = ON;
			
			SIU.GPDO[80].B.PDO = 1;	//PF0 DC Enable
		}
		
		UART1_putChar('w');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'e')	// DC Start or Stop
	{

		data_btrx = 0;
	}
	else if(data_btrx == 'r')
	{
		
		data_btrx = 0;
	}
	/////////////////////////////////////////////////////
	else if(data_btrx == 'a')
	{

		data_btrx = 0;
	}
	else if(data_btrx == 's')
	{

		data_btrx = 0;
	}
	else if(data_btrx == 'd')
	{
		
		data_btrx = 0;
	}
	else if(data_btrx == 'f')
	{

		data_btrx = 0;
	}
	//////////////////////// Desire Pulse /////////////////////////
	else if(data_btrx == 'z')		// Speed small Up
	{
		cntDesire += 10;
		if(cntDesire >= 1000 && cntDesire < 60000)
		{
			cntDesire = 999;
		}
		
		UART1_putChar((uint8_t)(cntDesire / 100) + '0');
		
		data_btrx = 0;	
	}
	else if(data_btrx == 'x')		// Speed small Down
	{
		cntDesire -= 10;
		if(cntDesire >= 60000)
		{
			cntDesire = 0;
		}

		UART1_putChar(cntDesire / 100 + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'c')		// Speed Up
	{
		cntDesire += 100;
		if(cntDesire >= 1000 && cntDesire < 60000)
		{
			cntDesire = 999;
		}
		
		UART1_putChar((uint8_t)(cntDesire / 100) + '0');
		
		data_btrx = 0;		
	}
	else if(data_btrx == 'v')		// Speed Down
	{
		cntDesire -= 100;
		if(cntDesire >= 60000)
		{
			cntDesire = 0;
		}

		UART1_putChar(cntDesire / 100 + '0');
		
		data_btrx = 0;	
	}
	/////////////////////// Gain Control ///////////////////////
	else if(data_btrx == 'o')		// P Gain up
	{
		P_DC1 += 0.1;
		P_DC2 += 0.1;
		
		UART1_putChar((uint8_t)(P_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'p')		// P Gain down
	{
		P_DC1 -= 0.1;
		P_DC2 -= 0.1;
				
		UART1_putChar((uint8_t)(P_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == '[')		// D Gain up
	{
		D_DC1 += 0.1;
		D_DC2 += 0.1;
		
		UART1_putChar((uint8_t)(D_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == ']')		//	D Gain down
	{
		D_DC1 -= 0.1;
		D_DC2 -= 0.1;
		
		UART1_putChar((uint8_t)(D_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'k')
	{

		data_btrx = 0;
	}
	else if(data_btrx == 'l')
	{

		data_btrx = 0;
	}
	else if(data_btrx == ';')
	{

		data_btrx = 0;
	}
	///////////////////// Send Data ///////////////////////////
	else if(data_btrx == '1')
	{
		flagBT = 1;
		data_btrx = 0;
	}
	else if(data_btrx == '2')
	{
		flagBT = 2;
		data_btrx = 0;
	}
	else if(data_btrx == '3')
	{
		flagBT = 3;
		data_btrx = 0;
	}
	else if(data_btrx == '4')
	{
		flagBT = 4;
		data_btrx = 0;
	}
	else if(data_btrx == '5')
	{
		flagBT = 5;
		data_btrx = 0;
	}
	else if(data_btrx == '6')
	{
		flagBT = 6;
		data_btrx = 0;
	}
	else if(data_btrx == '7')
	{
		flagBT = 7;
		data_btrx = 0;
	}
	else if(data_btrx == '8')
	{
		flagBT = 8;
		data_btrx = 0;
	}
	else if(data_btrx == '9')
	{
		flagBT = 9;
		data_btrx = 0;
	}
	else if(data_btrx == '0')
	{
		flagBT = 0;
		data_btrx = 0;
	}
}

void BehaviorUART(void)
{
	if(data_uart == '1')
	{
		flagUART = 1;
		data_uart = 0;
	}
	else if(data_uart == '2')
	{
		flagUART = 2;
		data_uart = 0;
	}
	else if(data_uart == '3')
	{
		flagUART = 3;
		data_uart = 0;
	}
	else if(data_uart == '4')
	{
		flagUART = 4;
		data_uart = 0;
	}
	else if(data_uart == '5')
	{
		flagUART = 5;
		data_uart = 0;
	}
	else if(data_uart == '6')
	{
		flagUART = 6;
		data_uart = 0;
	}
	else if(data_uart == '7')
	{
		flagUART = 7;
		data_uart = 0;
	}
	else if(data_uart == '8')
	{
		flagUART = 8;
		data_uart = 0;
	}
	else if(data_uart == '9')
	{
		flagUART = 9;
		data_uart = 0;
	}
	else if(data_uart == '0')
	{
		flagUART = 0;
		data_uart = 0;
	}
}

void DispData(void)
{
	// display 16*2 character
	writeNum1(15, 1, flagDisp);
	
	if(flagDisp == 0)						// Cam1
	{	
		writeNum100(0, 0, maxR);
		writeNum100(4, 0, minR);
		
		writeNum100(8, 0, maxL);
		writeNum100(12, 0, minL);

		writeNum100(0, 1, idxSLineUL[0]);
		writeNum100(4, 1, idxSLineUR[0]);
		
		writeNum100(8, 1, cam_thrL);
		writeNum100(12, 1, cam_thrR);
		
//		if(flagThres == OFF)
//		{
//			writeString(11, 1, " ");
//		}
//		else 
//		{
//			writeString(11, 1, "S");
//		}
//		if(flagPIT == OFF)
//		{
//			writeString(13, 1, "N");
//		}
//		else
//		{
//			writeString(13, 1, "Y");
//		}
	}
	else if(flagDisp == 1)					// Cam2
	{
		writeNum100(0, 0, idxLCC);
		writeNum100(4, 0, idxRCC);
		
		writeNum100(8, 0, idxULCL);
		writeNum100(12, 0, idxURCL);
		
		writeNum100(0, 1, idx2);
		writeNum100(4, 1, idx1);
		
		writeNum100(8, 1, idxUC);
		writeNum100(12, 1, sizeUnite);	
	}
	else if(flagDisp == 2)					// DC Motor
	{
		writeNum100(0, 0, cntDC1_avg);
		writeNum100(4, 0, cntDC2_avg);
		writeNum100(8, 0, cntDesire);
		
		writeNum1000(12, 0, (CNT_CTRL - CPeriod) / 64);
		
		writeSNum100(0, 1, dutyDC1);
		writeSNum100(5, 1, dutyDC2);
		
		if(flagDC == OFF)
		{
			writeString(13, 1, "N");
		}
		else
		{
			writeString(13, 1, "Y");
		}
	}
	else if(flagDisp == 3)					// PID Gain
	{
		writeNum100(0, 0, P_DC1 * 10);
		writeNum100(4, 0, I_DC1 * 100);
		writeNum100(8, 0, D_DC1 * 100);
		
		writeNum100(0, 1, P_DC2 * 10);
		writeNum100(4, 1, I_DC2 * 100);
		writeNum100(8, 1, D_DC2 * 100);
		
		writeNum100(12, 0, P_Servo * 10);
//		writeNum100(4, 1, I_Servo * 10);
		writeNum100(12, 1, D_Servo * 10);
	}
	else if(flagDisp == 4)					// Distance Sensor
	{
		writeHex(0, 0, a >> 8);
		writeHex(2, 0, a);
		writeHex(5, 0, b >> 8);
		writeHex(7, 0, b);
		writeNum1000(10, 0, measure);	
		
		writeHex(0, 1, input_t >> 8);
		writeHex(2, 1, input_t);
		
		if(flagAEB == ON)
		{
			writeString(5, 1, "O");
		}
		else if(flagAEB == OFF)
		{
			writeString(5, 1, "X");
		}
		
		writeNum100(6, 1, distance);
		writeNum1000(10, 1, medLaser);
	}
	else if(flagDisp == 5)					// IR Sensor
	{
		writeNum100(0, 0, dataIR[0][0] >> 2);
		writeNum100(4, 0, dataIR[1][0] >> 2);
		writeNum100(8, 0, dataIR[2][0] >> 2);
		writeNum100(12, 0, dataIR[3][0] >> 2);
		
		writeNum100(0, 1, dataPot >> 2);
		writeNum10(4, 1, cntIR);
		writeNum1(7, 1, flagSchool);
	}
}

void Button(void)
{
	if(SIU.GPDI[52].B.PDI == 0)		// ESW1
	{
		Delay(500000);
		while(SIU.GPDI[52].B.PDI == 0) {};
		Delay(500000);
		
//		D_DC1 += 0.1;
//		D_DC2 += 0.1;
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER);
		
		cntDesire += 10;
	}
	
	if(SIU.GPDI[53].B.PDI == 0)		// ESW2
	{
		Delay(500000);
		while(SIU.GPDI[53].B.PDI == 0) {};
		Delay(500000);

//		D_DC1 -= 0.1;
//		D_DC2 -= 0.1;
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER + 100);
	}
	
	if(SIU.GPDI[54].B.PDI == 0)		// ESW3
	{
		Delay(500000);
		while(SIU.GPDI[54].B.PDI == 0) {};
		Delay(500000);
		
//		I_DC1 += 0.01;
//		I_DC2 += 0.01;
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER + 200);
	}
	
	if(SIU.GPDI[55].B.PDI == 0)		// ESW4
	{
		Delay(500000);
		while(SIU.GPDI[55].B.PDI == 0) {};
		Delay(500000);

//		I_DC1 -= 0.01;
//		I_DC2 -= 0.01;
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER + 300);
		
		flagDisp++;
		if(flagDisp > 5) flagDisp = 0;
		
		clearCLCD();
	}
	
	if(SIU.GPDI[56].B.PDI == 0)		// ESW5
	{
		Delay(500000);
		while(SIU.GPDI[56].B.PDI == 0) {};
		Delay(500000);

//		P_DC1 += 0.1;
//		P_DC2 += 0.1;
		
		cntDesire -= 10;
	}
	
	if(SIU.GPDI[57].B.PDI == 0)		// ESW6
	{
		Delay(500000);
		while(SIU.GPDI[57].B.PDI == 0) {};
		Delay(500000);

//		P_DC1 -= 0.1;
//		P_DC2 -= 0.1;
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER - 100);
	}
	
	if(SIU.GPDI[58].B.PDI == 0)		// ESW7
	{
		Delay(500000);
		while(SIU.GPDI[58].B.PDI == 0) {};
		Delay(500000);
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER - 200);
	}
	
	if(SIU.GPDI[59].B.PDI == 0)		// ESW8
	{
		Delay(500000);
		while(SIU.GPDI[59].B.PDI == 0) {};
		Delay(500000);
		
//		EMIOS_0_OPWM_Duty(4, SERVO_CENTER - 300);
	}
	//////////////////////////////////////////////////////////////////////////////////
	if(SIU.GPDI[67].B.PDI == 0)		// Interal SW4
	{
		Delay(500000);
		while(SIU.GPDI[67].B.PDI == 0) {};
		Delay(500000);
		
		flagDisp++;
		if(flagDisp > 5) flagDisp = 0;
		
		clearCLCD();
	}
	
	if(flagDisp == 0)					// Display #0
	{
		if(SIU.GPDI[64].B.PDI == 0)		// SW1
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
			
			if(flagServo == ON)
			{
				flagServo = OFF;
			}
			else if(flagServo == OFF)
			{
				flagServo = ON;
			}
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2		Set Lane from center position
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
			idxURCL = idxSLineUR[0];
			idxULCL = idxSLineUL[0];
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3,		PIT Control
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
			if(flagPIT == OFF)
			{
				flagIR = 0;
				flagSchool = 0;
				flagSchoolInt = 0;
				
				flagPIT = ON;
				PIT.CH[0].TCTRL.B.TEN = 1;		// PIT Enable
			}
			else if(flagPIT == ON)
			{
				flagIR = 0;
				flagSchool = 0;
				flagSchoolInt = 0;
				
				flagPIT = OFF;
				PIT.CH[0].TCTRL.B.TEN = 0;		// PIT Disable
			}
		}		
	}
	else if(flagDisp == 1)				// Display #1
	{
		if(SIU.GPDI[64].B.PDI == 0)		// SW1
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
			
			MyPosSum = 0;
			
			if(flagServo == ON)
			{
				flagServo = OFF;
			}
			else if(flagServo == OFF)
			{
				flagServo = ON;
			}
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2		Set Lane from center position
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
			idxURCL = idxSLineUR[0];
			idxULCL = idxSLineUL[0];
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3,		PIT Control
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
			if(flagPIT == OFF)
			{
				flagPIT = ON;
				PIT.CH[0].TCTRL.B.TEN = 1;		// PIT Enable
			}
			else if(flagPIT == ON)
			{
				flagIR = 0;
				flagSchool = 0;
				flagSchoolInt = 0;
				
				flagPIT = OFF;
				PIT.CH[0].TCTRL.B.TEN = 0;		// PIT Disable
			}
		}		
	}
	else if(flagDisp == 2)				// Display #2
	{	
		if(SIU.GPDI[64].B.PDI == 0)		// SW1,		DC Speed Up
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
			
			cntDesire += 10;
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2,		DC Speed Down
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
			cntDesire -= 10;
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3,		DC Motor Control
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
			if(flagDC == OFF)
			{
				Delay(5000000);
				Delay(5000000);
				
				flagIR = 0;
				cntIR = 0;
				flagSchool = 0;
				flagSchoolInt = 0;
				
				flagDC = ON;		// DC Motor Start
			}
			else if(flagDC == ON)
			{
				flagDC = OFF;		// DC Motor Stop
				
				cntDesire = 0;
				EMIOS_0_OPWM_Duty(24, 0);
				EMIOS_0_OPWM_Duty(25, 0);
				EMIOS_0_OPWM_Duty(26, 0);
				EMIOS_0_OPWM_Duty(27, 0);
			}
		}
	}
	else if(flagDisp == 3)				// Display #3, PID Gain
	{
		if(SIU.GPDI[64].B.PDI == 0)		// SW1
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
			
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
		}
	}
	else if(flagDisp == 4)				// Display #4, Distance Sensor
	{
		if(SIU.GPDI[64].B.PDI == 0)		// SW1
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
		
			distance += 5;
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
			distance -= 5;
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
		}
	}	
	else if(flagDisp == 5)				// Display #5, IR Sensor
	{
		if(SIU.GPDI[64].B.PDI == 0)		// SW1
		{
			Delay(500000);
			while(SIU.GPDI[64].B.PDI == 0) {};
			Delay(500000);
			
		}

		if(SIU.GPDI[65].B.PDI == 0)		// SW2
		{
			Delay(500000);
			while(SIU.GPDI[65].B.PDI == 0) {};
			Delay(500000);
			
		}
		
		if(SIU.GPDI[66].B.PDI == 0)		// SW3
		{
			Delay(500000);
			while(SIU.GPDI[66].B.PDI == 0) {};
			Delay(500000);
			
		}
	}
}

int main(void) 
{
	initModesAndClock();
	initPeriClkGen();
	disableWatchdog();

	initVar();
	initGain();

	initPads();
	SIU.PGPDO[2].R = 0x00000000;

	initEMIOS_0();
	/////////////////////////////////////////////////////////////////////////////
	// 2khz
//	initEMIOS_0_MCB(23, 500 - 1);					// Counter Bus A, 1000usec	
	// 1khz
	initEMIOS_0_MCB(23, 1000 - 1);					// Counter Bus A, 1000usec	
	// 500hz  -> Optimize(?)
//	initEMIOS_0_MCB(23, 2000 - 1);					// Counter Bus A, 1000usec	
	// 250hz
//	initEMIOS_0_MCB(23, 4000 - 1);					// Counter Bus A, 1000usec
	
	initEMIOS_0_OPWM(24, 0x00, 0);					// PG10, DCMotor1, 0~2000
	initEMIOS_0_OPWM(25, 0x00, 0);					// PG11, DCMotor1, 0~2000
	initEMIOS_0_OPWM(26, 0x00, 0);					// PG12, DCMotor2, 0~2000
	initEMIOS_0_OPWM(27, 0x00, 0);					// PG13, DCMotor2, 0~2000
	/////////////////////////////////////////////////////////////////////////////
	initEMIOS_0_MCB(0, 20000 - 1);					// CH0, Counter Bus B, 20000usec
	initEMIOS_0_OPWM(4, 0x01, SERVO_CENTER);		// CH4, PA4, Steering Servo, 1000 ~ 1300 ~ 1600
	/////////////////////////////////////////////////////////////////////////////
	initEMIOS_0_MCB(8, 65535 - 1);					// CH8, Counter Bus C, 65534
	initEMIOS_0_IPWM(9, 0x01, 1);					// CH9, PA9, Lidar IPWM
	SIU.PCR[9].B.PA = 1;            				// PA9, EMIOS_0[9] Function
	SIU.PCR[9].B.IBE = 1;          					// PA9, Input Buffer Enable
	
	initADC0();
	
	initLINFlex_0();
	initLINFlex_1();
	
	initDSPI();
	initLS7366();
	
	INTC_InitINTCInterrupts();
	INTC_InstallINTCInterruptHandler(ISR_PIT0, 59, 1);
	
//	if(SIU.GPDI[103].B.PDI == 0x00)						// Toggle 2
//	{
//		INTC_InstallINTCInterruptHandler(ISR_LIN0_UART_RECV, 79, 3);
//		INTC_InstallINTCInterruptHandler(ISR_LIN1_UART_RECV, 99, 2);
//	}
	
	enableIrq();
	
	PIT.PITMCR.R = 0x00000001;       				/* Enable PIT and configure timers to stop in debug mode */
	PIT.CH[0].LDVAL.R = (uint32_t) (CNT_CAM - 1);	// 0 ~ 518 Cycle Period Set 10us 
													// 1usec => 64, 10usec => 640, 20usec => 1280
	PIT.CH[0].TCTRL.R = 0x000000002; 				// Enable PIT interrupt & make PIT active to count
	
	Delay(10000000);
	
	initCLCD();
	SIU.GPDO[80].B.PDO = 1;		//PF0 DC Enable
	
	while(1)
	{
		MainLoop();
	}
}
