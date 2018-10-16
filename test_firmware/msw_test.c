#include <system.h>
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

#define P_OUTA 		latc.0
#define P_OUTB 		latc.2
#define P_OUTC 		latc.3
#define P_OUTD 		latc.4
#define P_OUTE 		lata.2  //CCP3
#define P_OUTF 		latc.1	//CCP4
#define P_OUTG 		latc.5	//CCP1
#define P_OUTH 		lata.5	//CCP2
#define P_LED 		lata.4
void main()
{
	osccon = 0b01111010;
	trisa =  0b11001011;
	trisc =  0b11000000;

	ansela = 0b00000000;
	anselc = 0b00000000;

	int x=0;
	for(;;) {
		P_OUTA = !!(x==0);
		P_OUTB = !!(x==1);
		P_OUTC = !!(x==2);
		P_OUTD = !!(x==3);
		P_OUTE = !!(x==4);
		P_OUTF = !!(x==5);
		P_OUTG = !!(x==6);
		P_OUTH = !!(x==7);
		P_LED = !!(x==8);
		if(++x>8) x=0;
		delay_ms(200);
		
	}
}
