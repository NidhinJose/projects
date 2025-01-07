#include<lpc21xx.h>
void trans(char);
char recei();
void main()
{
	char a;
	PINSEL0=0x00000005;
	IODIR0=0x00000001;
	U0LCR=0x83;
	U0DLL=98;
	U0DLM=0;
	U0LCR=0x03;
	while(1)
	{
		a=recei();
		trans(a);
	}
}
void trans(char a                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         )
{
	U0THR=a;
	while((U0LSR&0x40)==0);
}
char recei()
{
	while((U0LSR&0x01)==0);
	return(U0RBR);
}