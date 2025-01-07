#include<lpc21xx.h>
void data(char);
void cmd(char);
void delay();
void main()
{
	int a,b,c,d,e;
	PINSEL0=0x00000000;
	PINSEL1=0x00400000;
	IODIR0=0x0000ffff;
	IODIR1=0xffff0000;
	//ADCR=0x01200001;
	cmd(0x38);
	cmd(0x80);
	cmd(0x01);
	cmd(0x06);
	cmd(0x0e);
		                                                                                                                                                                                                   
	while(1)
	{
		ADCR=0x01200001;
		while((ADDR&0x80000000)==0);
		a=(ADDR>>6)&0x3ff;
		b=(a/1000)+48;
		data(b);
		c=((a%1000)/100)+48;
		data(c);
		d=((a%100)/10)+48;
		data(d);
		e=(a%10)+48;
		data(e);
		cmd(0x01);
	}

}
void data(char a)
{
	IOCLR1=0xffffffff;
	IOSET1=a<<18;
	IOSET1=0x00030000;
	delay();
	IOCLR1=0x00020000;
	delay();
}
void cmd(char b)
{
	IOCLR1=0xffffffff;
	IOSET1=b<<18;
	IOSET1=0x00020000;
	delay();
	IOCLR1=0x00020000; 
	delay();
}
void delay()
{
	int i,j;
	for(i=0;i<300;i++)
	for(j=0;j<300;j++);
}