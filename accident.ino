#include<LiquidCrystal.h>
LiquidCrystal lcd(13,12,11,10,9,8);
#define vibrate_sense 7
 
char str[70];
char *test="$GPGGA";      
char logitude[10];
char latitude[10];
 
int i,j,k;
int temp;
//int Ctrl+z=26;    //for sending msg
int led=13;
 String s,lat,lon;
void setup()
{
  lcd.begin(16,2);
  Serial.begin(9600);
  pinMode(vibrate_sense, INPUT);
  pinMode(led, OUTPUT);
  lcd.setCursor(0,0);
  lcd.print("Accident Detect");
  lcd.setCursor(0,1);
  lcd.print("Alert System");
  delay(3000);
}
 
void loop()
{
  if(Serial.available())
{
  if(Serial.read()=='$')
  {
    s=Serial.readStringUntil(',');
   
    if(s.compareTo("GPGGA")==0)
    {
      Serial.readStringUntil(',');
      lat=Serial.readStringUntil(',');
      Serial.readStringUntil(',');
      lon=Serial.readStringUntil(',');
      Serial.print("Lat : ");
      Serial.print(lat);
      Serial.print("Lon : ");
      Serial.println(lon);
      }
    
    }
  }
  if (digitalRead(vibrate_sense)==0)
  {
    
    for(i=18;i<27;i++)          //extract latitude from string
    {
      latitude[j]=str[i];
      j++;
    }
   
    for(i=30;i<40;i++)          //extract longitude from string
    {
      logitude[k]=str[i];
      k++;
    }
   
    lcd.setCursor(0,0);        //display latitude and longitude on 16X2 lcd display 
    lcd.print("Lat(N)");
    lcd.print(lat);
    lcd.setCursor(0,1);
    lcd.print("Lon(E)");
    lcd.print(lon);
    delay(100);
    lcd.clear();
    lcd.print("Sending SMS");
    Serial.begin(9600);
    Serial.println("AT+CMGF=1");    //select text mode
    delay(10);
    Serial.println("AT+CMGS=\"6238287100\"");
    Serial.println("ATD6238287100;"); // enter receipent number
    Serial.println("Vehicle Accident occured:");
    Serial.print("Latitude(N): ");             //enter latitude in msg
    Serial.println(lat);                  //enter latitude value in msg
    Serial.print("Longitude(E): ");            //enter Longitude in Msg
    Serial.println(lon);                  //enter longitude value in msg
    Serial.print("http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
    Serial.println(lat);
    Serial.println("+");
    Serial.println(lon);
    Serial.write(26);                      //send msg  Ctrl+z=26
    lcd.print("SMS Sent");
    temp=0;
    i=0;
    j=0;
    k=0;
    delay(20000);                        // next reading within 20 seconds
    Serial.begin(9600);
  }
}
 
/*void serialEvent()
{
  while (Serial.available())            //Serial incomming data from GPS
  {
    //if(m.available())
{
  if(m.read()=='$')
  {
    s=m.readStringUntil(',');
   
    if(s.compareTo("GPGGA")==0)
    {
      m.readStringUntil(',');
      lat=m.readStringUntil(',');
      m.readStringUntil(',');
      lon=m.readStringUntil(',');
      Serial.print("Lat : ");
      Serial.print(lat);
      Serial.print("Lon : ");
      Serial.println(lon);
      }
    
    }
  }
    char inChar = (char)Serial.read();
     str[i]= inChar;                    //store incomming data from GPS to temparary string str[]
     i++;
     if (i < 7)                      
     {
      if(str[i-1] != test[i-1])         //check for right string
      {
        i=0;
      }
     }
    if(i >=60)
    {
     break;
    }
  }
}*/
