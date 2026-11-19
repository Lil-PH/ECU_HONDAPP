#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#define SSD1309_PIXEL_ON 1
#define SSD1309_SWITCHCAPVCC 0
Adafruit_ILI9341 tft(&SPI,46,10,-1);
class DashCanvas: public GFXcanvas1 {
 public:
 DashCanvas():GFXcanvas1(128,64){}
 bool begin(int,int){
 pinMode(45,OUTPUT); digitalWrite(45,LOW);
 SPI.begin(12,13,11,10); tft.begin(20000000); tft.invertDisplay(true); tft.setRotation(1); tft.fillScreen(0);
 digitalWrite(45,HIGH); return true;
 }
 void clearDisplay(){fillScreen(0);}
 void display(){
 static uint8_t previous[1024]={}; static bool first=true;
 if(!first && !memcmp(previous,getBuffer(),1024))return;
 // Envia uma faixa contigua por linha alterada, em vez de 8192 retangulos.
 static uint16_t pixels[512];
 const uint8_t* current=getBuffer();
 tft.startWrite();
 for(int y=0;y<64;y++) {
   int left=0,right=15;
   if(!first) {
     while(left<16 && current[y*16+left]==previous[y*16+left])++left;
     if(left==16)continue;
     while(right>left && current[y*16+right]==previous[y*16+right])--right;
   }
   const int width=(right-left+1)*16;
   for(int x=left*8;x<(right+1)*8;x++) {
     uint16_t color=(current[y*16+x/8] & (0x80>>(x&7)))?0xffff:0;
     int offset=(x-left*8)*2;
     pixels[offset]=pixels[offset+1]=color;
   }
   memcpy(pixels+width,pixels,width*sizeof(uint16_t));
   tft.setAddrWindow(32+left*16,25+y*2,width,2);
   tft.writePixels(pixels,width*2);
 }
 tft.endWrite();
 memcpy(previous,current,1024);first=false;
 }
};
DashCanvas display;
int touchKey=-1;
void touchSetup(){
 Wire.begin(16,15); Wire.setTimeOut(20); pinMode(18,OUTPUT);digitalWrite(18,LOW);delay(10);digitalWrite(18,HIGH);delay(200);
 tft.setTextColor(0xffff);tft.setTextSize(2);tft.setCursor(25,205);tft.print("<    MENU    >");
}
void touchUpdate(){
 static uint32_t lastRead=0;
 if(millis()-lastRead<10)return;
 lastRead=millis();
 touchKey=-1;Wire.beginTransmission(0x38);Wire.write(2);if(Wire.endTransmission(false))return;
 if(Wire.requestFrom(0x38,5)!=5)return;
 uint8_t n=Wire.read();uint8_t xh=Wire.read(),xl=Wire.read(),yh=Wire.read(),yl=Wire.read();
 (void)xh;(void)xl;
 if((n&15)==1){int y=((yh&15)<<8)|yl;touchKey=constrain(y/107,0,2);}
}
