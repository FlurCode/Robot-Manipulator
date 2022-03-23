```C
# Robot-Manipulator
This is code for easy robot manipulator with 4 degrees of freedom and simple app that was made by remote xy
#define dx 55 //длина схвата
#define dy 20 //выстота схвата
#define ao 65 //выстота 
#define ab 78 //длина звена
#define a01 180
#define a02 100
#define a03 75
#define AMOUNT 4  // кол-во серво
#include "ServoSmooth.h"
ServoSmooth servos[AMOUNT];
float g0ld = 0;
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// настройки соединения
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600

// конфигурация интерфейса
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
{ 255,4,0,16,0,44,0,15,31,1,
4,0,7,0,15,45,2,26,4,0,
39,0,15,44,2,26,4,0,38,47,
15,47,2,26,4,0,7,48,15,46,
2,26,67,4,2,47,59,5,2,26,
16 };

int prevS2, delta = 0;


// структура определяет все переменные и события вашего интерфейса управления
struct Slider{

// input variables
int8_t slider_1; // =0..100 положение слайдера
int8_t slider_2; // =0..100 положение слайдера
int8_t slider_3; // =0..100 положение слайдера
int8_t slider_4; // =0..100 положение слайдера

// output variables
char text_1[16];

// other variable
uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
struct MyStruct {
  int a1;
  int a2;
  int a3;
};

#pragma pack(pop)

void setup()
{

Serial.begin(9600);

RemoteXY_Init ();
servos[0].attach(A0, 350, 2300, 90);
servos[0].smoothStart();
servos[1].attach(A1, 350, 2300, 100);
servos[1].smoothStart();
servos[2].attach(A2, 350, 2300, 165);
servos[2].smoothStart();
servos[3].attach(A3, 350, 2300, 20);
servos[3].smoothStart();
servos[0].setSpeed(70); 
servos[0].setAccel(0.6);
servos[1].setSpeed(70);
servos[1].setAccel(0.6);
servos[2].setSpeed(70);
servos[2].setAccel(0.6);
servos[3].setSpeed(70);
servos[3].setAccel(0.6);

}

void loop()
{
servos[0].tick();
servos[1].tick();
servos[2].tick();
servos[3].tick();
RemoteXY_Handler ();
int s1 = RemoteXY.slider_1;
int s2 = RemoteXY.slider_2;
int s3 = RemoteXY.slider_3;
int s4 = RemoteXY.slider_4;
s1 = map(s1, 0, 100, -180, 180); //  x
s2 = map(s2, 0, 100, 0, 180);// y
s3 = map(s3, 0, 100, 0, 140); // z
s4 = map(s4, 0, 100, 20, 60);
MyStruct str;
str = invKinem(s1,s2,s3);
servos[0].setTargetDeg(str.a1);
servos[1].setTargetDeg(str.a2);
servos[2].setTargetDeg(str.a3);
servos[3].setTargetDeg(s4);
String string = String(String(s1) + " " + String(s2) + " " + String(s3)+ " " + String(s4));
string.toCharArray(RemoteXY.text_1,16);
delay(1);
}

MyStruct invKinem(int x, int y, int z) {
  MyStruct str;
  float ac = 0;
  float cd = 0;
  
  float g = 0;
 if (z>140){
    z=140.0;
    }
  if (z<0){
    z=0.0;
    }
  float l = sqrt(sq(x)+sq(y));
  if (l>190){
    l=190.0;
    g = g0ld;
    }
  if (l<120){
    l=120.0;
    g = g0ld;
    }
  if (l>120 and l<190){
  g = degrees(asin(y / l)); 
  g0ld = g;
  }

  if (x > 0) {
    g = 180 - g;
  };
  str.a1 = round(a01 - g);
  float x1 = l - dx;
  float z1 = z + dy;
  if (z1 > 65) {
    cd = z1 - ao;
    ac = sqrt(sq(x1)+sq(cd));
  };
  if (z1 < 65) {
    cd = ao - z1;
    ac = sqrt(sq(x1)+sq(cd));
  };

  if (z1 == 65) {
    ac = x1;
    cd = 0.0;
  };
  float abc = degrees(acos((2.0 * sq(ab) - sq(ac)) / (2.0 * sq(ab))));
  float bac = (180 - abc) / 2.0;
  float oc = sqrt(sq(x1) + sq(z1));
  float oac = degrees(acos((sq(ao) + sq(ac) - sq(oc)) /(2.0 * ao * ac)));
  float bao = bac + oac;
  float k = 180 - bao;
  str.a3 = round(a03 + abc - k);
  str.a2 = round(a02 + k);
  return str;

}
```
