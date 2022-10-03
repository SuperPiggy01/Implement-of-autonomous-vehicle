#include <stdio.h>
#include <IRremote.h>
#include "SR04.h"
#include <Servo.h>

int debug;
//===========mode==========
int mode=0;

//=======motor=============
#define ENA 5
#define ENB 6
#define PIN_Motor_STBY 3
#define PIN_Motor_R_side 7
#define PIN_Motor_L_side 8
int wv,DegRot=90,right,left,rv;
float d=4,v=43;//116.84>V>42.33


//=======IRremote===========
int IRpin = 9;
IRrecv IR(IRpin);
decode_results cmd;

//============Ultrasonic ======================
#define TRIG_PIN 13
#define ECHO_PIN 12
#define MAX_DISTANCE 200
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

//============Servo======================
Servo myservo;
#define PIN_Servo 10
#define Servo_init_val 90
long index[9] = {0 ,0, 0, 0, 1, 0, 0, 0, 0};
char cal_direction;
int cal_angle;



class carDirect{
  public:
    void init()
      { pinMode(ENA,OUTPUT);
        pinMode(ENB,OUTPUT);
        pinMode(PIN_Motor_STBY,OUTPUT);
        pinMode(PIN_Motor_R_side,OUTPUT);
        pinMode(PIN_Motor_L_side,OUTPUT);
        digitalWrite(ENA,HIGH);
        digitalWrite(ENB,HIGH);
      }
     void forward(float d, float v){
        float t;
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,HIGH);
        digitalWrite(PIN_Motor_L_side,HIGH);
        t=d/v*1000.;
        delay(t);
        stop();
      }
      void back(float d, float v){
        float t;
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,LOW);
        digitalWrite(PIN_Motor_L_side,LOW);
        t=d/v*1000.;
        delay(t);
        stop();
      }
      void right(int degree, int wv){
        float t;
        stop();
        delay(100);
        analogWrite(ENA,125);
        analogWrite(ENB,125);
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,LOW);
        digitalWrite(PIN_Motor_L_side,HIGH);
        t=degree/232.26*1000.;
        delay(t);
        stop();
        analogWrite(ENA,wv);
        analogWrite(ENB,wv);
      }
      void left(int degree, int wv){
        float t;
        stop();
        delay(100);
        analogWrite(ENA,125);
        analogWrite(ENB,125);
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,HIGH);
        digitalWrite(PIN_Motor_L_side,LOW);
        t=degree/232.26*1000.;
        delay(t);
        stop();
        analogWrite(ENA,wv);
        analogWrite(ENB,wv);
      }
      void stop(){
        digitalWrite(PIN_Motor_STBY,LOW);
      }
      void setCarSpeed(int leftVal,int rightVal){
        analogWrite(ENA,leftVal);
        analogWrite(ENB,rightVal);
      }
      void calF(){
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,HIGH);
        digitalWrite(PIN_Motor_L_side,HIGH);
        delay(1500);
        stop();
        }
      void calB(){
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,LOW);
        digitalWrite(PIN_Motor_L_side,LOW);
        delay(1500);
        stop();
        }
      void calR(int wv){
        stop();
        analogWrite(ENA,125);
        analogWrite(ENB,125);
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,HIGH);
        digitalWrite(PIN_Motor_L_side,LOW);
        delay(1550);
        analogWrite(ENA,wv);
        analogWrite(ENB,wv);
        stop();
      }
      void calL(int wv){
        analogWrite(ENA,125);
        analogWrite(ENB,125);
        digitalWrite(PIN_Motor_STBY,HIGH);
        digitalWrite(PIN_Motor_R_side,LOW);
        digitalWrite(PIN_Motor_L_side,HIGH);
        delay(5000);
        analogWrite(ENA,wv);
        analogWrite(ENB,wv);
        stop();
      } 
};
carDirect car_Direct;

class IRreceive{
  public:
    
    void init(){ 
      Serial.begin(9600);
      IR.enableIRIn();
      IR.blink13(true);//debag mode
      }
      
    void IRdirect(){ 
      if (cmd.value==0xFF629D)
      { car_Direct.forward(d,v);
        Serial.println("Up");}
      if (cmd.value==0xFFA857)
      { car_Direct.back(d,v);
        Serial.println("Down");}
      if (cmd.value==0xFFC23D)
      { DegRot=45;
        car_Direct.right(DegRot,wv);
        Serial.println("Right");}
      if (cmd.value==0xFF22DD)
      { DegRot=45;
        car_Direct.left(DegRot,wv);
        Serial.println("Left");}
      if (cmd.value==0xFF02FD)
      { mode=mode+1;
        if (mode>=2){
          mode=0;}
        Serial.println(mode);}
      if (cmd.value==0xFF6897)
      { Serial.println("1");}
      if (cmd.value==0xFF9867)
      { Serial.println("2");}
      if (cmd.value==0xFFB04F)
      { Serial.println("3");}
      if (cmd.value==0xFF30CF)
      { Serial.println("4");}
      if (cmd.value==0xFF18E7)//??
      { Serial.println("5");}
      if (cmd.value==0xFF7A85)
      { Serial.println("6");}
      if (cmd.value==0xFF10EF)
      { Serial.println("7");}
      if (cmd.value==0xFF38C7)
      { Serial.println("8");}
      if (cmd.value==0xFF5AA5)
      { Serial.println("9");}
      if (cmd.value==0xFF42BD)
      { Serial.println("*");
        Serial.println("Set Speed");
        delay(500);
        IR.resume();
        while (IR.decode(&cmd)==0){
          Serial.println("nothing");}
        Serial.println(cmd.value,HEX);
          if (cmd.value==0xFF6897){
            rv=1;
            Serial.println("rv=1");}
          if (cmd.value==0xFF9867){
            rv=2;
            Serial.println("rv=2");}
          if (cmd.value==0xFFB04F){
            rv=3;
            Serial.println("rv=3");}
          if (cmd.value==0xFF30CF){
            rv=4;
            Serial.println("rv=4");}
          if (cmd.value==0xFF18E7){
            rv=5;
            Serial.println("rv=5");}
          if (cmd.value==0xFF7A85){
            rv=6;
            Serial.println("rv=6");}
          if (cmd.value==0xFF10EF){
            rv=7;
            Serial.println("rv=7");}
          if (cmd.value==0xFF38C7){
            rv=8;
            Serial.println("rv=8");}
          if (cmd.value==0xFF5AA5){
            rv=9;
            Serial.println("rv=9");}
           else{
            Serial.println("Invalid CMD");
            Serial.println(cmd.value,HEX);}
            v=((94-33)/9.)*rv+33;
            }
      if (cmd.value==0xFF4AB5)
      { Serial.println("0");}
      if (cmd.value==0xFF52AD)
      { Serial.println("#");
        Serial.println("Set Distance");
        delay(500);
        IR.resume();
        while (IR.decode(&cmd)==0){
          Serial.println("nothing");}
        Serial.println(cmd.value,HEX);
          if (cmd.value==0xFF6897){
            d=10;
            Serial.println("Distance=1");}
          if (cmd.value==0xFF9867){
            d=20;
            Serial.println("Distance=2");}
          if (cmd.value==0xFFB04F){
            d=30;
            Serial.println("Distance=3");}
          if (cmd.value==0xFF30CF){
            d=40;
            Serial.println("Distance=4");}
          if (cmd.value==0xFF18E7){
            d=50;
            Serial.println("Distance=5");}
          if (cmd.value==0xFF7A85){
            d=60;
            Serial.println("Distance=6");}
          if (cmd.value==0xFF10EF){
            d=70;
            Serial.println("Distance=7");}
          if (cmd.value==0xFF38C7){
            d=80;
            Serial.println("Distance=8");}
          if (cmd.value==0xFF5AA5){
            d=90;
            Serial.println("Distance=9");}
          else{
              Serial.println("Invalid CMD");
              Serial.println(cmd.value,HEX);}}
      else{
        Serial.println("Invalid CMD");
        Serial.println(cmd.value,HEX);
        }
      delay(500);
      IR.resume();
     }
     void IRread(){
      if(IR.decode(&cmd)!=0){
      Serial.println(cmd.value,HEX);
      IRdirect();}
      }
        
};
IRreceive IRreceiveCenter;

class Radar{
  public:
    void init()
      { myservo.attach(PIN_Servo,500,2400);//connect pin 9 with the control line(the middle line of Servo) 
        myservo.write(Servo_init_val);}// move servos to center position -> 90°
    void scan_rtol()
    {  int i,k=1,l=9,BigNum;
       long j;
        for(i=30;i<=150;i=i+15){
          j=sr04.Distance();
//          Serial.println(String(j)+"cm"); 
          index[k] = j;
          k=k+1;
          myservo.write(i);
          delay(300);}
          for(l=9;l>=1;l=l-1){ 
          Serial.print(index[l]);Serial.print(",");}
          Serial.println("");
          BigNum = compareArray();
          Serial.println(BigNum);
          Serial.println(index[BigNum]);
          cal_direction = derictionCal(BigNum);
          Serial.println(cal_direction);
          cal_angle = angle(BigNum);
          Serial.println(cal_angle);}
    void scan_ltor(){  
      int i=165,k=9,l=9,BigNum;
      long j;
        if (i>=150){
          for(i=150;i>=30;i=i-15){
          j=sr04.Distance();
          //Serial.println(String(j)+"cm");
          index[k] = j;
          k=k-1;  
          myservo.write(i);// move servos to center position -> 90° 
          delay(300);}
          for(l=9;l>=1;l=l-1){ 
          Serial.print(index[l]);Serial.print(",");}
          Serial.println("");
          BigNum = compareArray();
          Serial.println(BigNum);
          Serial.println(index[BigNum]);
          cal_direction = derictionCal(BigNum);
          Serial.println(cal_direction);
          cal_angle = angle(BigNum);
          Serial.println(cal_angle);
          }
      }
      int compareArray(){
        int temp,l;
          temp=l=0;
          for(l=1;l<=9;l=l+1){ 
          if (index[temp]<index[l])
          {temp=l;}
          }
          return temp;
        }
      char derictionCal(int Num){
        float shield;
        int wall = 8;
        shield = 5;
        if ((index[5]<=(wall))&&(index[1]<=(wall))&&(index[2]<=(wall))&&(index[3]<=(wall))&&(index[4]<=(wall))&&(index[6]<=(wall))&&(index[7]<=(wall))&&(index[8]<=(wall))&&(index[9]<=(wall))){
          return 'B';}
        else if ((index[5]>=(2*d))&&(index[1]>=(shield))&&(index[2]>=(shield))&&(index[3]>=(shield))&&(index[7]>=(shield))&&(index[8]>=(shield))&&(index[9]>=(shield))){
          return 'F';}
        else if (Num<5){
          return 'R';}
        else if (Num>5){
          return 'L';}
        else{return 'F';}
        }
      int angle(int Num){ 
        if (Num<5){
        return 90-(Num+1)*15;}
        if (Num>5){
        return (Num+1)*15-90;}
        else{return 0;}
        }   
};
Radar radar;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  car_Direct.init();
  IRreceiveCenter.init();
  radar.init();
}

void loop() {
  //===========Motor Contrl=============
  // re_cal_wv();
  //  car_Direct.forward(d,v);
  //  car_Direct.back(d,v);
  //  car_Direct.right(DegRot,wv);
  //  car_Direct.left(DegRot,wv);
  //  car_Direct.stop();
  
  //===========Motor Contrl=============

  //===========IR remote===============
//  if(IR.decode(&cmd)!=0){
//     Serial.println(cmd.value,HEX);
//     IRreceiveCenter.IRdirect(cmd.value);}
      
  //===========IR remote===============

//============Ultrasonic ======================
//    long a;
//    a=sr04.Distance();
//    Serial.println(String(a)+"cm");
//============Ultrasonic ======================

//  radar.scan();

//  digitalWrite(LED_BUILTIN, HIGH);
//  //Serial.println("1");   // turn the LED on (HIGH is the voltage level)
//  delay(1000);
//  // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);
//  //Serial.println("0");    // turn the LED off by making the voltage LOW
//  delay(1000);
// wait for a second
//===================program start==================================
//===================step-1:cal_direction===========================
  re_cal_wv();  
  if (mode==0){
    IRreceiveCenter.IRread();  
    radar.scan_rtol();
    IRreceiveCenter.IRread();
//  decision_make();
    IRreceiveCenter.IRread();
    radar.scan_ltor();
    IRreceiveCenter.IRread();
//  decision_make();
  }
  if (mode==1){
    myservo.write(90);// move servos to center position -> 90° 
    delay(100);
    IRreceiveCenter.IRread();  
  }


}
void re_cal_wv(){  
  wv = (v+5.741)/0.4807;
  left=right=wv;
  car_Direct.setCarSpeed(left,right);}
void decision_make(){  
   if (cal_direction =='F'){
    if(d>3){
      d=index[5];
      d=0.5*d;}
      else{d=5;}
      car_Direct.forward(d,v);}
   if (cal_direction =='R'){
      DegRot = cal_angle;
      cal_degRot();
      car_Direct.right(DegRot,wv);
      d=3;
      car_Direct.forward(d,v);}
   if (cal_direction =='L'){
      DegRot = cal_angle;
      cal_degRot();
      car_Direct.left(DegRot,wv);
      d=3;
      car_Direct.forward(d,v);}
   if (cal_direction =='B'){
      DegRot = 180;
      d=5;
      car_Direct.back(d,v);
      car_Direct.left(DegRot,wv);}
}
void cal_degRot(){
  if (DegRot>=15&&DegRot<=30){
    DegRot=DegRot+15;}
  if (DegRot>=45&&DegRot<=60){
    DegRot=DegRot+15;} 
  }
