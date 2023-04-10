#include <SoftwareSerial.h> // 상위 제어기와 통신하기 위한 라이브러리 

int encoder_pot = A8;    // 조향모터 가변저항  
int val;  // 조향모터의 PID제어를 위한 변수
int encoder_val;
float kp = 0.2;
float ki = 0.00000 ;
float kd = 2.00;
float Theta, Theta_d;
int dt;
unsigned long t;
unsigned long t_prev = 0;
int val_prev =0;
float e, e_prev = 0, inte, inte_prev = 0;
float Vmax = 24;
float Vmin = -24;
float V = 0.1;
void SteerCon(); // 조향 모터의 PID제어 함수

const byte interruptPinA = 2;  // 구동모터의 엔코더센서 핀번호
const byte interruptPinB = 3;
volatile long EncoderCount = 0; // 엔코더 값 변수

const byte PWMPin = 12; // 조향 모터의 모터드라이브 PWM,DIR 핀번호 
const byte DirPin1 = 13;

int PWMone = 9; // 구동 모터의 모터드라이브 PWM1,PWM2,DIR1,DIR2 핀번호
int Dirone = 8;
int PWMtwo = 11;
int Dirtwo = 10;
void mDrive(); // 구동 모터의 OPEN LOOP 제어 함수 

int Speed, Steer,HSteer,HSpeed; // 상위 제어기와 통신을 위한 변수 
byte State, ESTOP, Mode;
byte ALIVE = 0;
char SendMessage[17];

int Thro,Aux,Gear,ALIE,T,A,G,AL; // 조종기를 위한 변수
void controller(); // 조종기 통신 함수 


void setup() {

  Serial.begin(115200); // 시리얼 통신을 위한 Baudrate 
  pinMode(DirPin1, OUTPUT); // 조향 모터 
  pinMode(PWMone,OUTPUT);
  pinMode(Dirone,OUTPUT); // 구동 모터
  pinMode(PWMtwo,OUTPUT);
  pinMode(Dirone,OUTPUT);
  pinMode(interruptPinA, INPUT_PULLUP); // 엔코더 핀 
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE); // 엔코더 값을 읽기 위한 인터럽트 서비스 루틴 
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);  

}


void loop() {
  controller();  // 조종기 명령값 수신 

if(State == 1){ //AUTO상태일때 
   if(Serial.available()){
    String str = Serial.readStringUntil('\n');  // HLV to PCU Packet: S T X AorM ESTOP SPEED(2byte) STEER(2byte) ALIVE ETX
    HSpeed = word(str[6],str[7]); //상위 제어기에서 SPEED값 저장
    HSteer = word(str[8],str[9]); //상위 제어기에서 STEER값 저장 
    
    Mdrive(HSpeed,0);    //SPEED값 제어
    SteerCon(HSteer);    //STEER값 제어
    SendMessage[6] = (HSpeed >> 8) | 0x00;  // PCU to HLV 통신 프로토콜 
    SendMessage[7] = HSpeed | 0x00;
    SendMessage[8] = (HSteer >> 8) | 0x00;
    SendMessage[9] = HSteer | 0x00;
   }
   else{  //AUTO상태이지만 통신이 불가능할때 속도 0 조향 0
     Mdrive(0,0);  
     SteerCon(500);
   }
  }
else{ // MANUAL상태일때 플랫폼 제어 
  Mdrive(Speed,Mode); //SPEED값 제어
  SteerCon(Steer); //STEER값 제어
 
  SendMessage[6] = (Speed >> 8) | 0x00; // PCU to HLV 통신 프로토콜
  SendMessage[7] = Speed | 0x00;
  SendMessage[8] = (Steer >> 8) | 0x00;
  SendMessage[9] = Steer | 0x00;

  }
    // PCU to HLV Packet: S T X AorM ESTOP GEAR SPEED(2byte) STEER(2byte) ENC(4byte) ALIVE ETX
    SendMessage[0] = 'S';
    SendMessage[1] = 'T';
    SendMessage[2] = 'X';
    SendMessage[3] = State; 
    SendMessage[4] = ESTOP;
    SendMessage[5] = Mode;
    SendMessage[10] = EncoderCount>>24 | 0x00 ;
    SendMessage[11] = EncoderCount>>16 | 0x00;
    SendMessage[12] = EncoderCount>>8 | 0x00;
    SendMessage[13] = EncoderCount | 0x00;
    SendMessage[14] = ALIVE | 0x00;
    SendMessage[15] = 0x0D;
    SendMessage[16] = 0x0A;
    Serial.write(SendMessage,17);
    ALIVE = ALIVE +1;
  delay(1);

}

void SteerCon(float Q) { // 조향모터의 제어를 위한 함수, Q = 조향 목표값  
    val = Q;                           
    if(val>800){  // 조향 최대값 제한
      val=800;
    }
    else if(val<200){ // 조향 최소값 제한 
      val= 200;
    }
    encoder_val =analogRead(encoder_pot);               // 현재 가변저항 값(조향모터 각도)
    t = millis();
    dt = (t - t_prev);                                  // PID제어를 위한 시간값
    Theta = val;                                        // Theta= 목표 각도
    Theta_d = encoder_val;                              // Theta_d= 현재 각도 

    e = Theta_d - Theta;                                // Error
    inte = inte_prev + (dt * (e + e_prev) / 2);         // Integration of Error
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // PID 계산 식

    if (V > Vmax) { // PID 계산값 최대 제한
        V = Vmax;
       inte = inte_prev;
     }
    if (V < Vmin) { // PID 계산값 최소 제한
      V = Vmin;
      inte = inte_prev;
      val_prev= val;
     }
  int PWMval = int(255 * abs(V) / Vmax); // PID제어를 이용한 조향 모터 속도 설정  
  
  if (PWMval > 200) { // 조향 모터 속도 최대값 제한 
    PWMval = 200;
  }
  if (V > 0.5) { // 조향 모터 제어 
    digitalWrite(DirPin1, HIGH);
    analogWrite(PWMPin, PWMval);
  }
  else if (V < -0.5) { // 조향 모터 제어 
    digitalWrite(DirPin1, LOW);
    analogWrite(PWMPin, PWMval);
  }
  else { // 조향 모터 진동 방지
    digitalWrite(DirPin1, LOW);
    analogWrite(PWMPin, 0);
  }
   t_prev = t;
    inte_prev = inte;
    e_prev = e;

}

void Mdrive(int q,int w){ //구동모터의 제어를 위한 함수, q = Speed, w = 정,역방향 지정
  if(w == 0){ // 모터 정회전(전진)
    digitalWrite(Dirtwo,0);
    analogWrite(PWMtwo,q);
    digitalWrite(Dirone,1);
    analogWrite(PWMone,q);
  }
  
  else if(w == 1){ // 모터 정지(중립)
    digitalWrite(Dirtwo,0);
    analogWrite(PWMtwo,0);
    digitalWrite(Dirone,0);
    analogWrite(PWMone,0);
  }
  
  else if(w == 2){ // 모터 역회전(후진)
    digitalWrite(Dirtwo,1);
    analogWrite(PWMtwo,q);
   digitalWrite(Dirone,0);
   analogWrite(PWMone,q);
  }
}

void ISR_EncoderA() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }
}
void ISR_EncoderB() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }
}


void controller() {
    Aux = pulseIn(A0, HIGH, 50000);   //  아날로그1번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Aux변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Aux변수에 저장된다.
    Thro = pulseIn(A5, HIGH, 50000);  //  아날로그5번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Thro변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Thro변수에 저장된다.
    Gear = pulseIn(A1, HIGH, 50000);  //  아날로그2번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Gear변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Gear변수에 저장된다.
    ALIE = pulseIn(A4, HIGH, 50000);  //  아날로그3번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Rudd변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Rudd변수에 저장된다.
    
    T = Thro/50; Thro = T*50; A = Aux/50; Aux = A*50; G = Gear/50; Gear = G*50; AL = ALIE/50; ALIE = AL*50; // 조종기 신호 안정화
    
    if(ALIE>1700){// 조종기 ALIE(조향) 신호 최대 최소값 제한
       ALIE=1700; 
     } 
    else if(ALIE<1000){
      ALIE = 1000; 
     } 
    
    if(Thro>1850){ // 조종기 THRO(구동) 신호 최대값 제한, Throttle Cut할때 ESTOP신호 주도록 설정
      Thro=1850; 
     }  
    else if(Thro<1000){ 
      ESTOP = 1;
     } 
    else{
      ESTOP=0;
     }  
    
    Speed = map(Thro, 1050, 1850, -255, 255); // 조종기 THRO 신호를 Speed값으로 변환 
    
    if(Speed>255){ // Speed값 최대 최소값 제한 및 ESTOP상황에서 멈추도록 설정
      Speed = 255; 
     } 
    else if(Speed<10 or ESTOP == 1 ){
      Speed = 0; 
     } 
    
    Steer = map(ALIE, 1200, 1700,1023, 0); // 조종기 ALIE 신호를 Steer값으로 변환
    
    if(Steer>800){ // Steer값 최대 최소값 제한
      Steer = 800; 
     } 
    else if(Steer<200){ 
      Steer = 200; 
     } 
    else if(Steer>480 and Steer<520){ // 조향 유지를 위한 DeadZone
      Steer = 500;
     } 
    
    if(Aux>1700){ // 조종기 Aux신호를 주행 상태결정하는 Mode변수로 치환, Mode=0:직진, Mode=1:중립, Mode=2:후진
      Mode = 0;
     } 
    else if(Aux<1700 and Aux>1300){ 
      Mode = 1; 
      Speed = 0; 
     } 
    else if(Aux<1300){ 
      Mode = 2; 
     }  
    
    if(Gear>500 and Aux > 500){ // 조종기가 On일때 
     if(Gear<1200){ //조종기 Gear신호를 통해 자율주행모드, 수동주행모드 설정. State = 1:Auto mode, State = 0: Manual mode
        State = 1;
      } 
     else{ 
        State = 0; 
       } 
      }
    else{ // 조종기가 Off일때 속도 0 조향 0
      Speed = 0; 
      Steer = 500; 
    }  
}
