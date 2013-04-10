//Define PIN LOCATIONS
#define M1 0
#define M2 1
#define M3 2
#define M4 3
#define M5 4

#define M1_PWM_PIN   3
#define M2_PWM_PIN 5
#define M3_PWM_PIN 6
#define M4_PWM_PIN 10
#define M5_PWM_PIN 11

const int MOTOR_PWM_PINS[] = {
  M1_PWM_PIN,
  M2_PWM_PIN,
  M3_PWM_PIN,
  M4_PWM_PIN,
  M5_PWM_PIN};


#define M1_DIR_PIN 2
#define M2_DIR_PIN 4
#define M3_DIR_PIN 7
#define M4_DIR_PIN 8
#define M5_DIR_PIN 12
const int MOTOR_DIR_PINS[] = {
  M1_DIR_PIN,
  M2_DIR_PIN,
  M3_DIR_PIN,
  M4_DIR_PIN,
  M5_DIR_PIN};

#define M1_POS_PIN 0
#define M2_POS_PIN 1
#define M3_POS_PIN 2
#define M4_POS_PIN 3
#define M5_POS_PIN 4

//DEFINES SERIAL CONTROl
#define CMD_START 0
#define CMD_MODE 1
#define CMD_SET_MOTOR 2
#define CMD_SET_TARGET 3
#define CMD_SET_GRIPPER 4

int pos_Targets[] = {
  0,0,0,0};
boolean cur_Direction[] = {
  0,0,0,0};

int prevPostion[] = {
  0,0,0,0};
int prevError[] = {
  0,0,0,0};
int Kp[] = {
  2,2,2,2};
int Ki[] = {
  1,1,1,1};

int FORWARD  = 0xFF;
int  REVERSE = 0xBB;
int START_CMD = 0x55;
int SECOND_CMD = 0xAA;
//Serial Switch State Management
int state = 0;
int c = 0;
int m = 0; //Motor to control
int positionTarget = 0;

int maxPos[] = {
  1,2,3,4,5};
int minPos[] = {
  1,2,3,4,5};
int M2PWM = 0;

void setup() 
{ 
  Serial.begin(9600);
  Serial.write("Starting");
  setupPins();
  initPostions();
} 


void loop() 
{ 

//MOTOR CONTROL
  motorControl();

//STREAM DATA
  streamData();

//Manage Serial
  while(Serial.available()){
    delay(10);
    int c = Serial.read();
    Serial.println(c);
    receiveSerial(c);
  }


} 

void setupPins(){
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M5_DIR_PIN, OUTPUT);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT);
  pinMode(M4_PWM_PIN, OUTPUT);
  pinMode(M5_PWM_PIN, OUTPUT); 
}
void initPostions(){
  int i;
  int pos;
  for(i=0;i<4;i++){
    pos = analogRead(i);
    prevPostion[i] = pos;
    pos_Targets[i] = pos;
  }
}

unsigned int calculatePWM(int motorNum){
  int curPos;
  int error;
  int pControl;
  int iControl;
  int Control;
  if(motorNum>4){
    return 0;
  }
  //Calcualte Porportional Gain
  curPos = analogRead(motorNum);
  error  = pos_Targets[motorNum]-curPos;
  Serial.println(pos_Targets[motorNum]);
  //Deterimine Direction
  //Create Proportional Gain
  prevError[motorNum] = prevError[motorNum]+error;
  iControl = Ki[motorNum]*prevError[motorNum];
  if(error<0)
  {
    cur_Direction[motorNum] = 0;
  }
  else{
    cur_Direction[motorNum] = 1;
  }

  pControl = abs(Kp[motorNum]*error);
  Control = abs(iControl)+ pControl;
  if (abs(error)<3){
    prevError[motorNum] = 0;
    Control = 0;
  }
  return Control;
}

void receiveSerial(int rc){
  int newPos;
  switch(state) {
  case CMD_START:                         // First byte of packet must be 0x55 'U'
    if(rc == START_CMD) ++state;
    break;
  case CMD_MODE:                         // Second byte of packet must be 0xAA	                
    if(rc == SECOND_CMD) {
      m = 0;
      //checksum = 0;
      ++state;
    } 
    else {
      Serial.print("CASE RESET");
      state = 0;
    }
    break;
  case CMD_SET_MOTOR:                         //Get Motor	                
    if(rc<4)
    {
      m=rc;
      ++state;
    }
    else{
      state = 0;
      Serial.print("CASER RESET");
    }
    break;
  case CMD_SET_TARGET:
    newPos = (int)rc;
    //Serial.print(newPos);				 //Get Target            	
    pos_Targets[m] = newPos*4;
    state = 0;
    Serial.print("Motor "); 
    Serial.print(m, DEC);
    Serial.print(" set at "); 
    Serial.println(pos_Targets[m]);
    break;
  }
}

void motorControl(void){
  int motorNum;
  int motorPWM;
  for(motorNum=0;motorNum<4;motorNum++){
    motorPWM = calculatePWM(motorNum);

    if (motorPWM>255) motorPWM = 255;
    analogWrite(MOTOR_PWM_PINS[motorNum],motorPWM);

    if(cur_Direction[M2]){
      digitalWrite(MOTOR_DIR_PINS[motorNum], LOW);
    }
    else{
      digitalWrite(MOTOR_DIR_PINS[motorNum], HIGH);
    }
  }
}

void streamData(void){
  int motorNum;
  for(motorNum=0;motorNum<4;motorNum++){
    Serial.print(analogRead(motorNum));
    Serial.print("\t");
  }
  Serial.println();
}




