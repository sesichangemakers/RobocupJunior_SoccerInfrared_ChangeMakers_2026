#include <Arduino.h>
#include <math.h>

// ===== Config =====
#define SENSOR_COUNT 12
#define SAMPLE_COUNT 500
#define HISTORY_SIZE 10
#define NOISE_THRESHOLD 10
#define EMA_ALPHA 0.15f
#define WEIGHT_EXPONENT 2.0f

const uint8_t sensorPins[SENSOR_COUNT]={6,7,46,11,12,13,14,48,45,35,38,39};

float cosTable[SENSOR_COUNT], sinTable[SENSOR_COUNT];
uint16_t rawCount[SENSOR_COUNT];
uint16_t filtCount[SENSOR_COUNT];
uint16_t hist[SENSOR_COUNT][HISTORY_SIZE];
uint8_t histIdx=0; bool histFull=false;

float emaX=0,emaY=0; bool emaInit=false;
float ballAngle=-1;
float ballDistance=0;

void sampleSensors(){
  memset(rawCount,0,sizeof(rawCount));
  for(int s=0;s<SAMPLE_COUNT;s++){
    for(int i=0;i<SENSOR_COUNT;i++){
      if(digitalRead(sensorPins[i])==LOW) rawCount[i]++;
    }
  }
}

void updateHistory(){
  for(int i=0;i<SENSOR_COUNT;i++) hist[i][histIdx]=rawCount[i];
  histIdx=(histIdx+1)%HISTORY_SIZE;
  if(histIdx==0) histFull=true;
  int n=histFull?HISTORY_SIZE:histIdx;
  if(n==0)n=1;
  for(int i=0;i<SENSOR_COUNT;i++){
    uint32_t sum=0;
    for(int j=0;j<n;j++) sum+=hist[i][j];
    filtCount[i]=sum/n;
  }
}

int maxSensor(){
  int idx=0; uint16_t v=0;
  for(int i=0;i<SENSOR_COUNT;i++) if(filtCount[i]>v){v=filtCount[i];idx=i;}
  return idx;
}

void calcBall(){
  int m=maxSensor();
  if(filtCount[m]<NOISE_THRESHOLD){ballAngle=-1;ballDistance=0;return;}
  float sx=0,sy=0; uint32_t dsum=0;
  for(int k=-2;k<=2;k++){
    int idx=(m+k+SENSOR_COUNT)%SENSOR_COUNT;
    float w=pow((float)filtCount[idx],WEIGHT_EXPONENT);
    sx+=cosTable[idx]*w;
    sy+=sinTable[idx]*w;
    dsum+=filtCount[idx];
  }
  if(!emaInit){emaX=sx;emaY=sy;emaInit=true;}
  emaX=EMA_ALPHA*sx+(1-EMA_ALPHA)*emaX;
  emaY=EMA_ALPHA*sy+(1-EMA_ALPHA)*emaY;
  float a=atan2f(emaY,emaX)*180.0f/PI;
  if(a<0)a+=360;
  ballAngle=a;
  ballDistance=dsum/5.0f;
}

void setup(){
  Serial.begin(115200);
  for(int i=0;i<SENSOR_COUNT;i++){
    pinMode(sensorPins[i],INPUT);
    float r=(i*30.0f)*DEG_TO_RAD;
    cosTable[i]=cosf(r);
    sinTable[i]=sinf(r);
  }
  Serial.println("IR Test - Japanese strategy adapted");
}

void loop(){
  uint32_t t=micros();
  sampleSensors();
  updateHistory();
  calcBall();

  Serial.println("================================");
  Serial.println("RAW:");
  for(int i=0;i<SENSOR_COUNT;i++){
    Serial.printf("S%02d=%3u ",i,rawCount[i]);
    if((i+1)%4==0)Serial.println();
  }
  Serial.println("FILTERED:");
  for(int i=0;i<SENSOR_COUNT;i++){
    Serial.printf("S%02d=%3u ",i,filtCount[i]);
    if((i+1)%4==0)Serial.println();
  }
  Serial.printf("MAX SENSOR: %d\n",maxSensor());
  Serial.printf("ANGLE: %.2f deg\n",ballAngle);
  Serial.printf("DIST(INT): %.1f\n",ballDistance);
  Serial.printf("Loop: %lu us (%.1f Hz)\n",micros()-t,1000000.0/(micros()-t));
  delay(100);
}