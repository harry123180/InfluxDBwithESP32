//MPU6050
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "FFT.h"
#include <math.h>
#include <Wire.h>
const int MPU = 0x68;
int16_t AcX, AcY, AcZ;
float gForceX, gForceY, gForceZ;
float orgdata[3][150];
///////////////////////////////////////
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"

#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif
#include <WiFiManager.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#define INFLUXDB_URL "http://125.229.142.15:9453"
// InfluxDB v2 server or cloud API token (Use: InfluxDB UI -> Data -> API Tokens -> Generate API Token)
#define INFLUXDB_TOKEN "VLJI0v-iKBTAmclC9KWPYEwN4swa6mITR-LJK1uw3c1_LDzwgroQ-eliFQcq1-YJc6G1FdL_ULa-z2U1aKe5mw=="
// InfluxDB v2 organization id (Use: InfluxDB UI -> User -> About -> Common Ids )
#define INFLUXDB_ORG "K1082"
// InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define INFLUXDB_BUCKET "test"
#define TZ_INFO "GMT-8"
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point sensor("mem4");
WiFiManager wm;
///////////////////////////////////
/* EdegComputing宣告*/
#include "EdgeComputing.h"
#define axis_num  3//總共有三軸
short int sensitivity = 54;
#define FFT_N 1024 // Must be a power of 2
Computer EC(FFT_N,axis_num,sensitivity);

//****FFT 必須的變數****//
int Sampling_Rate =1000;
short int TimerRef = 1000000/Sampling_Rate;
const float TOTAL_TIME = 1.024; // This is equal to FFT_N/sampling_freq
float fft_input0[FFT_N];
float fft_output0[FFT_N];
float fft_input1[FFT_N];
float fft_output1[FFT_N];
float fft_input2[FFT_N];
float fft_output2[FFT_N];
float freq_mag[axis_num][FFT_N/2];
int ORG_signal[axis_num][FFT_N];
float Time_Array[axis_num][FFT_N];
bool flag0 = false;
char print_buf[500];
bool EC_State=false;
//*******************octave band壓縮****************//
const int NUM_BANDS = 6;
const float CENTER_FREQS[NUM_BANDS] = {16,    31.5, 63,  125, 250,  500};
const int LOWER_BOUNDS[NUM_BANDS] =   {11,    23,   45,  89,  178,  356};
const int UPPER_BOUNDS[NUM_BANDS] =   {22,    44,   88,  177, 355,  511 };
int axis_choice =2;
void compressOctaveBand(float* data_fft, float* output) {
  for (int i = 0; i < NUM_BANDS; i++) {
    float sum = 0;
    for (int j = LOWER_BOUNDS[i]; j <= UPPER_BOUNDS[i]; j++) {
      sum += data_fft[j];
    }
    output[i] = sum;
  }
}
typedef struct data_package {
int num=2;
double Mean_[3] = {0};
double Std_[3] = {0};
double RMS_[3] = {0};
double Kurtosis_[3] = {0};
double tp_[3] = {0};
float max_magnitude[axis_num] = {0};
float fundamental_freq[axis_num] = {0};
} data_package;

data_package data_pkg;
/*
esp_now_peer_info_t peerInfo;//line 185
// 數據發送時回調
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // 將發件人mac地址複製到一個字符串
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
*/
//*******Task任務內容*********//
void taskOne( void * parameter ){
  while(1){
        unsigned long start_time = millis() ;
        sprintf(print_buf,"%d",flag0);
        sensor.clearFields();
        if(flag0){
          //EC.Convert_2d(ORG_signal,Time_Array);
          EC.Mean_2D(Time_Array,data_pkg.Mean_);
          EC.Std_2D(Time_Array,data_pkg.Mean_,data_pkg.Std_);          
          EC.RMS_2D(Time_Array,data_pkg.RMS_);          
          EC.Kurtosis_2D(Time_Array,data_pkg.Mean_,data_pkg.Std_,data_pkg.Kurtosis_);
          sensor.addField("Mean", data_pkg.Mean_[axis_choice]);
          sensor.addField("Std", data_pkg.Std_[axis_choice]); 
          sensor.addField("Kurtosis", data_pkg.Kurtosis_[axis_choice]); 
           /*
           Serial.print(data_pkg.Mean_[0]);
           Serial.print(" ");
           Serial.print(data_pkg.Mean_[1]);
           Serial.print(" ");
           Serial.println(data_pkg.Mean_[2]);
           */
          fft_config_t *real_fft_plan_0 = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input0, fft_output0);
          fft_config_t *real_fft_plan_1 = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input1, fft_output1);
          fft_config_t *real_fft_plan_2 = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input2, fft_output2);
          for (int k = 0 ; k < FFT_N ; k++){
            real_fft_plan_0->input[k] = (float)Time_Array[0][k];//將fft_signal填入輸入槽位
            real_fft_plan_1->input[k] = (float)Time_Array[1][k];//將fft_signal填入輸入槽位
            real_fft_plan_2->input[k] = (float)Time_Array[2][k];//將fft_signal填入輸入槽位  
          }
          fft_execute(real_fft_plan_0);
          fft_execute(real_fft_plan_1);
          fft_execute(real_fft_plan_2);
          for (int k = 1 ; k < real_fft_plan_1->size / 2 ; k++){
            //The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output
            freq_mag[0][k] = sqrt(pow(real_fft_plan_0->output[2*k],2) + pow(real_fft_plan_0->output[2*k+1],2))/1;
            freq_mag[1][k] = sqrt(pow(real_fft_plan_1->output[2*k],2) + pow(real_fft_plan_1->output[2*k+1],2))/1;
            freq_mag[2][k] = sqrt(pow(real_fft_plan_2->output[2*k],2) + pow(real_fft_plan_2->output[2*k+1],2))/1;
            float freq = k*1.0/TOTAL_TIME;  
            //Serial.println(  freq_mag[0][k]);  
            //sensor.addField(String(freq)+"Hz",freq_mag[0][k]);  
            if(freq_mag[0][k] >  data_pkg.max_magnitude[0]){
                data_pkg.max_magnitude[0] = freq_mag[0][k];
                data_pkg.fundamental_freq[0] = freq;
            }
            if(freq_mag[1][k] >  data_pkg.max_magnitude[1]){
                data_pkg.max_magnitude[1] = freq_mag[1][k];
                data_pkg.fundamental_freq[1] = freq;
            }   
            if(freq_mag[2][k] >  data_pkg.max_magnitude[2]){
                data_pkg.max_magnitude[2] = freq_mag[2][k];
                data_pkg.fundamental_freq[2] =freq;
            }        
          }
          
          float compressed[NUM_BANDS] = {};
          //for(int qq=0;qq<512;qq++){
             //Serial.print(qq*1.0/TOTAL_TIME);
             //Serial.print(" ");
             //Serial.println(freq_mag[axis_choice][qq]);
          //}
          compressOctaveBand(freq_mag[axis_choice] , compressed);  
          for(int sender_indx =0;sender_indx<NUM_BANDS;sender_indx++){
             sensor.addField(String(CENTER_FREQS[sender_indx])+"Hz",compressed[sender_indx]);                  
             //Serial.print(compressed[sender_indx]);
             //Serial.print(" ");
             //Serial.println(String(CENTER_FREQS[sender_indx])+"Hz");
          }
          fft_destroy(real_fft_plan_0);//釋放fft記憶體
          fft_destroy(real_fft_plan_1);
          fft_destroy(real_fft_plan_2);   
          EC.Total_Power_2D(freq_mag,data_pkg.tp_); 
          EC_State = true; 
          //ESP-Now
          //esp_err_t result = esp_now_send(0, (uint8_t *) &data_pkg, sizeof(data_pkg));
          Serial.println(millis() - start_time );
          Serial.print("Writing: ");
          Serial.println(sensor.toLineProtocol());
        
          Serial.println(wm.getWLStatusString());
          if (!client.writePoint(sensor)) {
            Serial.print("InfluxDB write failed: ");
            Serial.println(client.getLastErrorMessage());
          }
          vTaskDelay(3000);
          flag0 = false;//將fft_sginal填充完畢 flag復位
        }
  }
  Serial.println("Ending task 1");
  vTaskDelete( NULL );
}


void dataReceiver(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);  // request a total of 14 registers
  AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  processData();
}
void processData(){
  gForceX = AcX / 16384.0;
  gForceY = AcY / 16384.0; 
  gForceZ = AcZ / 16384.0;
}
void debugFunction(int16_t AcX, int16_t AcY, int16_t AcZ,uint8_t i){
  Time_Array[0][i]=gForceX;
  Time_Array[1][i]=gForceY;
  Time_Array[2][i]=gForceZ;
}
void setup() {
  Serial.begin(115200);
  Wire.begin(14,27);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  wm.resetSettings();
  bool res;
  res = wm.autoConnect("AutoConnectAP","password");
  Serial.print("Connecting to wifi");
  if(!res) {
          Serial.println("Failed to connect");
          // ESP.restart();
      } 
  else {
          //if you get here you have connected to the WiFi    
          Serial.println("connected...yeey :)");
      }
  // Add tags
  sensor.addTag("device", DEVICE);
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  delay(1000);
  //
  //Task宣告及初期設定
  xTaskCreatePinnedToCore(
  taskOne, //本任務實際對應的Function
  "TaskOne", //任務名稱（自行設定）
  10000, //所需堆疊空間（常用10000）
  NULL, //輸入值
  0, //優先序：0為最低，數字越高代表越優先
  NULL, //對應的任務handle變數
  tskNO_AFFINITY); //指定執行核心編號（0、1或tskNO_AFFINITY：系統指定）
}
void loop() {
  for(int i=0;i<FFT_N;i++){
  dataReceiver();
  debugFunction(AcX,AcY,AcZ,i);
  }
  flag0=true;
}
