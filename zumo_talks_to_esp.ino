#include <Wire.h>
#include <Zumo32U4.h>

#define GYRO_PERIOD 20

#define BOT_RADIUS_CM 4.5
#define CMD_PERIOD 100
#define CMD_FILT_FACTOR 0.5
#define SPEED_FILT_FACTOR 0.5
#define PING_LIMIT_PERIOD 300
#define ZSERIAL Serial1

long timeCMD;
long gyro_time;
long timePING;

int gz_raw;
int8_t speed_cm = 0;
int8_t omega_deg = 0;
int left_filt_output = 0;
int right_filt_output = 0;

int16_t left_speed_filt_mm = 0;
int16_t right_speed_filt_mm = 0;
int16_t prev_left_speed = 0;
int16_t prev_right_speed = 0;

int16_t Kp = 1;

unsigned long serialdata;
int inbyte = 0;

uint8_t nBytes = 0;
uint8_t k = 0;
uint8_t data[16] = {0};

Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

void setup()
{
  motors.setSpeeds(0, 0);
  ZSERIAL.begin(115200);
  Serial.begin(9600);
  
  Wire.begin();

  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
  }

  imu.enableDefault();

  timeCMD = millis();
  timePING = millis();
  gyro_time = millis();
}

void loop()
{
  if(timeSince(gyro_time) > GYRO_PERIOD)
  {
    gyro_time = millis();
    imu.read();
    gz_raw = imu.g.z;

    //left_speed_filt_mm = 
    
    //float dt = (float)timeSince(gyro_time)/1000.0;
    //gyro_z = (gyro.z()+GYRO_BIAS_DEG) * GYRO_SCALE_FACTOR;
    //delta_yaw_deg += gyro_z*dt;
    //yaw_deg += gyro_z*dt;
  }

  if(timeSince(timePING) > PING_LIMIT_PERIOD)
  {
    motors.setSpeeds(0, 0);
  }
  else if(millis() - timeCMD > CMD_PERIOD)
  {
    timeCMD = millis();
    
    float left_cm = speed_cm - BOT_RADIUS_CM*float(omega_deg*8)*3.14/180.0;
    float right_cm = speed_cm + BOT_RADIUS_CM*float(omega_deg*8)*3.14/180.0;
    left_cm = min(left_cm, 20);
    left_cm = max(left_cm, -20);
    right_cm = min(right_cm, 20);
    right_cm = max(right_cm, -20);
    int left_output = left_cm * 7; //TODO: estimate cm/sec per cmd unit
    int right_output = right_cm * 7;
    left_filt_output = left_filt_output * CMD_FILT_FACTOR + left_output * (1 - CMD_FILT_FACTOR);
    right_filt_output = right_filt_output * CMD_FILT_FACTOR + right_output * (1 - CMD_FILT_FACTOR);
    motors.setSpeeds(left_filt_output, right_filt_output);
  }

  // A3/4/ encoder request
  // A1/1/speed_byte,curv_byte

  // Consider making nBytes, k, data global
  //   and set k = 0 when you might have a start byte
  //   in the middle of expected group of bytes
  // Instead of setting k = 0, simply set startIndex = k-1
  //   Then change the if statements to check data[(startIndex+j)%7

  if(ZSERIAL.available())
  {
    while(ZSERIAL.available())
    {
      data[k] = ZSERIAL.read();
      ++k;
      if(k > 7)
      {
        k = 0;
      }
      //Serial.print(data[k]); Serial.print(",");
    }
    //Serial.println();
    ZSERIAL.flush();
  }

  if(k == 5 && data[0] == 'A' && data[1] == '3' && data[2] == '/' && data[3] == '4' && data[4] == '/')
  {
    // A3/4/
    //encLeft/Right read and reset
    ZSERIAL.write(int8_t(encoders.getCountsAndResetLeft()));
    ZSERIAL.write(int8_t(encoders.getCountsAndResetRight()));
    ZSERIAL.write(highByte(int16_t(gz_raw)));
    ZSERIAL.write(lowByte(int16_t(gz_raw)));
    k = 0;
  }
  else if(k == 7 && data[0] == 'A' && data[1] == '1' && data[2] == '/' && data[3] == '1' && data[4] == '/')
  {
    timePING = millis(); // A1/1/
    // Read the next two numbers <raw speed 0 to 240>/<raw omega 0 to 240>/
    speed_cm = (int8_t)data[5];
    omega_deg = (int8_t)data[6];         
    Serial.print("speed cm "); Serial.print(speed_cm);
    Serial.print(", omega deg "); Serial.println(omega_deg);
    k = 0;
  }

}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}
