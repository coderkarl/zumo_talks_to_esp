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
  // A2/1/autoblade/

  if(ZSERIAL.available())
  {
    while(ZSERIAL.available())
    {
      uint8_t b = ZSERIAL.read();
      Serial.print(b); Serial.print(",");
    }
    Serial.println();
  }
  
  if(ZSERIAL.read() == 'B'){
    getSerial();
    switch(serialdata)
    {
      case 1: // A1/
      {
        getSerial(); // A1/1,2,3,4/
        switch (serialdata)
        {
          case 1: // A1/1/
          {
            timePING = millis();
            // Read the next two numbers <raw speed 0 to 240>/<raw omega 0 to 240>/
            if(ZSERIAL.available())
            {
              speed_cm = (int8_t)ZSERIAL.read(); //getSerial()-120;
            }
            if(ZSERIAL.available())
            {
              omega_deg = (int8_t)ZSERIAL.read(); //getSerial()-120;
              //uint8_t b3 = ZSERIAL.read();
              //uint8_t b4 = ZSERIAL.read();              
              Serial.print("speed cm "); Serial.print(speed_cm);
              Serial.print(", omega deg "); Serial.println(omega_deg);
              //Serial.print(b3); Serial.print(","); Serial.println(b4);
              //Serial.println((int8_t)255); //confirmed (int8_t)255 is -1
            }
            ZSERIAL.flush();
            break;
          }
        }
        break;
      } // end A1/
      case 3: // A3/
      {
        getSerial(); // A3/1,2,3,4/
        switch (serialdata)
        {
          case 4: // A3/4/
          {
            //encLeft/Right read and reset
            ZSERIAL.write(int8_t(encoders.getCountsAndResetLeft()));
            ZSERIAL.write(int8_t(encoders.getCountsAndResetRight()));
            ZSERIAL.write(highByte(int16_t(gz_raw)));
            ZSERIAL.write(lowByte(int16_t(gz_raw)));
            //Serial.println(int(delta_yaw_deg*1000));
            //delta_yaw_deg = 0.0;
            break;
          }
        }
        break;
      } // end A3/
    } // end switch serialdata
  }

}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = ZSERIAL.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}
