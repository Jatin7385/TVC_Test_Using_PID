/* After more research, I've found that it is Torque we need to compensate for, not just the angle, in a TVC system. The Torque faced by the Rocket, using which it rotates to Theta angle, needs to be compensated by the Nozzle spewing out exhaust gas, which turns into
 compensatory Torque. This is a Torque conservation problem. Now, Torque = rFsin(phi). Let phi be the angle the Nozzle moves(For compensating the natural torque), F the thrust by the motor and r be the distance from the rotational axis to the nozzle(d between 
 Centre of gravity to nozzle). Torque is also Ia(I * alpha), where I is the Inertia of Rotation( a scalar value which tells us how difficult it is to change the rotational velocity of the object around a given rotational axis.
Rotational inertia plays a similar role in rotational mechanics to mass in linear mechanics.) and a(alpha) is the angular acceleration(d^2(Theta)/dt^2) = d^2(Change in angle of rocket)/dt^2. We know Ia (This is the torque we have faced naturally which caused the rocket
to tip). This is the error. So we use Torque as the input errors in the PID Controller. We get an output control signal for Torque, which then can be used to find the unknown Phi in the formula Torque = rFsin(phi). Phi is the angle we need to rotate the servo.

For now, we will assume, that the only job of the servo, is to compensate for the current angle.

So current_angle = Current rotation of the rocket(Generally should be 0 degree)
    servo_val = 90(The servo's initial position should be at 90 degrees)
    PID is using current_angle to give the corrective servo angle signal. In the actual system, we won't be using this. We will be using Torque.
*/

//#include <Adafruit_MPU6050.h>

#include <Servo.h>

//Adafruit_MPU6050 mpu;

Servo myservo; 

/* Get new sensor events with the readings */
//sensors_event_t a, g, t;

int i = 0;

float KP = 0.8;
float KI = 0.01;
float KD = 0.2;

int proportional_error = 0;
int integral_error = 0;
int derivative_error = 0;
float old_time = 0;
float current_time = 0;
float prev_proportional_error = 0.0;
int current_angle = 0; // Current angle of the rocket from the ground 
int servo_val = 90; // Servo val is the angle of the nozzle from the ground
int control_signal = 0; // 
float dt = 0;

int setpoint = 0;

int counter = 0;
int flag = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("Finding MPU6050");

   myservo.attach(9);

  myservo.write(0);
  delay(20);
   myservo.write(90);
   delay(20);
  
//  // Try to initialize!
//  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
//    while (1) {
//      delay(10);
//    }
//  }
//  Serial.println("MPU6050 Found!");
//
//  // set accelerometer range to +-8G
//  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//
//  // set gyro range to +- 500 deg/s
//  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//
//  // set filter bandwidth to 21 Hz
//  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:

  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");

//  Serial.print("Rotation X: ");
//  Serial.print(g.gyro.x);
//  Serial.print(", Y: ");
//  Serial.print(g.gyro.y);
//  Serial.print(", Z: ");
//  Serial.print(g.gyro.z);
//  Serial.println(" rad/s");

//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");

    
    proportional_error = setpoint - current_angle;
    integral_error += proportional_error;
    derivative_error = proportional_error - prev_proportional_error;
    current_time = millis();
    dt = (current_time - old_time)/1000;

    Serial.print("P : ");
    Serial.print(proportional_error);
    Serial.print(" , I : ");
    Serial.print(integral_error);
    Serial.print(" , D : ");
    Serial.print(derivative_error);
    Serial.print(" , DT : ");
    Serial.print(dt);

    control_signal = (KP * proportional_error) + (KI * integral_error * dt) + ((KD/dt) * derivative_error);

    Serial.print(", Current angle : ");
    Serial.print(current_angle);
    Serial.print(" , Servo Val : ");
    Serial.print(servo_val);
    Serial.print(" , Control Signal is : ");
    Serial.println(control_signal);

    current_angle += control_signal;
    servo_val = 90 + control_signal;
    if(servo_val < 0)
    {
      servo_val = 0;
    }
    else if(servo_val > 180)
    {
      servo_val = 180;
    }
    myservo.write(servo_val); 
    delay(100); 

    prev_proportional_error = proportional_error;
    i += 1;

    old_time = millis();
    
    if(i%10 == 0 && flag == 0)
    {
      current_angle += 70;
      counter = 0;
      flag = 1;
    }

    if(flag == 1)
    {
        current_angle -= 10;
    }

    if(current_angle <= 0)
    {
      flag = 0;
    }
  Serial.println(i);
  delay(2000);
}
//#include <Wire.h>
// 
// 
//void setup()
//{
//  Wire.begin();
// 
//  Serial.begin(9600);
//  while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
//}
// 
// 
//void loop()
//{
//  byte error, address;
//  int nDevices;
// 
//  Serial.println("Scanning...");
// 
//  nDevices = 0;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    Serial.println("Transmission begun");
//    error = Wire.endTransmission();
//    Serial.println("Transmission ended");
// 
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
// 
//      nDevices++;
//    }
//    else if (error==4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }    
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
// 
//  delay(5000);           // wait 5 seconds for next scan
//}
