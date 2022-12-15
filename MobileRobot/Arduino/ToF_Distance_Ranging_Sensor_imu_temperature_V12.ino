#include <Wire.h>
#include <VL6180X.h>
#include <DHT.h>
#include <TimeLib.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define RANGE 1

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x30
#define address1 0x31


/* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = 6;
int enablePin1 = 7;
int umbral = 180;


/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;

// Definimos el pin digital donde se conecta el sensor
#define DHTPIN 3
// Dependiendo del tipo de sensor
#define DHTTYPE DHT11
// Inicializamos el sensor DHT11
DHT dht(DHTPIN, DHTTYPE);

Adafruit_MPU6050 mpu;
int16_t AcX1,AcY1,AcZ1,Tmp1,GyX1,GyY1,GyZ1;
int16_t D6,D7;
int16_t h,minu,seg;
int16_t Temperatura;


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);
 
  
  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW);

  // Comenzamos el sensor DHT - TEMPERATURA 
  dht.begin();
    
  delay(1000);
  
  // Left sensor
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
  sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor0.setTimeout(500);
  sensor0.stopContinuous();
  sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor0.startInterleavedContinuous(100);
  delay(100);
  
  // Right sensor
  digitalWrite(enablePin1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address1);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(1000);
}

void loop()
{
  //Serial.print("<telemetry>\n");
  // Serial.print("    <sensor>\n      <name>Temperature01</name>\n      <type>temperature</type>\n      <value>");
  // Serial.print(hic);
  // Serial.print("</value>\n      <unit>Celsius</unit>\n    </sensor>\n");

  //Serial.print("    <sensor>\n      <name>Distance01</name>\n      <type>distance</type>\n      <value>");
  //Serial.print(sensor0.readRangeContinuousMillimeters());
  //if (sensor0.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  //Serial.print("</value>\n      <unit>milimeters</unit>\n    </sensor>\n");

  //Serial.print("    <sensor>\n      <name>Distance02</name>\n      <type>distance</type>\n      <value>");
  //Serial.print(sensor1.readRangeContinuousMillimeters());
  //if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  //Serial.print("</value>\n      <unit>milimeters</unit>\n    </sensor>\n");

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  AcX1=a.acceleration.x; 
  AcY1=a.acceleration.y;
  AcZ1=a.acceleration.z; 
  GyX1=g.gyro.x;
  GyY1=g.gyro.y;
  GyZ1=g.gyro.z;

  D6 = sensor0.readRangeContinuousMillimeters();
  D7 = sensor1.readRangeContinuousMillimeters();

  h = hour();            // the hour now  (0-23)
  minu = minute();          // the minute now (0-59)
  seg = second();
  Temperatura = dht.readTemperature();

  // h min seg;ACC ACCX ACCY ACCZ [m/s^2];GGY GGYX GGYY GGYZ [rad/s];DIS D6 D7 [mm];Colision D6(Bool) D7(Bool);Temperatura Tambiental [ºC]
  /*Serial.print(h);Serial.print(" ");Serial.print(minu);Serial.print(" ");Serial.print(seg);Serial.print(";");
  Serial.print("ACC ");Serial.print(AcX1);Serial.print(" ");Serial.print(AcY1);Serial.print(" ");Serial.print(AcZ1);Serial.print(" ");Serial.print("m/s^2");Serial.print(";");
  Serial.print("GYY ");Serial.print(GyX1);Serial.print(" ");Serial.print(GyY1);Serial.print(" ");Serial.print(GyZ1);Serial.print(" ");Serial.print("rad/s");Serial.print(";");
  Serial.print("DIS ");Serial.print(D6);Serial.print(" ");Serial.print(D7);Serial.print(" ");Serial.print("mm");Serial.print(";");
  Serial.print("COLISION ");Serial.print(D6<umbral);Serial.print(" ");Serial.print(D7<umbral);Serial.print("\n");*/

  Serial.print(h);Serial.print(" ");Serial.print(minu);Serial.print(" ");Serial.print(seg);Serial.print(";");
  Serial.print(AcX1);Serial.print(" ");Serial.print(AcY1);Serial.print(" ");Serial.print(AcZ1);Serial.print(";");
  Serial.print(GyX1);Serial.print(" ");Serial.print(GyY1);Serial.print(" ");Serial.print(GyZ1);Serial.print(";");
  Serial.print(D6);Serial.print(" ");Serial.print(D7);Serial.print(";");
  Serial.print(D6<umbral);Serial.print(" ");Serial.print(D7<umbral);Serial.print(";");
  Serial.print(Temperatura);Serial.print("\n");
  //Serial.print(Temperatura);Serial.print(" ");Serial.print(Temperatura);Serial.print("\n");
  
  delay(10);
}
