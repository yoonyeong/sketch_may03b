#define FRONT_LED_PIN 10
#define REAR_LED_PIN 9
#define LEFT_MD_A 22
#define LEFT_MD_B 23
#define RIGHT_MD_A 24
#define RIGHT_MD_B 25
#define LEFT_MOTOR_EN 4
#define RIGHT_MOTOR_EN 5
#define NUM_TX_BYTES 5
#define NUM_RX_BYTES 17
#define S_DIN 42
#define S_SCLK 43
#define S_SYNCN 44
#define IN_SEN_EN 26


int SensorA[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int SensorD[8] = {30,31,32,33,34,35,36,37};
int x_val = 0;
int y_val = 0;
int car_val[3] = {0, 0, 0};
int storage = 0;



unsigned char TX_buf[NUM_TX_BYTES] = {0x76, 0x00, 0xF0, 0x00, 0xF0};
unsigned char TX_stop_buf[NUM_TX_BYTES] = {0x76, 0x00, 0x0F, 0x00, 0x0F};
unsigned char RX_buf[NUM_RX_BYTES];


boolean ultrasonic_result = false;
boolean line_tracing = false ;
char text[] = "hello haha";

void setup() {

      int i = 0;
      Serial.begin(115200);
      randomSeed(analogRead(0));
      
      
//
   while (text[i] != '\0')
     Serial.write(text[i++]);
     Serial.write("Received cmds: ");


    int z;
        int dac_val_min[8] =
        {82, 182, 152, 86, 89, 169, 155, 68};
        int dac_val_max[8] =
        {422, 711, 628, 442, 408, 534, 643, 383};




      Serial1.begin(115200);
      pinMode(FRONT_LED_PIN,OUTPUT);
      pinMode(REAR_LED_PIN,OUTPUT);
      pinMode(LEFT_MD_A, OUTPUT);
            pinMode(LEFT_MD_B, OUTPUT);
            pinMode(RIGHT_MD_A, OUTPUT);
            pinMode(RIGHT_MD_B, OUTPUT);
            pinMode(LEFT_MOTOR_EN, OUTPUT);
            pinMode(RIGHT_MOTOR_EN, OUTPUT);
            pinMode(IN_SEN_EN,OUTPUT);
            pinMode(S_DIN,OUTPUT);
            pinMode(S_SCLK,OUTPUT);
            pinMode(S_SYNCN,OUTPUT);

            digitalWrite(S_SCLK,LOW);
            digitalWrite(S_SYNCN,HIGH);
            digitalWrite(IN_SEN_EN,HIGH);

            digitalWrite(LEFT_MD_A, LOW);
            digitalWrite(LEFT_MD_B, LOW);
            digitalWrite(RIGHT_MD_A, LOW);
            digitalWrite(RIGHT_MD_B, LOW);
            digitalWrite(LEFT_MOTOR_EN, LOW);
            digitalWrite(RIGHT_MOTOR_EN, LOW);


      for (z=0; z<8; z++)
            pinMode(SensorD[z], INPUT);

            DAC_setting(0x9000); //for Write-Through Mode
      for (z=0; z<8; z++)
      {
        int mean_val =(dac_val_min[z]+dac_val_max[z])/2; //10-bit
        DAC_CH_Write(z, mean_val >> 2);
      //should be 8-bit
      }

}


void DAC_CH_Write(unsigned int ch, unsigned int da)
{
        unsigned int data = ((ch << 12) & 0x7000) |
        ((da << 4) & 0x0FF0);
        DAC_setting(data);
        }
        void DAC_setting(unsigned int data)
        {
        int z;
        digitalWrite(S_SCLK,HIGH);
        delayMicroseconds(1);
        digitalWrite(S_SCLK,LOW);
        delayMicroseconds(1);
        digitalWrite(S_SYNCN,LOW);
        delayMicroseconds(1);
        for(z=15;z>=0;z--)
        {
        digitalWrite(S_DIN,(data>>z)&0x1);
        digitalWrite(S_SCLK,HIGH);
        delayMicroseconds(1);
        digitalWrite(S_SCLK,LOW);
        delayMicroseconds(1);
        }
        digitalWrite(S_SYNCN,HIGH);
}









void move_stop()
{
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}


void front_led_control(boolean t)
{
  if (t==true)
    digitalWrite(FRONT_LED_PIN,HIGH);
  else
    digitalWrite(FRONT_LED_PIN,LOW);
}

void rear_led_control(boolean t)
{
  if (t==true)
    digitalWrite(REAR_LED_PIN,HIGH);
  else
    digitalWrite(REAR_LED_PIN,LOW);
}

void ultrasonic_sensor_read()
{
ultrasonic_result = false;
Serial1.write(TX_buf, NUM_TX_BYTES);
}

void infrared_sensor_read()
{
int z;
Serial.print("SamSam Let's Go!");
for(z=7;z>=0;z--)
{
unsigned int val = analogRead(SensorA[z]);
Serial.print(val);
Serial.print(" ");
}
Serial.println("");

for(z=7;z>=0;z--)
{
unsigned int val = digitalRead(SensorD[z]);
Serial.print(val);
Serial.print(" ");
}


}

void line_tracing_enable()
{
line_tracing = true;
Serial.write("Line tracing is enabled..");
}


void line_tracing_disable()
{
line_tracing = false;
move_stop();
Serial.write("Line tracing is disabled..");
}



void car_init(){

 x_val = random(10);
 y_val = random(10)+100;
storage = random(60)+10; 

car_val[0] = storage;
car_val[1] = x_val;
car_val[2] = y_val;


  
}


void car_info()
{
 
   
                     Serial.print(car_val[0]);
                     Serial.print(',');
                     Serial.print(car_val[1]);
                     Serial.print(',');
                     Serial.print(car_val[2]);             
                          
                 
                  
  }

  
void move_forward_speed(int left, int right)
{

    digitalWrite(LEFT_MD_A, HIGH);
    digitalWrite(LEFT_MD_B, LOW);
    //Rotate clockwise for right motor
    digitalWrite(RIGHT_MD_A, LOW);
    digitalWrite(RIGHT_MD_B, HIGH);
    //Now turn left and right motors ON!
    analogWrite(LEFT_MOTOR_EN, left);
    analogWrite(RIGHT_MOTOR_EN, right);

}


void turn_left_speed(int left, int right)
{
      digitalWrite(LEFT_MD_A, LOW);
    digitalWrite(LEFT_MD_B, HIGH);

    digitalWrite(RIGHT_MD_A, LOW);
    digitalWrite(RIGHT_MD_B, HIGH);


    analogWrite(LEFT_MOTOR_EN,left);
    analogWrite(RIGHT_MOTOR_EN,right);


}


void turn_right_speed(int left, int right)
{
    digitalWrite(LEFT_MD_A, HIGH);
    digitalWrite(LEFT_MD_B, LOW);

    digitalWrite(RIGHT_MD_A, HIGH);
    digitalWrite(RIGHT_MD_B, LOW);
    analogWrite(LEFT_MOTOR_EN,left);
    analogWrite(RIGHT_MOTOR_EN,right);


}


void turn_pivot_left_speed(int left, int right)
{
  digitalWrite(LEFT_MD_A, LOW);
  digitalWrite(LEFT_MD_B, HIGH);

  digitalWrite(RIGHT_MD_A, LOW);
  digitalWrite(RIGHT_MD_B, HIGH);


  analogWrite(LEFT_MOTOR_EN,left);
  analogWrite(RIGHT_MOTOR_EN,right);


}


void turn_pivot_right_speed(int left, int right)
{
    digitalWrite(LEFT_MD_A, HIGH);
    digitalWrite(LEFT_MD_B, LOW);

    digitalWrite(RIGHT_MD_A, HIGH);
    digitalWrite(RIGHT_MD_B, LOW);
    analogWrite(LEFT_MOTOR_EN,left);
    analogWrite(RIGHT_MOTOR_EN,right);



}




void loop()
{
 
}




void serialEvent(){

    int command = Serial.read();

    switch (command)
  {
      case 1:
      car_info();
      break;

      case 2:
      car_init();
      break;

      default:
      move_stop();
      front_led_control(false);
      rear_led_control(false);
  }

}




void serialEvent1()
{
  unsigned char z, tmp = 0;
  Serial1.readBytes((char *)RX_buf, NUM_RX_BYTES);

  if ( (RX_buf[0] == 0x76) && (RX_buf[1] == 0x00) && (ultrasonic_result == false) )
  {
    for (z = 2; z < NUM_RX_BYTES-1; z++)
      tmp += RX_buf[z];
      tmp = tmp & 0xFF;

      if (RX_buf[NUM_RX_BYTES-1] == tmp)
      {
        Serial.println("FRONT");

        for (z=4; z < 11; z++)
          {
            Serial.print(" F");
            Serial.print(z-4);
            Serial.print(": ");
            Serial.print(RX_buf[z]);
          }

        Serial.println("\nBACK");

        for (z=11; z < NUM_RX_BYTES-1; z++)
          {
            Serial.print(" B");
            Serial.print(z-11);
            Serial.print(": ");
            Serial.print(RX_buf[z]);
          }
      }
      ultrasonic_result = true;
      Serial1.write(TX_stop_buf,NUM_TX_BYTES);
  }
}
      
      
      


      
