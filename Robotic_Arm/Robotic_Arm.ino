#include "IRremote.h"
#include "SR04.h"
#define TRIG_PIN 12
#define ECHO_PIN 11

int receiver = 8; // Signal Pin of IR receiver to Arduino Digital Pin 11
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;



IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'


//values for sound sensor
int  sensorAnalogPin = A0;    // Select the Arduino input pin to accept the Sound Sensor's analog output 
int  sensorDigitalPin = 3;    // Select the Arduino input pin to accept the Sound Sensor's digital output
int  analogValue = 0;         // Define variable to store the analog value coming from the Sound Sensor
int  digitalValue;            // Define variable to store the digital value coming from the Sound Sensor
int  Led13 = 13;              // Define LED port; this is the LED built in to the Arduino (labled L)
                              // When D0 from the Sound Sensor (connnected to pin 7 on the
                              // Arduino) sends High (voltage present), L will light. In practice, you
                              // should see LED13 on the Arduino blink when LED2 on the Sensor is 100% lit.
                              
// describing Remote IR codes 
void translateIR() // takes action based on IR code received
{

  switch(results.value)

  {
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); break;
  case 0xFF629D: Serial.println("VOL+"); break;
  case 0xFF22DD: Serial.println("FAST BACK");    break;
  case 0xFF02FD: Serial.println("PAUSE");    break;
  case 0xFFC23D: Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: Serial.println("DOWN");    break;
  case 0xFFA857: Serial.println("VOL-");    break;
  case 0xFF906F: Serial.println("UP");    break;
  case 0xFF9867: Serial.println("EQ");    break;
  case 0xFFB04F: Serial.println("ST/REPT");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: 
    Serial.println(" other button   ");

  }// End Case

  delay(500); // Do not get immediate repeat


} //END translateIR
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("IR Receiver Button Decode");

  pinMode(sensorDigitalPin, INPUT);   //Define pin 7 for digital input for sound
  pinMode(Led13,OUTPUT);              //Define LED13 for digital outpupt for sound

  
  irrecv.enableIRIn(); // Start the receiver
  delay(1000);

}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  //Distance
  a=sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  delay(100);

  //Sound sensor
  analogValue = analogRead(sensorAnalogPin); // Read the value of the analog interface A0 assigned to digitalValue 
  digitalValue=digitalRead(sensorDigitalPin); // Read the value of the digital interface 7 assigned to digitalValue 
  Serial.println(analogValue); // Send the analog value to the serial transmit interface
  
  if(digitalValue==HIGH)      // When the Sound Sensor sends signla, via voltage present, light LED13 (L)
  {
    digitalWrite(Led13,HIGH);
  }
  else
  {
    digitalWrite(Led13,LOW);
  }
  
  //IR Remote
  if (irrecv.decode(&results)) // have we received an IR signal?

  {
    translateIR(); 
    irrecv.resume(); // receive the next value
  }

  delay(500);
}/* --(end main loop )-- */
