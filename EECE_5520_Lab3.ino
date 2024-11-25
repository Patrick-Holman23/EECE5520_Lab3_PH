// EECEE 5250 Lab 3
// Patrick Holman
// 11/10/2024

#include <Wire.h>
#include <DS3231.h>
#include <LiquidCrystal.h>
#include "IRremote.h"
#include "arduinoFFT.h"

DS3231 clock;
RTCDateTime dt;

const int Enable12 = 5;  // PWM pin to L293D's EN12 (pin 1) 
const int Driver1A = 4;  // To L293D's 1A (pin 2)
const int Driver2A = 3;  // To L293D's 2A (pin 7)
bool D1A = 0; // Initalize direction to clockwise
bool D2A = 1; // Initalize direction to clockwise
int speed_settings[4] = {0, 128, 192, 255};
int speed_index = 1;
int speed = speed_settings[speed_index]; // Initialize fan to full speed
int old_speed = speed_settings[speed_index]; // Use to store old speed for Play/Pause with remote
const int button = 53;           
boolean lastButton = HIGH; // Initliaze as HIGH (unpressed)
boolean currentButton = HIGH; // Initliaze as HIGH (unpressed)
char DirectionDisplay[3] = "C"; // Use to display rotation direction
char SpeedDisplay[5]= ""; // Use to store speed for display

LiquidCrystal lcd(7, 8, 9, 10, 11, 12); 
bool display = 1; // Initialize display flag to 1

const int sensorPin = A0;  // Sound sensor input
const int sampleRate = 1000; // Sample rate in Hz
const int samples = 128;    // Number of samples for FFT (must be a power of 2, 128, 256, 512, etc.)
ArduinoFFT< double > FFT = ArduinoFFT< double >();  // Create a FFT object
double vReal[samples];      // Real part 
double vImag[samples];      // Imaginary part

const int receiver = 2;
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
bool setTimeFlag = 0;
int setTimeTemp[6] = {0, 0, 0, 0, 0, 0};
int setTime[3] = {0, 0, 0};
int setTimeIndex = 0;

void setup(){
  //---set pin direction
  pinMode(Enable12,OUTPUT);
  pinMode(Driver1A,OUTPUT);
  pinMode(Driver2A,OUTPUT);
  pinMode(button, INPUT_PULLUP);  //Set as input and use internal pullup
  Serial.begin(9600);

  Serial.println("Initialize RTC module");
  clock.begin(); 
  // clock.setDateTime(__DATE__, __TIME__);  Not retrieving date&time so that the time during compile isn't programmed on reset

  lcd.begin(16, 2);
    
  cli(); //Clear interrupt flag bit (disable interrupts)
  // 1hz interrupt for setting LCD display
  TCCR1A = 0; // clear TCCR1A 
  TCCR1B = 0; // clear TCCR1B
  TCNT1  = 0; // Clear the 16 bit timer/counter (TCNT1H and TCNT1L)
  OCR1A = 15624; // set 16 bit output compare register to 1 second -> (16MHz / 1024prescalar) * 1second - 1 = 15624 bits
  TCCR1B |= (1 << WGM12); // Set bit 2 of WGM13:0 for CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS12 and CS10 for 1024 prescaler 
  TIMSK1 |= (1 << OCIE1A); // Enable output compare A match interrupt
  sei();//Set interrupt flag bit (re-enable interrupts)

  irrecv.enableIRIn();
}

// 1Hz Interrupt Routine
ISR(TIMER1_COMPA_vect){ 
  display = 1; // Set display flag
}

// Motor Control Function
void motorCTRL(byte speed, bool D1A, bool D2A){
	analogWrite(Enable12,speed);  // PWM
	digitalWrite(Driver1A,D1A);   // Boolean
	digitalWrite(Driver2A,D2A);   // Boolean 
}

//Debouncing function
boolean debounce(boolean last){
  boolean current = digitalRead(button); // Read current state of button
  if(last != current){ // Check if 
    delay(5); // Delay for 5ms
    current = digitalRead(button); // Read current state of button again
  }
  return current; // Return current button state
}

void translateIR() {    

  switch(results.value){

    case 0xFFE21D: // FUNC/STOP
      setTimeIndex = 0; // Reset TimeIndex counter
      setTimeFlag = 1; // Set flag to allow for setting time with remote control input
      Serial.println("Set time flag is set");
    break;

    case 0xFF02FD: // PLAY/PAUSE
      if (speed == 0){
        speed = old_speed;
      }
      else {
        old_speed = speed; 
        speed = speed_settings[0]; 
      }
    break;

    case 0xFFC23D: // FAST FORWARD
      if (speed_index < 3){
        speed_index++;
        speed = speed_settings[speed_index];
      }
    break;

    case 0xFF22DD: // FAST BACK
      if (speed_index > 0){
        speed_index--;
        speed = speed_settings[speed_index];
      }   
    break;

    case 0xFF6897: 
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 0;
          setTimeIndex++;
        }
      }
    break;

    case 0xFF30CF:
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 1;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF18E7:  
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 2;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF7A85: 
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 3;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF10EF: 
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 4;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF38C7: 
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 5;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF5AA5:
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 6;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF42BD:
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 7;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF4AB5: 
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 8;
          setTimeIndex++;
        }
      }
    break;
    case 0xFF52AD:
      if (setTimeFlag){
        if (setTimeIndex < 6){
          setTimeTemp[setTimeIndex] = 9;
          setTimeIndex++;
        }
      }
    break;

  default: 
    Serial.print(" other button   ");
    Serial.println(results.value);
  }
}

void loop() {

  motorCTRL(speed, D1A, D2A);
  
  // Check for Button Press
  currentButton = debounce(lastButton);
  if(lastButton == LOW && currentButton == HIGH){ // Button Press Detected
    motorCTRL(speed, D1A ^= 1, D2A ^= 1);
    if (D1A == 0){ // If clockwise
      DirectionDisplay[1] = '\0'; 
    }
    if (D1A == 1){ // If counter clockwise
      DirectionDisplay[1] = 'C';
    }
  }
  lastButton = currentButton;
  
  // Update display with 1 second interrupt
  if (display == 1) {

    dt = clock.getDateTime();
    lcd.setCursor(0,0);

    if (speed == 0){ // Find what speed to display on lcd
      SpeedDisplay[0] = '0';
      SpeedDisplay[1] = ' ';
      SpeedDisplay[2] = ' ';
      SpeedDisplay[3] = ' ';
    }
    if (speed == 128){
      SpeedDisplay[0] = '1';
      SpeedDisplay[1] = '/';
      SpeedDisplay[2] = '2';
      SpeedDisplay[3] = ' ';
    }
    if (speed == 192){
      SpeedDisplay[0] = '3';
      SpeedDisplay[1] = '/';
      SpeedDisplay[2] = '4';
      SpeedDisplay[3] = ' ';
    }
    if (speed == 255){
      SpeedDisplay[0] = 'F';
      SpeedDisplay[1] = 'U';
      SpeedDisplay[2] = 'L';
      SpeedDisplay[3] = 'L';
    }

    lcd.print(DirectionDisplay); lcd.print(" "); lcd.print(SpeedDisplay); lcd.print(" "); // Print blank character to prevent extra L in some cases
    // Set cursor to bottom row and print HH:MM:SS
    lcd.setCursor(0, 1);
    lcd.print(dt.hour < 10 ? "0" : ""); lcd.print(dt.hour);   lcd.print(":");
    lcd.print(dt.minute < 10 ? "0" : ""); lcd.print(dt.minute); lcd.print(":");
    lcd.print(dt.second < 10 ? "0" : ""); lcd.print(dt.second); lcd.print(" ");
    display = 0; // Clear display flag
  }

  // IR signal function
  if (irrecv.decode(&results)) // Check for IR signal
  {
    translateIR();
    delay(100);
    irrecv.resume(); // receive the next value
  }

  if (setTimeIndex == 6){
    setTime[0] = setTimeTemp[0]*10 + setTimeTemp[1];
    setTime[1] = setTimeTemp[2]*10 + setTimeTemp[3];
    setTime[2] = setTimeTemp[4]*10 + setTimeTemp[5];
    clock.setDateTime(2024, 11, 25,setTime[0],setTime[1],setTime[2]);
    setTimeIndex = 0;
    setTimeFlag = 0;
  }

  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(sensorPin);  // Read analog input
    vImag[i] = 0;                      // Imaginary part is always 0 for FFT
    delayMicroseconds(1000000 / sampleRate);  // Wait to maintain the sample rate
  }

  // Perform the FFT analysis
  FFT.windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply Hamming window
  FFT.compute(vReal, vImag, samples, FFT_FORWARD);  // Perform FFT

  // Find the peak frequency
  double peakFrequency = FFT.majorPeak(vReal, samples, sampleRate);
  if (peakFrequency >= 431 && peakFrequency <= 449) {  // A4 
    Serial.println("Detected note: A4");
    if (speed_index < 3){
        speed_index++;
        speed = speed_settings[speed_index];
      }
  } else if (peakFrequency >= 257 && peakFrequency <= 267) {  // C4
    Serial.println("Detected note: C4");
    if (speed_index > 0){
        speed_index--;
        speed = speed_settings[speed_index];
      }
  }

}

  
