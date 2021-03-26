/*
  RFID sensor and logger
  Requires Arduino V1.0 and above

  Written by Jonathan Reynolds, Game & Wildlife Conservation Trust, Fordingbridge, SP6 1EF, UK
  This code is being made publicly available for research and education purposes, but remains the Intellectual Property of the author.  
  The above authorship must be acknowledged in full in any publication resulting from use of the code or parts or adaptations thereof.
  Anyone wishing to use the code or parts or adaptations of it for commercial purposes is required to contact the author.  
    
  See ############ for full details of the device hardware.	

  The code assumes the use of the Ardulog RTC (Hobbytronics Ltd) which combines the ATMega 328 microprocessor with an RTC and microSD card reader/writer.
  Hobbytronics example software expects to find a config.txt file on the SD card, with entries to determine BAUD rate and output file name.
  I have fixed baud rate (9600) in the code and implemented a filenaming convention by date. Instead I use the config.txt file is now used to identify siteID and operatorID.
  A unique loggerID can be set for each unit when this firmware is uploaded (Line 76).  This is recorded with the data so that the device can be identified in case of hardware failure.
  In practice I have never found this safeguard to be useful (the hardware is surprisingly robust), so the loggerID could be cut out or ignored as preferred. 
  
  When the device is switched on, a start-up routine is executed, then the device falls into a low-current sleep mode.
  This is interrupted when an external wake-up switch is activated, bringing pin A3 LOW.  The switch can be either NC or NO.  
  On wake-up, this code uses pin D4 on the Ardulog RTC to start/stop the RFID reader by switching its power supply with a MOSFET.  
  The reader remains 'on' for a period (windowLength) predetermined in the firmware before being switched back off.
  There is also a pre-determined latent period (set by pauseLength, line 87) to prevent too many records of the same tag.   
  The processor goes back to sleep (POWER_DOWN) after a fixed period (set by windowLength, line 88) if the external switch is in 'inactive' position.
  or initiates a new window if the switch suggests that an animal is still present.

  The RFID reader communicates with the Ardulog RTC via I2C through pins D0 (RX) and D1 (TX).
  
  Communication between the Ardulog RTC and the SD card uses SPI, which takes place on digital pins 11, 12, and 13. 
  On the Ardulog RTC these are hard-wired to the SD socket. Additionally, pin D6 is hard-wired as the chip-select pin.
  The firmware selects the SD card by default - SPI could be made to talk to additional devices using another pin specified in the call SD.begin(). 
  Note that even if you don't use the hardware SS pin, it must be left as an output or the SD library won't work.

  The on-board RTC is hard-wired to pins SDA (A4) and SCL (A5) of the AtMega 328 chip to communicate by I2C (as well as to the 3.3V supply and to common GND).
  The RTC time can be initialised via the Arduino IDE and the serial monitor using a separate sketch.  
  The SD card must be absent to do this, and the present sketch must afterwards be loaded up again. 
  
  Green LED flashes on/off continuously at power-up or restart if SD card is missing, cannot be read or written to, or if no config.txt file is present on card.  
  This changes to brief 'heartbeat' flash every 2 seconds once problems are rectified and the device is ready to log data.
  Orange LED indicates activity at SD card socket.
   
  Output file records 4 types of event:
  1 = Switch activated, starts timing activation 'window', waiting to read RFID tag
  2 = First tag read in the activation window
  3 = Follow-on read within same activation window
  4 = Activation window timed out.  This moment is determined by variable windowLength, set by default to be 1 min. 
  
  Line 79 provides the opportunity to set a device number (loggerID) in the code, which will be recorded with the data.
  This is potentially useful in case any failure is suspected in an individual device when data are analysed.
  
*/
 
// --------------------------------------------------------------------------------------------------------------------------------
//Specify function libraries to include

#include <SPI.h>  // Library for SPI communication, not explicitly called by this code, but used by SD library.
#include <SD.h>   // Library for SD card operations.
#include <Wire.h> // Library for I2C communication with Real Time Clock.
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// --------------------------------------------------------------------------------------------------------------------------------
//Define name constants to refer to input and output pins

#define MOSFET 4 // Defines pin D4 to switch MOSFET on and thus power up RFID reader.
#define SWITCH 2 // Defines external switch connected to D2 that prompts processor to change voltage state of D4.

// --------------------------------------------------------------------------------------------------------------------------------
//Global constants and variable definitions

const int CardDetect = 6;      // the card detect pin, hard-wired to the SD-card holder on the Ardulog RTC.
const int greenLED = 5;        // green LED hard-wired on pin 5.

unsigned int baud = 9600;      // baud rate.
unsigned int i = 1;            // initialises a counter for logfile numbers (value of 0 would cause errors in program). 

char command_string[60];       // string to store command
unsigned char command_index;
char filename[13] = "YYMMDD00.CSV";   // filename to log data.
const char* Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"}; // the asterisk indicates that Day[] is an array of character arrays (i.e. a pointer).
char loggerID[] = "026";       // Determined here in code when loading firmware.
char siteID[7];                // To store 6-character siteID identified declared by operator in config.txt on SD card.
char operatorID[4];            // To store 3-character operatorID read from config.txt on SD card.
String dateString;
String timeString;
String tagID;                  // To store the incoming tagID.
byte data1 = 0;                // To store incoming data byte until added to tagID.
byte count = 0;                // Counts number of digits in incoming tagID. 
byte event = 9;                // Stores type of event to log. 1=Switch activated; 2=Tag read; 3=Follow on tag read within same switch window; 0=End of activation window.  Initialised with nonsense value. 
int switchValueD = 0;          // Stores status of switch when last polled.
unsigned long startTime = 0;   // Stores time on entering activation window.
unsigned long timeNow = 0;     // Stores time on entering pause.
unsigned long pauseLength = 10000;  // Determines how many milliseconds to pause before accepting another tag reading (10000ms=10s).
unsigned long windowLength = 60000; // Determines how many milliseconds to leave RFID reader powered after switch activated (1 min=60000ms). Must be >500 ms.

int seconds;
int minutes;
int hours;
int dow;                        //dow = day of the week
int days;
int months;
int years;
int idBit;

File configFile;
File logFile; 

// --------------------------------------------------------------------------------------------------------------------------------
//FUNCTION DECLARATIONS

//Convert BCD to decimal
int bcd2bin(int temp)
  {
      int a,b,c;
      a=temp;
      b=0;
      if(a>=16)
      {
  	while(a>=16)
	{
	  a=a-16;
	  b=b+10;
	  c=a+b;
          temp=c;
	}
      }
      return temp;
  }  //end of BCD to decimal

// Get current date and time stamp from RTC 
void getRTCdata(int& a,int& b,int& c,int& d,int& e,int& f,int& g) // '&' symbol makes function update the variables passed into the function by substituting a...g.  So no value returned, void loop. 
  {
    // First open master-slave relationship with RTC at RTC address
    Wire.beginTransmission(0x68);// 0x68 is I2C address of DS1338
    Wire.write(0);	
    Wire.endTransmission();//End of the I2C communication
    Wire.requestFrom(0x68,7);//I2C communication is started, reads 7 bytes.  In order, these are seconds, minutes, hours, dow, days, months, years.
    a = bcd2bin(Wire.read());
    b = bcd2bin(Wire.read());
    c = bcd2bin(Wire.read());
    d = bcd2bin(Wire.read());
    e = bcd2bin(Wire.read());
    f = bcd2bin(Wire.read());
    g = bcd2bin(Wire.read()); 
  }  //end of getRTCdata

// Construct and return date string to write to file
String getdateString (int dow,int days,int months,int years)
{
  dateString = "";
  dateString += Day[dow];
  dateString += ' ';
  dateString += days;
  dateString += '/';
  dateString += months;
  dateString += '/';
  dateString += years;
  return dateString;
}

// Construct and return date string to write to file
String gettimeString (int hours,int minutes,int seconds)
{
  timeString = "";
  timeString += hours;
  timeString += ':';
  timeString += minutes;
  timeString += ':';
  timeString += seconds;
  return timeString;
}

//Signal function: flashes green LED 6 times to indicate logger has read a tag (or other event - useful for debugging).
void signal()
{
  for (int k=0; k<6; k++) // Event signal is 5 quick flashes on green LED.
  {
    digitalWrite(greenLED, HIGH);  // Green LED on.
    delay(10);                     // On for 10 ms.
    digitalWrite(greenLED, LOW);   // Sets the LED off.
    delay(250);                    // Off for half a second.
  }
} // End of signal.

//Heartbeat function: flashes green LED to indicate logger is fit to record.
void heartbeat()
{
  for (int k=0; k<6; k++)  // Does 5 slow flashes (10 seconds' worth) before returning. 
  {
    digitalWrite(greenLED, HIGH);  // Green LED on.
    delay(20);                     // On for 20 ms.
    digitalWrite(greenLED, LOW);   // Sets the LED off.
    delay(2000);                   // Off for two seconds.
  }
} // End of heartbeat.

// ERROR function - Flash green LED warning signal.  Orange LED may flash briefly at the same time if error is related to SD card.
void error()
  {
     digitalWrite(greenLED, HIGH);  // Green LED on.
     delay(500);                    // On for 500 ms.
     digitalWrite(greenLED, LOW);   // Green LED off.
     delay(500);                    // Off for 500 ms.
  }  // End of error.

// Sleep.  Prepares the processor to enter the chosen sleep mode.  See http://www.gammon.com.au/interrupts
void sleepNow(void)
{
    // Choose preferred sleep mode.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    // Make sure further interrupts can't occur before going to sleep
    noInterrupts(); 
    // Set sleep enable bit.
    sleep_enable();    
    // Determine how interrupt will function.  LOW at pin 2 will trigger interrupt. Confusingly, pin 2 = interrupt 0. 
    attachInterrupt(0, pinInterrupt, LOW);
    // Now finally allow interrupts.  After an interrupts statement, the next instruction in the code WILL be executed, so sleep is now certain.
    interrupts(); 
    // Go to sleep using previously selected sleep mode.
    sleep_cpu();
}

// Interrupt service routine.  This allows sleep to be interrupted.
void pinInterrupt(void)
{
    sleep_disable();     // First thing after waking from sleep, disable sleeping.
    detachInterrupt(0);  // Stop LOW interrupts.
}

// --------------------------------------------------------------------------------------------------------------------------------/
// SET UP THE SYSTEM
// Define the input/output pins, check SD card and config file, then set up a file to record data for this session.
void setup()
{ 
  pinMode(CardDetect, INPUT);    // Card Detect on pin 6.
  pinMode(greenLED, OUTPUT);     // To indicate ready or error status via green LED on pin 5.
  pinMode(10, OUTPUT);           // CS (chip select) pin 10.
  pinMode(MOSFET, OUTPUT);
  digitalWrite(MOSFET, LOW);
  pinMode(SWITCH, INPUT_PULLUP); // Enable internal pull-up resistor on switch pin
  Serial.begin(9600);            // Initializes the Serial library.  
  Wire.begin();  //initializes  the Wire library and joins the I2C bus as master (because parameter not specified)
  
  // Check SD card present. LOW on pin A2 (card detect pin) means a card is inserted.
  while(digitalRead(CardDetect) == HIGH) 
  {   
   error(); // ERROR: no SD card.  Flash green LED in warning mode until problem rectified.
  }

  // OK, so an SD card is present. Check whether card can be written to.  Returns true on success, false on failure.
  while(!SD.begin(10))   // NB the optional parameter "10" indicates which is the CS pin.  On the Ardulog, pin 10 is hard-wired to select the SD card reader.
  {
   error(); // ERROR: can't write to card.  Flash green LED in warning mode until problem rectified.
  }    

  // SD card is present and usable if we reach this line.  Now need to read config.txt file.
  // First check whether config.txt file exists on the SD card.
  while(!SD.exists("config.txt")) 
  {
    error(); // ERROR: No config.txt file.  Flash green LED in warning mode until problem rectified.
  }
  configFile = SD.open("config.txt", FILE_READ);
  for (int c=0; c<6; c++)       // Start reading file.
  {
    char s = configFile.read(); // Read number of bytes prescribed by loop.
    if(s == '\n' || s == EOF)
    {
      error();                  // If line end or EOF is reached unexpectedly, go into ERROR mode until problem rectified.
    }
    else
    {
      siteID[c] = s;            // If not yet line end or EOF, write the next character to string.
    }
  }     
  siteID[6] = '\0';             // Write string terminator.
  char x = configFile.read();   // Read and ignore next character (can be any white space character such as new line '\n').
  char y = configFile.read();   // Read and ignore next character (can be any white space character such as new line '\n').
  for (int j=0; j<3; j++)       // Now continue reading more bytes as prescribed by fresh loop.
  {
    char o = configFile.read(); // Read number of bytes prescribed by loop.
    if(o == '\n' || o == EOF)
    {
      error();                  // If line end or EOF is reached unexpectedly, go into ERROR mode until resolved.
    }
    else
    {
      operatorID[j] = o;        // Write the character to string.
    }
  }
  operatorID[3] = '\0';         // Write string terminator.
  configFile.close();  // Close config file again immediately, otherwise power-off or SD card removal would lose data.
  
  // Now create file to record data.
  // First compose filename string.
  i=1;
  getRTCdata(seconds,minutes,hours,dow,days,months,years);
  filename[0] = years/10 + '0';
  filename[1] = years%10 + '0';
  filename[2] = months/10 + '0';
  filename[3] = months%10 + '0';
  filename[4] = days/10 + '0';
  filename[5] = days%10 + '0';
  filename[6] = i/10 + '0'; //calculates 'tens' in current value of i and adds null bit
  filename[7] = i%10 + '0'; //calculates 'units' in current value of i and adds null bit
 
  // Check whether any file for this date already exists - e.g. if logger was already reset today- and set appropriate file number for this date
  while(SD.exists(filename)) 
  {
    i = i+1;
    // Check not more than 99 files already created on this date.  SD card 8.3 filename format means this must not exceed 99.
    while(i>99)
    {
     error(); // ERROR: flash green LED in warning mode until problem rectified
    }
    filename[6] = i/10 + '0'; //calculates 'tens' in current value of i and adds null bit
    filename[7] = i%10 + '0'; //calculates 'units' in current value of i and adds null bit
  }
  
  // Now we have a unique filename for this logging session, so create the file by opening it, and write header line
  logFile = SD.open(filename, FILE_WRITE); // create file 
  logFile.println("date, time, siteID, event, tag_ID, loggerID, operatorID"); //Print field headers to log file
  event=0; // Indicates this is a record of the session start
  getRTCdata(seconds,minutes,hours,dow,days,months,years);
  getdateString(dow,days,months,years);
  gettimeString(hours,minutes,seconds);
  //Print session start values.  Fields that could have leading zeros are surrounded by quotes (\") otherwise Excel thinks they are numbers and trims leading zeros.
  logFile.println(dateString + "," + timeString + "," + "\"" + siteID + "\"" + "," + event + "," + "\"" + tagID + "\"" + "," + "\"" + loggerID + "\"" + "," + operatorID); 
  logFile.close();  // Need to close file again immediately, otherwise power-off or SD card removal would lose data

  // OK, ready to log, show heart-beat flashes. 
  heartbeat();
  // Move on to main loop.
} 

// --------------------------------------------------------------------------------------------------------------------------------
// MAIN SECTION
// This is the main part of the code where the minute-by-minute action happens
void loop()
{
  // Poll current status of switch.
  // NB for squirrel hoppers, switch is wired as normally closed (NC), but resting state (lid down) has switch depressed so its resting state is open.  
  // Pin D2 (SWITCH) is pulled HIGH by the internal pullup, hence switchValueD == 1 means the device is 'inactive'.
  switchValueD = digitalRead(SWITCH);

  // If switch not activated, flash twice and go to sleep
  if (switchValueD == 1) 
  {
    error(); // flash
    error(); // flash
    sleepNow();
  }

  // On waking up (because of switch activation) main loop resumes here
    event=1;  // It's a switch activation event.  Something is there, but there's no ID yet, and it could be an untagged animal anyway.

    // Switch on RFID reader and make a record that an activation window has been started.
    digitalWrite(MOSFET, HIGH);
    signal();
    tagID = "";
    unsigned long startTime = millis();
    getRTCdata(seconds,minutes,hours,dow,days,months,years);
    getdateString(dow,days,months,years);
    gettimeString(hours,minutes,seconds);
    logFile = SD.open(filename, FILE_WRITE);  
    logFile.println(dateString + "," + timeString + "," + "\"" + siteID + "\"" + "," + event + "," + "\"" + tagID + "\"" + "," + "\"" + loggerID + "\"" + "," + operatorID);
    logFile.close();

    // Await tag ID data on RX pin until activation window times out.
    do
    {
		while (Serial.available() > 0)   // If serial data on offer, do this section of code once, then repeat DO loop.
	        {	
      	    // Accumulate tagID string.
			count = 0; tagID = "";
			// Read next byte on serial RX (digital pin 0)
			data1 = Serial.read();
			delay(30); // 30 millisecond delay necessary to ensure clean tag read.
			idBit = data1;
			  if (idBit == 2) // If 'start of tag' byte (2) is found
				{ 
					tagID += data1;
					tagID += ' ';
					count += 1;
					// Collect rest of ID
					do 
					{
						// Read next byte on serial RX (digital pin 0)
						data1 = Serial.read();
						// Add byte to tagID character string until there are 14 bytes
						tagID += data1;
						tagID += ' ';
						count += 1;
					} while (count < 14);
                    idBit = data1;
			        if (idBit == 3) // If 'end of tag' byte (3) was received to complete ID string
                    {
						signal();					
                    }
					// Got ID data!  Either promote event to 2 (first read in this activation window), or leave it as 3 (same window, subsequent read).
					if (event == 1) 
					{
						event = 2;     
					}
					// Record tag read event in the data file.
					getRTCdata(seconds,minutes,hours,dow,days,months,years);
					getdateString(dow,days,months,years);
					gettimeString(hours,minutes,seconds);
					logFile = SD.open(filename, FILE_WRITE);  
					logFile.println(dateString + "," + timeString + "," + "\"" + siteID + "\"" + "," + event + "," + "\"" + tagID + "\"" + "," + "\"" + loggerID + "\"" + "," + operatorID);
					logFile.close();
					event = 3;     // The next tag read within this activation window will be a follow-on event.       
				}
    
			// If tag has just been read, pause before reading again, to avoid too many records of same tag.  Keep draining serial buffer.
			// [This is arguably inefficient programming, because the processor cannot do anything else (e.g. read a different tag) while this is happening.
			// In the squirrel context, that is fine, because only one squirrel can fit in the feeder!  If several tagged animals could present themselves within one pauseLength,
			// an alternative arrangement would be to record only IDs that had not already been read within the same pauseLength.  This would obviously require extra coding.]
                    if (event != 1)
                    {
					timeNow = millis();
					while (millis() - timeNow < pauseLength)
						{
						if (Serial.available()) 
							{
							Serial.read();
							}
						}
                    }  
		} 
    } while (millis() - startTime < windowLength);
    // Activation window timed out.  Switch off 5V supply to tag reader. 
    digitalWrite(MOSFET, LOW);
    // Record end of activation window.
	event = 4;  // i.e. window end.
	getRTCdata(seconds,minutes,hours,dow,days,months,years);
	getdateString(dow,days,months,years);
	gettimeString(hours,minutes,seconds);
	logFile = SD.open(filename, FILE_WRITE);  
	logFile.println(dateString + "," + timeString + "," + "\"" + siteID + "\"" + "," + event + "," + "\"" + tagID + "\"" + "," + "\"" + loggerID + "\"" + "," + operatorID);
	logFile.close();
 
}  //  Go back and poll switch again.  A new activation window can start immediately if the switch indicates an animal is still present.
 

// END
// --------------------------------------------------------------------------------------------------------------------------------
