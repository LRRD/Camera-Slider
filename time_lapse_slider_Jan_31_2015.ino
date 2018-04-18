 #include <Stepper.h>

//Version 10-15 7:35

// Rev. October 22, 2013
//Time lapse slider code
//Mostly Steve Gough, Sept 2013
//Some snips from Rugged Driver site http://ruggedcircuits.com/html/rugged_motor_driver.html

//Version 11 Jan 24, 2015.  Seems to be working OK.
//Rev 12 Jan 31,2015; lots of work and bug fixing.

// include the Adafruit backpack library code:
#include "Wire.h"
#include "LiquidCrystal.h"
 
/*

To do.
1.  Figure out how to adjust camsettle variable in the field so it can be long for long exposures.
2.  Still not sure if step and time calculations are correct; esp. runtime calc v. actual
3.  Figure out steps per inch of travel so maxSteps can be calculated for different rails.  Also need to be able to field set this.


*/

LiquidCrystal lcd(0);

// Define how many steps there are in 1 revolution of your motor
#define STEPS_PER_REVOLUTION 200

// Enable (PWM) outputs
#define EN1_PIN 3
#define EN2_PIN 11

// Direction outputs
#define DIR1_PIN 12
#define DIR2_PIN 13

Stepper stepper(STEPS_PER_REVOLUTION, DIR1_PIN, DIR2_PIN);

/////////////////////////////////////////////////////////////////////////////////  SET PWM AND RPM HERE
// Set initial default values


volatile unsigned RPM = 50;           //40 seems to work well, higher than 60 or so does not work well at all.
					//NOTES ON RPM:  1) vibrates like a steamboat at 10 rpm. 
					 			//All these done with PWM set at 70
					//  -- 20 rpm better sound, but still a lot of vibration
					//  --  30  plenty fast for traverses, but annoying sound, still; quite a bit of vibration
					//  35  better sound and less vibration
					//  45  sounds more stepper like; pretty good, moves quick but not too quick
					//  50  very nice and smooth; best so far
					//   60  starting to be a bit harsh noise/vibration-wise, so 50 is !T!
					
					
volatile unsigned PWM = 70
	;    //sets power of the motor; also can heat driver if set high; still
						//not sure; chips got very hot at 100% while holding; check this
						// NOTE:  NEEDS TO BE 70 OR LARGER TO HOLD/MOVE CARRIAGE ON VERTICAL CLIMB
						//Did a bunch of experimentation Sep 29 2013, pwm at 70 and RPM at 50 is best, increase PWM if carriage slips
						//when used in vertical position
						
						
int liteVal = 200 ;  //PWM value for backlight
						
						
int DIR = 1;

unsigned long notch_Time = 0;   // set to 3 seconds.
int shutter_Pin = 7;  //relay to fire shutter 
int step_Travel = 0;  //tracks number of steps so carriage can return home
int notch_Steps = 0; //number of steps per notch

int potpin = 16;  // analog pin A2; TIME DELAY AT EACH NOTCH IN MILLIS
int potpin2 = 17; // analog pin A3 (last one); NUMBER OF STEPS PER NOTCH
////////////////////  Note pins A4 and A4 are IC2 and used for Adafruit serial backpack to drive LCD display
int stop_Pin = 2;
int stop_Switch = 0;  //////////////////////////////Very important this be set right depending
						//on the type of stop switch used!  Otherwise crazy behavior.
int ledPin = 4;      //lights when shutter is fired, very useful for remote verification that
					//things are working
int battPin =A0 ; //voltage divider connected to battery in goes to Analog 0					

int coilPin = 5;    //fault pin on Rugged Driver disabled; would normally use this pin; add switch and pulldown resistor
int backlitePWMpin = 10;
int sweePin = 8;  //Changed this on Jan 26 2015; will be set on switch and used to run manual sweep program; "run" happens with analogRead >700.
										
int coilSwitch = 0;  //switch to turn coils on/off for holding on non-horizontal slider stetups					
int stepCount = 0;
unsigned long shutterPress = 1000;
unsigned long camSettle = 10000;  //for long exposures; set to 10 seconds, probably a variable you'd want to adjust in the field somehow.
int ramp_Lap = 1;   //keeps track of "laps" up and down ramp, first trip is 1, second is 2.

unsigned long totalDelay = 0; //notch delay + camSettle + shutterPress; the total time the carriage sits w/o moving
unsigned long runTime = 0;  // total time in minutes of run, calculated from delay and notches
unsigned long runStartmillis = 0;  //holds millis() value at time of run start
unsigned long timeElapsed = 0;  //runtime elapsed, seconds
unsigned long timeRemain = 0;  //time remaining in Lap 1, seconds

float battVolts = 0.00 ; //battery voltage from pin A0 mapping
int toggle = 0 ; //toggles display
//diagnostics
unsigned long shutterStart = 0;
unsigned long shutterStop = 0;
int sweepSpeed = 0;  //value for speed of sweep in sweep mode
int sweeporNot = 0;
int battPinraw = 0;
int battTemp = 0;
int maxSteps = 300;  //number of steps carriage can travel on given beam
int switchPin = 15;  //rotatry switch set like potentiometer
long time6 = millis();
int frameCount = 0;  //tracks number of frames shot and displays

void setup()   ////////////////////////////////////////////////////////////////////////
	{
    	Serial.begin(9600);
		digitalWrite(stop_Pin, HIGH);  /////////////set pullup resistor (actually stop pin is no longer used as of Oct 22)

		  // set up the LCD's number of columns and rows: 
		  lcd.begin(20,4);
		 //lcd.setBacklight(HIGH);		
		pinMode (backlitePWMpin, OUTPUT);
		analogWrite(backlitePWMpin, liteVal);  //PWM pin connected to pin 15 on LCD; which is disconnected
											//from the Adafruit backpack; this has to be ANALOGwrite,
											//not "digitalWrite" you big dummy!
		digitalWrite(sweePin, HIGH);   //set pullup resistors; pin is pulled low when run switch is closed
											
		lcd.clear();
		  // set the cursor to column 0, line 1
		  // (note: line 1 is the second row, since counting begins with 0):
		  lcd.setCursor(0, 0);
		  // Print a message to the LCD.
		  lcd.print("  LRRD time    ");
		lcd.setCursor(0,1);
		  lcd.print("  lapse slider   ");
		lcd.setCursor(0,2);
		
		lcd.print("V. 12 Jan 31, 2015");
		lcd.setCursor(0,3);
		lcd.print ("batt voltage ");
		//calculate voltage; R1 = 100K, R2 = 200K
		battPinraw = analogRead(battPin);
		battTemp = map(battPinraw, 0, 1023, 0, 1544); 
		battVolts = battTemp / 100.00 ; //turn around divider formula since we know Vout, need Vin
										//note this did not work when only "100" is in equation, must be "100.00"
										//this drives me nuts.............
			lcd.print (battVolts);
			lcd.print ("  ");
			

		
		
		delay(2000);  //show batt voltage for four seconds

		pinMode (stop_Pin, INPUT);
		pinMode (shutter_Pin, OUTPUT);
		digitalWrite(shutter_Pin, LOW);		//SET PULLDOWN RESISTOR (there is no such thing!  Only pullups)
		pinMode (ledPin, OUTPUT);
		digitalWrite (coilPin, HIGH);  // sets coils on/off pin to high (coils off); switch ON pulls low
		pinMode(backlitePWMpin, OUTPUT);
		pinMode(sweePin, INPUT);
		digitalWrite(sweePin, HIGH);   //set pullup resistor; switch pulls pin LOW for RUN mode
		
		  // Configure all outputs off for now
		  pinMode(EN1_PIN, OUTPUT); digitalWrite(EN1_PIN, LOW);
		  pinMode(EN2_PIN, OUTPUT); digitalWrite(EN2_PIN, LOW);
		  pinMode(DIR1_PIN, OUTPUT); digitalWrite(DIR1_PIN, LOW);
		  pinMode(DIR2_PIN, OUTPUT); digitalWrite(DIR2_PIN, LOW);
		  // Configure fault inputs with pullups
		  //pinMode(FAULT1_PIN, INPUT); digitalWrite(FAULT1_PIN, HIGH);
		  //pinMode(FAULT2_PIN, INPUT); digitalWrite(FAULT2_PIN, HIGH);

		  // Change from divide-by-64 prescale on Timer 2 to divide by 8 to get
		  // 8-times faster PWM frequency (976 Hz --> 7.8 kHz). This should prevent
		  // overcurrent conditions for steppers with high voltages and low inductance.
		 TCCR2B = _BV(CS21);   //Gough note:  THIS REALLY REDUCES NOISE AND VIBRATION

		  // Now enable PWM and start motion
		  analogWrite(EN1_PIN, PWM);			///this is standard Arduino language; pwm varies from 
		  analogWrite(EN2_PIN, PWM);			/// 0 to 255 (to give 0 to 100%)
		  stepper.setSpeed(RPM);
		
///////////////////////Home the carriage  /////////////  Removed
////////Turns off coils if coil switch is set to off
			if (digitalRead(coilPin) == LOW)		// turns off coils during wait if switch is on
			{

				digitalWrite(EN1_PIN, LOW);
				digitalWrite(EN2_PIN, LOW);
				//digitalWrite(DIR1_PIN, LOW);  // no need to send DIR pin low I think 
				//digitalWrite(DIR2_PIN, LOW);
			}

///Error message if rotary switch is on "run"
			if(analogRead(switchPin) > 750 )	
			{
				lcd.clear();
			lcd.print ("ROTARY SWITCH");
			lcd.setCursor (0,1);
			lcd.print("IS SET TO RUN"); 
			
			//fast blink green LED
				long interval = 150;           // interval at which to blink (milliseconds)
				int ledState = LOW;             // ledState used to set the LED
				long previousMillis = 0;        // will store last time LED was updated
				unsigned long currentMillis = millis();

				  if (currentMillis - previousMillis > interval) 
				{
				// save the last time you blinked the LED 
				    previousMillis = currentMillis;   

				    // if the LED is off turn it on and vice-versa:
				    if (ledState == LOW)
				      ledState = HIGH;
				    else
				      ledState = LOW;

				    // set the LED with the ledState of the variable:
				    digitalWrite(ledPin, ledState);
				}												//end fastblink LED

			while(analogRead(switchPin) > 750)
			{
			lcd.setCursor (0,2);	
			lcd.print ("move to ON now");
			lcd.setCursor(0,2);
			delay(600);
			lcd.print ("                    ");
			delay(400);
			lcd.setCursor(0,2);
			lcd.print ("move to ON now");
			}

			}
			
	}  ///////////////////////////////////////////////////////////////////////End setup
	
	
	void sweeptest()
	{
		while(1)
		{
			int potreading = 0;
		potreading = analogRead(potpin2);
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("right pot (2)  ");
		lcd.print(potreading);
		int potread0 = 0;
		potread0 = analogRead(potpin);
		lcd.setCursor (0,1);
		lcd.print("left pot  ");
		lcd.print(potread0);
		int switchread = 0;
		switchread = analogRead(switchPin);
		lcd.setCursor (0,2);
		lcd.print("switch ");
		lcd.print(switchread);
		delay(200);
			
		}		
		
	}
	
	void manSweep()
	{
		//LCD prompts; if you're here, sweePin switch is low
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("manual sweep is ON");
	
		lcd.setCursor(0,1);
		lcd.print(" center LH pot");
		lcd.setCursor(0,2);
		lcd.print("  to begin");
		delay (500);  // short delay in case pot is already centered
		
		int rawpot = analogRead (potpin);  //read pot state
		

	 		if (( rawpot < 600) && (rawpot > 400))  // pot in dead band; begin manual control
	 	   	{
				lcd.clear();
				lcd.setCursor(0,1);
				lcd.print(" manual sweep active");
				delay (2000);
				lcd.clear();
				lcd.setCursor(0,2);
				lcd.print ("step count");
			}
		
  		  //analogWrite(EN1_PIN, 200);			///sets higher PWM just for manual sweep
  		  //analogWrite(EN2_PIN, 200);
		
			//pot is outide of deadband, wait until centered
		if ((rawpot > 560) || (rawpot <450))  //while pot is outside deadband
			{
			rawpot = analogRead (potpin);   //read the pot, you big dummy
		 		if (( rawpot < 550) && (rawpot > 480))  // pot in dead band
				{
					lcd.clear();
					lcd.setCursor(0,1);
					lcd.print(" manual sweep active");
					delay (2000);
					lcd.clear();
					lcd.setCursor(0,2);
					lcd.print ("step count");
				}
					//note we have a "deadband" between 480 and 550.
			}	
		
		// set step counter to zero outside of while statement
		int stepCount = 0;

		//int RPMtemp = 0;
		
		while (digitalRead(sweePin) == LOW)  //sweep switch is on
		{		
			rawpot = analogRead(potpin);
			

			
			
			while (( rawpot > 400) && (rawpot < 600))  //center "deadband"
				{
				coilsOFF();   // disables coils if knob's in deadband; to stop motor
				//do nothing in deadband
				//also prints stepcount at a time when it won't block stepping speed

					for (int i=0; i <= 2; i++)
					{
						lcd.setCursor(0,3);
						lcd.print("     ");
						lcd.print (stepCount);
						lcd.print("     ");
						rawpot = analogRead(potpin);
					}
				
				}
			
				/*
			if (rawpot <= 513)
				RPMtemp = map(rawpot, 0, 513, 150, 40);
				
			if (rawpot > 513)
					RPMtemp = map(rawpot, 513, 1023, 40, 150);
			*/						//took out variable speed; not working well
							
			if (rawpot	<= 513)
				DIR = -1;
			if (rawpot > 513)   // set stepper direction by pot position either side of deadband
				DIR = 1;
			coilsON();	
			
			stepper.setSpeed(50);	
			
			stepper.step(DIR);
			//long dir2 = DIR;     //converts DIR from int to LONG for math
			stepCount = stepCount + DIR;
			
						//lcd.setCursor(0,0);
						//lcd.print(stepCount);
						//lcd.print("    ");	
			
			/*
			lcd.setCursor(0,0);
			lcd.print ("step count");
			lcd.setCursor(0,1);
			lcd.print(stepCount);
			lcd.print("        ");
			lcd.setCursor(13,1);
			if (DIR == 1)
				lcd.print (">>>");
					if (DIR == -1)
				lcd.print("<<<");
					lcd.print("   ");
			*/
					
							
			/*lcd.setCursor(0,2);
			lcd.print("rawpot= ");
			lcd.print(rawpot);
			lcd.print("   ");
			lcd.setCursor(0,3);
			lcd.print("RPMtemp = ");
			lcd.print (RPMtemp);
			lcd.print("   ");
			//delay(300);
			*/
			//Serial.println("running");
					
			//}
			
		}
		
	}   ///end mansweep
			
			
	/*  >>>>>>>>>>>>>>  disabled Jan 26 2015; don't think this is needed any more	
	void fastBlink()
	{
		if(digitalRead(runPin) == LOW )	
		{
			lcd.clear();
		lcd.print ("ROTARY SWITCH");
		lcd.setCursor (0,1);
		lcd.print("IS SET TO RUN");
		
		//fast blink green LED
			long interval = 150;           // interval at which to blink (milliseconds)
			int ledState = LOW;             // ledState used to set the LED
			long previousMillis = 0;        // will store last time LED was updated
			unsigned long currentMillis = millis();

			  if (currentMillis - previousMillis > interval) 
			{

		
			// save the last time you blinked the LED 
			    previousMillis = currentMillis;   

			    // if the LED is off turn it on and vice-versa:
			    if (ledState == LOW)
			      ledState = HIGH;
			    else
			      ledState = LOW;

			    // set the LED with the ledState of the variable:
			    digitalWrite(ledPin, ledState);
			}
			
		}
	}
	*/		
	void coilsON()
	{
		analogWrite(EN1_PIN, PWM);			///this is standard Arduino language; pwm varies from 
	  	analogWrite(EN2_PIN, PWM);			/// 0 to 255 (to give 0 to 100%)
		stepper.setSpeed(RPM);

	}

							void coilsOFF()   ///turns off all coils  NOTE:  only works if coilswitch is in OFF position
							{
								if (digitalRead(coilPin) == LOW)		// turns off coils during wait if switch is on
								{
	
								digitalWrite(EN1_PIN, LOW);
								digitalWrite(EN2_PIN, LOW);
							  	 //digitalWrite(DIR1_PIN, LOW);
							  	 //digitalWrite(DIR2_PIN, LOW);
								}
							}

void shutter()			///releases shutter
{
		
digitalWrite(shutter_Pin, HIGH);  //fire shutter with relay
digitalWrite(ledPin, HIGH);

frameCount = frameCount +1;   //increments frameCount each time shutter is pressed

Serial.println ("shutter pressed.....................................");
Serial.println ("     ");

delay(shutterPress);					//hold shutter for one second
digitalWrite(shutter_Pin, LOW);	//shutter fire turned off

digitalWrite(ledPin, LOW);  // turns on LED during camSettle for visual indication of shutter firing

delay(camSettle);					//give camera a second to settle before vibration of movement begins


/////////////////////////////IMPORTANT;  if shutter speed exceeds one second plus notch_Time, movement
							/////will begin before shutter closes
}

void stop()   // endless loop that shuts down motor coils and blinks LED
{
    //First disable all pins so motor is not holding while it waits 
 digitalWrite(EN1_PIN, LOW);
 digitalWrite(EN2_PIN, LOW);
  //pinMode(DIR1_PIN, OUTPUT); digitalWrite(DIR1_PIN, LOW);
  //pinMode(DIR2_PIN, OUTPUT); digitalWrite(DIR2_PIN, LOW);

	 while(1)		//should run forever
	{
		//Serial.println("   stopped    /////////////");
		digitalWrite (ledPin, HIGH);	// flash LED to signal sequence is over
		delay(750);
		digitalWrite (ledPin, LOW);
		delay(750);
	}  
}

////////////////////////////////////////////////////////////////

void setDials()   /////////////////////////////////////////////////////////////////////////////////////
{


	lcd.clear();
	
	for (int i = 0; i < 40; i= i++)  ///fills screens with " - " is nice splash
		lcd.print("- ");
		
	lcd.clear();

	if  (analogRead(switchPin) <100)  //switch is set to OFF
		{

		lcd.print("   rotary switch");
		lcd.setCursor(0,1);
		lcd.print("   is set to OFF");
		}	
	
		while (analogRead(switchPin) <100)  //switch is set to OFF; read switch and wait
			{
			if (analogRead(switchPin) > 100)  //break if switch moved off "off" position
			{
				lcd.clear();
				break;
			}
			
			}
				
while(analogRead(switchPin) < 590)  //position 4 (last before "run") returns ~615

{
	
	while(analogRead(switchPin) <250) // switch position 2 reads 203, next position is 408; set camsettle and max steps
	
	{
		

			camSettle = analogRead(potpin);
			camSettle = map(camSettle, 0, 1023, 25000, 2000);   //map camSettle from 2 to 20 seconds
			//lcd.clear();
			lcd.setCursor(0,0);
			lcd.print( "camSettle = ");
			lcd.print((camSettle/1000));
			lcd.print(" sec  ");
			lcd.setCursor(0,1);
			lcd.print("                  ");


				maxSteps = analogRead(potpin2);
				maxSteps = map(maxSteps, 0, 1023, 20000, 1000);   //map 
				lcd.setCursor(0,2);
				lcd.print( "maxSteps = ");
				lcd.print(maxSteps);
				lcd.print(" ");
				lcd.setCursor(0,3);
				lcd.print("dist = ");
				float mmval = (maxSteps * 0.203);  //mm per step; measured Jan 31 2015
				int mmvalint = int(mmval);
				lcd.print (mmvalint);    // convert float to int and print to LCD
				lcd.print(" mm ");
	

			if (analogRead(switchPin)<150 or analogRead(switchPin)>250)  //rotary moved off position 2
			{
				lcd.clear();
				break;
			}
		
	}	
	
	while(analogRead(switchPin) > 300 and analogRead(switchPin)<500)  //position three actual return is 408
																		//set notch steps and delay
	
	{
	
		notch_Steps = analogRead(potpin2);            // reads the value of the potentiometer (value between 0 and 1023)  
		notch_Steps = map(notch_Steps, 0, 1023, 500, 10);     // scale for 10 to 300 steps full scale of pot

		
		notch_Time = analogRead(potpin);				//read time potentiometer
		notch_Time = map(notch_Time, 0, 1023, 50000, 1000);  //sets notch wait from 2 to 50 seconds
		
		
			lcd.setCursor(0,0);
			lcd.print("notch steps = ");   //Prints number of steps the motor takes per notch
			lcd.print(notch_Steps);

			lcd.print("  ");  //chops off any lingering digits at end

				lcd.setCursor(0,1);
					lcd.print("delay = ");   
					totalDelay = (notch_Time + camSettle + shutterPress)/1000;

					lcd.print(totalDelay);  //shows total delay in seconds as knob is adjusted
					lcd.print(" seconds  ");  //chops off any lingering digits at end

					Serial.print("notch_Time");
					Serial.println(notch_Time);
					Serial.print("camSettle ");
					Serial.println(camSettle);
					Serial.print("shutterPress ");
					Serial.println(shutterPress);
					Serial.println("             ");
					//delay(1500);

		//Calculate and display on third row		
				int notchSecs = (notch_Time/1000);  //convert notch_Time in millis to seconds
				lcd.setCursor(0,2);
			lcd.print ("notches = ");
						float totalNotches = maxSteps/notch_Steps;
						int totesNotches = int (totalNotches);  //cast to int
				lcd.print (totesNotches);
				lcd.print("   ");

				unsigned long totalTime = int (totalNotches) * (notchSecs + totalDelay);
				lcd.setCursor(0,3);
			lcd.print ("runtime = ");
			runTime = (totalTime);
				lcd.print (runTime/60); //prints total running time in minutes
				lcd.print (" min  ");

				if (analogRead (switchPin) <300 or analogRead(switchPin) >500)  //breaks out switch is moved off position three
				{
					lcd.clear();
					break;
				}	
		
	}
	
		
	
} 

}  //close setDials function

void lcdUpdate()   /////////////////////////////////////////////////////////////////////////////////////
{
	
				//checks potpin2 and sets PWM for backlight; no interaction, just reads and sets
				
				liteVal = analogRead(potpin2);            // reads the value of the potentiometer (value between 0 and 1023)  
				liteVal = map(liteVal, 0, 1023, 255, 25);     // scale for pwm values for backlight
				//Serial.println ("ran do while liteVal  ");
				//Serial.println (" --------");
				analogWrite(backlitePWMpin, liteVal);
				
	if (sweeporNot == 0)      //rest of this runs only in non-sweep mode
	{
			
	lcd.setCursor(0,0);
	lcd.print("runtime = ");
	int runMins = runTime/60;
		lcd.print( runMins);
		lcd.print ( "m ");
		int secs = runTime - (runMins *60);
			lcd.print( secs);
		lcd.print(" s ");
		
	lcd.setCursor(0,1);
	lcd.print("elapsed secs = ");
	lcd.print ((millis() - runStartmillis) / 1000);
		lcd.print("  ");
	
	lcd.setCursor (0,2);
	lcd.print ("remain secs ");
	long remainSecs = (runTime) - ((millis() - runStartmillis) / 1000);
	lcd.print("  ");
	lcd.print (remainSecs);
	lcd.print("  ");
	lcd.setCursor (0,3);
	lcd.print("                   "); //clears the line so residual battery volts digits are gone
	lcd.setCursor (0,3);
	
	switch (toggle)     //Simply switches toggle back and forth so that the value
						//can be used to alternately display batt voltage and total steps
						//if variable toggle = 1 it's set to zero and vice versa
	{
		case 1:
		toggle = 0;
			break;
		case 0:
		toggle = 1;
		break;
	}
	
	if (toggle == 0 )
		{
		lcd.print ("batt voltage  ");
		//calculate voltage; R1 = 100K, R2 = 200K
		battPinraw = analogRead(battPin);
		battTemp = map(battPinraw, 0, 1023, 0, 1544);  // zero to 5 volts mapped for 1 to 1544
		//because voltage divider gives 5 volts at 14.9 volts; this equation includes divider
		//calculations and also givve significant digits
		//map function won't work with lower values because it generates integers only
		battVolts = battTemp / 100.00 ; //turn around divider formula since we know Vout, need Vin
										//note this did not work when only "100" is in equation, must be "100.00"
										//this drives me nuts.............
			lcd.print (battVolts);		
		}		
	
	if (toggle == 1)						
		{
			lcd.print ("Estep ");

				lcd.print (step_Travel);
				lcd.print(" ");
				lcd.print("frame ");
				lcd.print(frameCount);
		}							
	
		Serial.print ("rampLap  ");
		Serial.println (ramp_Lap);
		Serial.print ("notch steps ");
		Serial.println (notch_Steps);
		Serial.print ("notch_Time ");
		Serial.println (notch_Time);
		Serial.print ("total delay  ");
		Serial.println (totalDelay);
	Serial.print ("runtime  ");
	Serial.println (runTime);
	Serial.print ("millis  ");
	Serial.println(millis());
	Serial.print ("runStartmillis  ");
	Serial.println(runStartmillis);
	Serial.print ("remainSecs  ");
	Serial.println( remainSecs);
	Serial.print ("steps so far  ");
	Serial.println (step_Travel);
	//Serial.print ("liteVal  ");
	//Serial.println (liteVal);
	Serial.println ("  ----------------");
	
	
	}  //end sweeporNot if statement
	
}


void loop()    //////////////////////////////////////////////////////////////////////
{
	coilsOFF();

	if(digitalRead(sweePin) == LOW)
		manSweep();
	
	setDials();
	runStartmillis = millis();	
	lcdUpdate();
	
	step_Travel = 0; 
	
	if  (analogRead(switchPin) <100)  //switch is turned to OFF
		stop();	

	while (analogRead(switchPin) >700)  //position 4 on switch
	
	{
		
		shutter();  //take the first picture before moving
	
	while (ramp_Lap ==1)
	{
		if  (analogRead(switchPin) <100)  //rotary switch moved to OFF position
			stop();							//stops program run
		
		coilsON();	
	    analogWrite(EN1_PIN, PWM);
	    analogWrite(EN2_PIN, PWM);
	    stepper.setSpeed(RPM);
		
		
		step_Travel = step_Travel + notch_Steps ;
		

		stepper.step(notch_Steps);   // step forward notch_Steps
		step_Travel = step_Travel ++ ;
		
		coilsOFF();
															
					shutterStart = millis();

					unsigned long delayLoop = millis();	
					unsigned long timer = millis();
						while (delayLoop + notch_Time > millis())  //counts off time until notch_Time passes
						{	
							
							if (millis()- timer >=500)  //updates LCD every 2 seconds
							{
							lcdUpdate();
						 	timer = millis();					
							}	
						}		
			
			shutter();
			lcdUpdate();
			
			//if (digitalRead(coilPin) == LOW)		//turns on coils for move if they were turned off
			//coilsON();	
				shutterStop = millis();
					
					if (step_Travel >= maxSteps)
					{
						ramp_Lap = 2;
						step_Travel = 0;	//reset step counter for next lap
						break;
					}
																
	}
	
	while (ramp_Lap ==2)
	{
		if  (analogRead(switchPin) <100)  //rotary switch moved to OFF position
			stop();							//stops program run
		
		coilsON();					//make sure coils are on and PWM is correct
						//I made lots of bugs with other functions, this is a "just in case"
	    analogWrite(EN1_PIN, PWM);
	    analogWrite(EN2_PIN, PWM);
	    stepper.setSpeed(RPM);
		
		
		step_Travel = step_Travel + notch_Steps ;
		

		stepper.step(-notch_Steps);   // step backward notch_Steps; only difference between this
		step_Travel = step_Travel ++ ;	// and lap 1 is the minus sign here
		
		coilsOFF();
															
					shutterStart = millis();

					unsigned long delayLoop = millis();	
					unsigned long timer = millis();
						while (delayLoop + notch_Time > millis())  //counts off time until notch_Time passes
						{	
							
							if (millis()- timer >=500)  //updates LCD every 2 seconds
							{
							lcdUpdate();
						 	timer = millis();					
							}	
						}		
			
			shutter();
			lcdUpdate();
			
			//if (digitalRead(coilPin) == LOW)		//turns on coils for move if they were turned off
			//coilsON();	
				shutterStop = millis();
					
					if (step_Travel >= maxSteps)
					{
						ramp_Lap = 1;
						step_Travel = 0;	//reset step counter for next lap
						break;
					}
																
	}
	
	}   ///end while switchpin >700  
			


}    ///////////////////   END  LOOP








////////////////////////////reads resistor values for switch; diagnostic/design only
/*
while (1<2)
{
	int test = analogRead(15);
		Serial.println (test);
		delay (250);
		
}
*/		
	//sweeptest();

		
					/*
if (serialDiag == 1)
  
  {
	  Serial.print ("rampLap  ");
Serial.println (ramp_Lap);
Serial.print ("notch steps ");
Serial.println (notch_Steps);
Serial.print ("notch_Time ");
Serial.println (notch_Time);
Serial.print ("total delay  ");
Serial.println (totalDelay);
Serial.print ("runtime  ");
Serial.println (runTime);
Serial.print ("millis  ");
Serial.println(millis());
Serial.print ("runStartmillis  ");
Serial.println(runStartmillis);
//Serial.print ("remainSecs  ");
//Serial.println( remainSecs);
Serial.print ("steps so far  ");
Serial.println (step_Travel);
//Serial.print ("liteVal  ");
//Serial.println (liteVal);
Serial.println ("  ----------------");
					
				

//if (digitalRead(coilPin) == LOW)		// turns off coils during wait if switch is on
	//coilsOFF();

	}*/






