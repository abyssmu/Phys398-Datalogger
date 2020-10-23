/*************************************************************************************
  This program is PPS_study.ino

  Let's study how fast we can interrogate the GPS PPS pin, tied to Arduino pin D43.

  You will need to have a GPS running properly, in contact with satellites and
  properly time-synched to UTC, for this to work properly. 

  Looking at a pair of data loggers with purportedly identical Arduinos (both are of
  Italian manufacture), I find that the micros() count between GPS PPS pulses are 
  typically 999,958 for one Arduino and 1,000,224 for the other. This is a differnce of
  about 0.064%. Note that this corresponds to a discrepancy between the two devices 
  of 640 microseconds per second.

  I am using two different methods of interrogating poin D43, which is where I route 
  the GPS PPS signal:
      GPS_PPS_value = digitalRead(GPS_PPS_PIN);
  and 
      GPS_PPS_value = (*myPin_port & myPin_mask);

  The first method takes about 3.84 microseconds per call. The second is much faster, 
  only requiring abojut 630 nanoseconds. To use this second method you'll also need
  the statements  
      uint8_t myPin_mask = digitalPinToBitMask(GPS_PPS_PIN);
      volatile uint8_t *myPin_port = portInputRegister(digitalPinToPort(GPS_PPS_PIN));
  in your setup function.
  
  George Gollin
  University of Illinois 
  October 1, 2020

*************************************************************************************/

#include <Wire.h>

// declare which Arduino pin sees the GPS PPS signal
int GPS_PPS_PIN = 43;

/////////////////////////////////////////////////////////////////////////

void setup () {

  // fire up the serial monitor
  Serial.begin(115200);
  while(!Serial){};
  
  Serial.println("\n\n*********************************************************\n");
  Serial.print("Let's study how fast we can interrogate the GPS PPS pin, ");
  Serial.println("tied to Arduino D43.\n");

  Serial.println("                 ********************************");
  Serial.println("                 **   The GPS needs to have    **");
  Serial.println("                 ** acquired satellites before **");
  Serial.println("                 **        this will work      **");
  Serial.println("                 ********************************");

  // declare the GPS PPS pin to be an Arduino input 
  pinMode(GPS_PPS_PIN, INPUT);

  // do this once at setup
  uint8_t myPin_mask = digitalPinToBitMask(GPS_PPS_PIN);
  volatile uint8_t *myPin_port = portInputRegister(digitalPinToPort(GPS_PPS_PIN));
  
  // do some timing studies here.
  const uint32_t loop_max = 100000;
  int seconds_so_far;
  const int seconds_to_watch = 10;
  uint32_t millis_transition[100];
  uint32_t micros_transition[100];
  uint32_t index_transition[100];

  int GPS_PPS_value;
  int GPS_PPS_value_old;

  /////////////////////////////////////////////////////////////
  
  // in this section of code study the time needed for a call to 
  // digitalRead(GPS_PPS_PIN) in comparision to using 
  // (*myPin_port & myPin_mask); to read the Arduini pin.

  /////////////////////////////////////////////////////////////
  
  uint32_t tstart;
  uint32_t tstop;
  uint32_t t_elapsed_1;
  uint32_t t_elapsed_2;
  uint32_t t_elapsed_3;

  uint32_t idummy = 0;
  uint32_t jdummy = 0;
  uint32_t kdummy = 0;

  float ii;
  float jj;
  float kk;

  Serial.print("\nNow execute a dummy loop "); 
  Serial.print(loop_max);
  Serial.println(" times.");

  // record starting time (microseconds)
  tstart = micros();

  for (uint32_t loop_index = 0; loop_index < loop_max; loop_index++)
  {
    idummy++;
    jdummy++;
    kdummy++;

    ii = sqrt(idummy);
    jj = sqrt(jdummy);
    kk = sqrt(kdummy);

    // GPS_PPS_value = digitalRead(GPS_PPS_PIN);
    // GPS_PPS_value = (*myPin_port & myPin_mask);
  }

  // record stopping time (microseconds)
  tstop = micros();
  t_elapsed_1 = tstop - tstart; 

  Serial.print("Print some nonsense to force the compiler to include the loop. ");
  Serial.print(ii);
  Serial.print(jj);
  Serial.println(kk);

  Serial.print("Dummy loop total execution time = ");
  Serial.print(t_elapsed_1);  
  Serial.print(" microseconds (= "); Serial.print(float(t_elapsed_1) / 1000000.);
  Serial.println(" seconds)");

  /////////////////////////////////////////////////////////////
  
  // now put in a call to the pin-determining code.

  Serial.println("\nNow execute a similar loop, but including a digitalRead(GPS_PPS_PIN)."); 

  idummy = 0;
  jdummy = 0;
  kdummy = 0;

  // record starting time (microseconds)
  tstart = micros();

  for (uint32_t loop_index = 0; loop_index < loop_max; loop_index++)
  {
    idummy++;
    jdummy++;
    kdummy++;

    ii = sqrt(idummy);
    jj = sqrt(jdummy);
    kk = sqrt(kdummy);

    GPS_PPS_value = digitalRead(GPS_PPS_PIN);
    // GPS_PPS_value = (*myPin_port & myPin_mask);
    }

  // record stopping time (microseconds)
  tstop = micros();

  t_elapsed_2 = tstop - tstart; 

  Serial.print("Print some nonsense to force the compiler to include the loop. ");
  Serial.print(ii);
  Serial.print(jj);
  Serial.println(kk);

  Serial.print("Total loop execution time = ");
  Serial.print(t_elapsed_2);  
  Serial.print("  microseconds (= "); Serial.print(float(t_elapsed_2) / 1000000.);
  Serial.println(" seconds)");

  float dt2 = (t_elapsed_2 - t_elapsed_1) / float(loop_max);

  Serial.print("\nTime (microseconds) per digitalRead(GPS_PPS_PIN) is ");
  Serial.println(dt2);

  /////////////////////////////////////////////////////////////
  
  // now put in a call to the other pin-determining code.

  Serial.println("\nNow execute a loop using (*myPin_port & myPin_mask) instead."); 

  idummy = 0;
  jdummy = 0;
  kdummy = 0;

  // record starting time (microseconds)
  tstart = micros();

  for (uint32_t loop_index = 0; loop_index < loop_max; loop_index++)
  {
    idummy++;
    jdummy++;
    kdummy++;

    ii = sqrt(idummy);
    jj = sqrt(jdummy);
    kk = sqrt(kdummy);

    // GPS_PPS_value = digitalRead(GPS_PPS_PIN);
    GPS_PPS_value = (*myPin_port & myPin_mask);
    }

  // record stopping time (microseconds)
  tstop = micros();

  t_elapsed_3 = tstop - tstart; 

  Serial.print("Print some nonsense to force the compiler to include the loop. ");
  Serial.print(ii);
  Serial.print(jj);
  Serial.println(kk);

  Serial.print("Loop execution time: ");
  Serial.print(t_elapsed_3);  
  Serial.print(" microseconds (= "); Serial.print(float(t_elapsed_3) / 1000000.);
  Serial.println(" seconds).");

  float dt3 = (t_elapsed_3 - t_elapsed_1) / float(loop_max);

  Serial.print("\nTime (microseconds) per (*myPin_port & myPin_mask) is ");
  Serial.println(dt3);

  /////////////////////////////////////////////////////////////
  
  // in this section of code study the change in limmis and
  // micros between GPS PPS pulses. Do it using digitalRead(GPS_PPS_PIN) 
  // and also (*myPin_port & myPin_mask).

  /////////////////////////////////////////////////////////////

  Serial.println("\nNow look at change in millis and micros values between PPS pulses.");
  Serial.println("\nFirst interrogate PPS pin using this: GPS_PPS_value = digitalRead(GPS_PPS_PIN);");
  Serial.println("We.ll do this for ten seconds.");

  // initialize a flag holding the GPS PPS pin status: this pin pulses positive as soon as 
  // the seconds value rolls to the next second.
  GPS_PPS_value_old = 0;
  seconds_so_far = 0;
  uint32_t loop_count = 0;

  while(seconds_so_far < seconds_to_watch)
  {
    loop_count++;
    GPS_PPS_value = digitalRead(GPS_PPS_PIN);
    // GPS_PPS_value = (*myPin_port & myPin_mask);

    // did we just get a 0 -> 1 transition in PPS? If so,
    // digitalRead(GPS_PPS_PIN) will return 1, but 
    // (*myPin_port & myPin_mask) will return 64.
    /// if no transition, then both are zero.

    if (GPS_PPS_value >= 1 && GPS_PPS_value_old == 0) 
    {    
      // record (and store) millis and mocros values.
      millis_transition[seconds_so_far] =  millis();
      micros_transition[seconds_so_far] =  micros();
      index_transition[seconds_so_far] = loop_count;
      seconds_so_far++;
    }

    // do our bookkeeping here.
    GPS_PPS_value_old = GPS_PPS_value;
  } 

  // now print out what we found.

  for (int ijk = 0; ijk < seconds_so_far; ijk++)
  {
    Serial.print("\nPPS 0 -> 1 transition at millis, micros = ");
    Serial.print(millis_transition[ijk]);
    Serial.print("  ");
    Serial.println(micros_transition[ijk]);

    if(ijk > 0)
    {
      Serial.print("change in millis and micros between this and previous PPS pulse: ");        
      Serial.print(millis_transition[ijk] - millis_transition[ijk - 1]);        
      Serial.print("  ");
      Serial.println(micros_transition[ijk] - micros_transition[ijk - 1]);
      Serial.print("'while loop' passes in one second = ");
      Serial.println(index_transition[ijk] - index_transition[ijk - 1]);
    }
  }

  Serial.println("\nNow interrogate PPS pin this way: GPS_PPS_value = (*myPin_port & myPin_mask);");

  GPS_PPS_value_old = 0;
  seconds_so_far = 0;
  loop_count = 0;

  while(seconds_so_far < seconds_to_watch)
  {
    loop_count++;
    // GPS_PPS_value = digitalRead(GPS_PPS_PIN);
    GPS_PPS_value = (*myPin_port & myPin_mask);

    // did we just get a 0 -> 1 transition in PPS? If so,
    // digitalRead(GPS_PPS_PIN) will return 1, but 
    // (*myPin_port & myPin_mask) will return 64.
    /// if no transition, then both are zero.

    if (GPS_PPS_value >= 1 && GPS_PPS_value_old == 0) 
    {    
      // record (and store) millis and mocros values.
      millis_transition[seconds_so_far] =  millis();
      micros_transition[seconds_so_far] =  micros();
      index_transition[seconds_so_far] = loop_count;
      seconds_so_far++;
    }

    // do our bookkeeping here.
    GPS_PPS_value_old = GPS_PPS_value;
  } 

  // now print out what we found.

  for (int ijk = 0; ijk < seconds_so_far; ijk++)
  {
    Serial.print("\nPPS 0 -> 1 transition at millis, micros = ");
    Serial.print(millis_transition[ijk]);
    Serial.print("  ");
    Serial.println(micros_transition[ijk]);
 
    if(ijk > 0)
    {
      Serial.print("change in millis and micros between this and previous PPS pulse: ");        
      Serial.print(millis_transition[ijk] - millis_transition[ijk - 1]);        
      Serial.print("  ");
      Serial.println(micros_transition[ijk] - micros_transition[ijk - 1]);
      Serial.print("'while loop' passes in one second = ");
      Serial.println(index_transition[ijk] - index_transition[ijk - 1]);

    }
  }
  
  Serial.println("\nWe are finished!");
}

/////////////////////////////////////////////////////////////////////////

void loop () 
{

// nothing here!

}    

