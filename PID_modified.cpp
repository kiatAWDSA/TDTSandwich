/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 *
 * Changes by Soon Kiat Lau
 * - Feb 2019: Added setLastInput function to manually modify the lastInput variable.
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_modified.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
 PID::PID(){}
 
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, unsigned long curTime, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = curTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, unsigned long curTime, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, curTime, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens. Give it the current time and
 *   it will calculate the time interval based on the last time it calculated output.
 **********************************************************************************/
void PID::Compute(unsigned long curTime)
{
  // NOTE: timechange is in ms!!!
  timeChange = (curTime - lastTime);
  error = *mySetpoint - *myInput;
  dInput = (*myInput - lastInput);
  outputSum+= (ki * timeChange * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if(!pOnE) outputSum-= kp * dInput;

  if(outputSum > outMax) outputSum= outMax;
  else if(outputSum < outMin) outputSum= outMin;

  /*Add Proportional on Error, if P_ON_E is specified*/
  // NOTE: output is different from outputSum. outputSum is stored in memory
  // (integral portion), while output is the actual output of the PID algorithm
  if(pOnE) tempOutput = kp * error;
  else tempOutput = 0;

  /*Compute Rest of PID Output*/
  tempOutput += outputSum - kd / timeChange * dInput;

  if(tempOutput > outMax) tempOutput = outMax;
  else if(tempOutput < outMin) tempOutput = outMin;
  *myOutput = tempOutput;

  /*Remember some variables for next time*/
  lastInput = *myInput;
  lastTime = curTime;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   kp = Kp;
   ki = Ki; 
   kd = Kd;
   
   // EDIT: We insert the contribution from time everytime we call compute so that we are
   // not restricted to a fixed cycle time (the original library uses the commented code below)
   //double SampleTimeInSec = ((double)SampleTime)/1000;
   //ki = Ki * SampleTimeInSec;
   //kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* setLastTime(unsigned long newTime)******************************************
 * Sets the variable lastTime to newTime
 * Used when starting the PID after it was paused
 * If this is not done, the difference between curTime and
 * lastTime in the Compute function will become huge if the
 * PID is paused for a long time
 ******************************************************************************/
void PID::setLastTime(unsigned long newTime)
{
  lastTime = newTime;
}

/* setLastTime(unsigned long newTime)******************************************
 * Sets the variable lastInput to newLastInput
 * Used when restarting the PID, but from a more current
 * "last input" than one that could have been obtained
 * an extremely long time ago.
 ******************************************************************************/
void PID::setLastInput(double newLastInput)
{
  lastInput = newLastInput;
}

/* setOutputSum(double newOutputSum)*******************************************
 * Sets the variable outputSum to newOutputSum
 * Useful if we want to force the control algorithm to a certain state.
 * For example, if we want the control to start at full blast and then slowly
 * reduce until we are at the target setpoint, then we could call setOutputSum
 * to set outputSum to outMax.
 ******************************************************************************/
void PID::setOutputSum(double newOutputSum)
{
  outputSum = newOutputSum;
}

/* Reset()*********************************************************************
 * Resets the "memorized" variables e.g. outputSum, lastInput, etc. Used when
 * changing setpoints to avoid delays due to resistance from the "memorized"
 * variables.
 ******************************************************************************/
void PID::Reset()
{
  outputSum = 0;
  lastInput = *myInput;
  setLastTime(millis());
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

