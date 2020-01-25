#ifndef PID_modified_h
#define PID_modified_h
#define LIBRARY_VERSION	1.2.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID();
    
    PID(double*, double*, double*,                        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, unsigned long, int, int); //   Setpoint.  Initial tuning parameters are also set here.
                                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,                        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, unsigned long, int);      //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)
                                          //   In the original library, this affects the computation of
                                          //   the new output, but in this library it has no effect on
                                          //   Compute() and only affects other functions.

    void Compute(unsigned long curTime);  // * performs the PID calculation.  It should be
                                          //   called every time you want to calculate the new output.

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
                      
  void setLastTime(unsigned long newTime);  // Sets the variable lastTime to newTime
                                            // Used when starting the PID after it was paused
                                            // If this is not done, the difference between curTime and
                                            // lastTime in the Compute function will become huge if the
                                            // PID is paused for a long time
                                            
  void setLastInput(double newLastInput);   // Sets the variable lastInput to newLastInput
                                            // Used when restarting the PID, but from a more current
                                            // "last input" than one that could have been obtained
                                            // an extremely long time ago.
                                            
  // Directly modify the outputSum, which is the bulk of the integral term. Useful if we want to start
  // the control from a certain state and slowly adjust from there.
  void setOutputSum(double newOutputSum);
                      
  void Reset(); // Resets the "memorized" variables e.g. outputSum, lastInput, etc. Used when changing
                // setpoints to avoid delays due to resistance from the "memorized" variables
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				//   format for display purposes
	double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
  double ki;                  // * (I)ntegral Tuning Parameter
  double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
  unsigned long timeChange;
	double dInput, error, lastInput, outputSum, tempOutput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
};
#endif

