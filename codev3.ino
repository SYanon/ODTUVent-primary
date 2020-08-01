//#include stepper
//#include <trajfactory.h>
#include "Arduino.h"
#define trajectory_h

//We will define a class called trajectory that will hold all the variables and functions we 
//need for a specific trajectory
//I did not write the #ifndef and #endif because this is the main code. 
/////////==============================================================///////////////
typedef struct  {

  // Current slave mode.
  char mode;

  // Ventilation settings.
  int tidal_volume;
  int respiration_rate;
  int inhale;
  int exhale;
  int hold_seconds;
  int hold_decimals;

  // Time between trajectory points.
  int delta_time;

  // Current ventilation time.
  int hours;
  int minute;
  int seconds;

  // Boolean to indicate to send settings.
  bool send;

} VentSettings;

typedef struct {

  // Limits for ventilation settings.
  int min_tidal_volume = 300;
  int max_tidal_volume = 700;
  int delta_tidal_volume = 50;

  int min_respiration_rate = 10;
  int max_respiration_rate = 30;
  int delta_respiration_rate = 1;

  int min_exhale = 1;
  int max_exhale = 5;
  int delta_exhale = 1;

} VentLimits;



class Trajectory {
  public: 
    Trajectory (int delta_t, int* positions , int length); //This is a function in the class 
//Methods: 
    int nextStep();
//Find a value: 
    int getLength();
    int getCurrentStep();
    int getDeltaTime();

private: 

 // Current step in the trajectory.
    int _cur_step;

    // Length of the trajectory.
    int _length;

    // Pointer to array of positions.
    int* _traj_pos;

    // Delta time between positions.
    int _delta_t;

  
};

//Now we will define the methods inside the class above, one by one.

//The first is Trajectory function used to record values of the movement. 
Trajectory::Trajectory(int delta_t, int* positions, int length): 
  _delta_t(delta_t),
  _cur_step(0),
  _length(length){

    _traj_pos = positions; 
  }
//Next step function
 int Trajectory::nextStep(){
  Serial.println("current step:");
  Serial.println(_cur_step);
  Serial.println("*(_traj_pos");
  Serial.println(*(_traj_pos));
  int next = *(_traj_pos + _cur_step); // Finding the next position based on current position
  Serial.println("next is now");
  Serial.println(next);
  
  _cur_step++; //Move to the next step (this is a counter) 
  
  if(_cur_step >= _length) { //Check if the counter has reached the end position, or the end count. 
    _cur_step = 0; 
  }
  return next;
 }
//Get the other variables 
int Trajectory::getLength() {
  return _length;
}

int Trajectory::getCurrentStep() {
  return _cur_step;
}

int Trajectory::getDeltaTime() {
  return _delta_t;
}

//Having stored all the necessary trajectory values its time to build the trajectory itself. 
//Defining a class that contains all the breathing functions and parameters: 
class TrajFactory {
  public: 
    TrajFactory(int respitory_rate, float i_e_ratio, float hold_time, int delta_t);
    TrajFactory(); //This is same as above but will have default values if nothing was entered

    Trajectory* build(int respitory_rate, float i_e_ratio, int setpoint, float hold_time, int delta_t);
    Trajectory* buildSetpoint(int setpoint);

    //Getters again: 
    float getTotalTime();
    float getInhaleTime();
    float getExhaleTime();
    int getDeltaTime();
    int getLength();

  private: 
    int _length; //length of the trajectory to generate, this will be sent back 

    int* _traj_pos = 0; //array of pointers, initially to zero. You'll understand how this collaborates with the
    // above class when we start writing the functions of this class, 

    // Ventilation settings.
    int _respitory_rate;
    float _i_e_ratio;

    // Variables to hold positions of trajectory.
    int _x_s;
    int _x_0;

    // Variables to hold timing keypoints in trajectory
    float _t_in;
    float _t_out;
    float _t_hold;
    float _t_tot;
    int _delta_t;
};

  
//Defining the functions assosiated with the class above: 
TrajFactory::TrajFactory( int respitory_rate, float i_e_ratio, float hold_time, int delta_t):

  _respitory_rate(respitory_rate),
  _i_e_ratio(i_e_ratio),
  _t_hold(hold_time),
  _delta_t(delta_t){}

  TrajFactory::TrajFactory(){}

//Lets define the building function. 
Trajectory* TrajFactory::buildSetpoint(int setpoint){

  _t_tot = 60/_respitory_rate; 
  Serial.println("TOTAL TIME IS: ");
  Serial.println(_t_tot);
  
  //calculate breath cycle times: 
  _t_out = (_t_tot - _t_hold)/(_i_e_ratio + 1);
  _t_in = _i_e_ratio * _t_out;

  //Kinematics of the breathing curve are here: 

  float x_h = (_x_s + _x_0)/2.0;
  float v_in = 2*((_x_s - _x_0)/_t_in);
  float a_in = (v_in * v_in)/(_x_s-_x_0);
  float v_out = 2*((_x_s - _x_0)/_t_out);
  float a_out = (v_out * v_out)/(_x_s-_x_0);

  // Calculate the length of the new trajectory.
  _length = int (_t_tot * (1000.0 / _delta_t));
  Serial.println("Total length is:");
  Serial.println(_length);

// If there's a trajectory saved, delete it.
  if (_traj_pos != 0) {
    delete [] _traj_pos; 
  }

//Create a new array of trajectory
 _traj_pos = new int[_length];
 // Calculate the timing of keypoints in the trajectory.
  int in_concave = int (1000*(_t_in/2.0));
  int in_convex = int (1000*_t_in);
  int hold_end = int (1000*(_t_in + _t_hold));
  int out_convex  = int (1000*(_t_in + _t_hold + _t_out/2.0));

//Generate Trajectory, the trajectory loop. This is the game now. We set up all the parametrs
//here is where we useem all. 

for (int i = 0; i<_length; i++){

  int t = _delta_t*i; 
  Serial.println("Time is:");
  Serial.println(t);
  // Calculate in stroke concave portion.
    if (t < in_concave) {
      *(_traj_pos + i) = _x_0 + a_in*(t/1000.0)*(t/1000.0) / 2;

    // Calculate in stroke convex portion.
    } else if (t >= in_concave && t < in_convex) {
      float cur_t = (t - in_concave)/1000.0;
      *(_traj_pos + i) = x_h + v_in*cur_t - a_in*cur_t*cur_t/2;

    // Calculate hold position.
    } else if (t >= in_convex && t < hold_end) {
        *(_traj_pos + i) = _x_s;

    // Calculate out stroke convex portion.
    } else if (t >= hold_end && t < out_convex) {
      float cur_t = (t - hold_end)/1000.0;
      *(_traj_pos + i) = _x_s - a_out*cur_t*cur_t / 2;
    
    // Calculate out stroke concave portion.
    } else {
      float cur_t = (t - out_convex)/1000.0;
      *(_traj_pos + i) = x_h - v_out*cur_t + (a_out*cur_t*cur_t)/2;
      //Serial.println(*(_traj_pos +i));
    }
    Serial.println("*(_traj_pos)");
    Serial.println(*(_traj_pos));
    Serial.println("_traj_pos + i ");
    Serial.println(*(_traj_pos + i));
    
    
  }

  // Create and return trajectory.
  return new Trajectory(_delta_t, _traj_pos, _length);
}
Trajectory* TrajFactory::build(int respitory_rate, float i_e_ratio, int setpoint, float hold_time, int delta_t){

  // Update internal parameters.
  _respitory_rate = respitory_rate;
  _i_e_ratio = i_e_ratio;
  _x_s = setpoint;
  _t_hold = hold_time;
  _delta_t = delta_t;
  Serial.println("respitory_rate and setpoint parameters are");
  Serial.println(respitory_rate);
  Serial.println(setpoint);
  Serial.println(_x_s);

  return buildSetpoint(setpoint);
}

// Getters.
float TrajFactory::getTotalTime(){
  return _t_tot;
}

float TrajFactory::getInhaleTime(){
  return _t_in;
}

float TrajFactory::getExhaleTime(){
  return _t_out;
}

int TrajFactory::getDeltaTime(){
  return _length;
}

int TrajFactory::getLength(){
  return _length;
}




//======================================================//
//The Arduino loop finally
//====================================================//
//Initializing the variables: 
TrajFactory tf = TrajFactory(); //Notice that there are no inputs to the function here. 
Trajectory* traj_ptr = 0; 

//Vent Parameters:


// Default settings.
VentSettings vs = {'L', 500, 12, 1, 3, 0, 00, 20, 0, 0, 0, false}; 
// Default limits.
VentLimits vl;
int cal[9] = {950, 1062, 1155, 1238, 1320, 1393, 1465, 1538, 1610};

int rr = vs.respiration_rate;
float ie = vs.inhale/vs.exhale;
int setpoint; 
float hold= vs.hold_seconds;
int delta_t=vs.delta_time;
char state=vs.mode;


void setup() {
//============================================//
//sensor setup


//================================================
    // Start serial output for trajectory monitoring.
  Serial.begin(9600);



  // Set state to loading.
  state = 'L';

}

void loop() {

  if (vs.mode == 'L') {
    // Calculate setpoint based on calibration.
     setpoint = cal[(vs.tidal_volume - vl.min_tidal_volume)/vl.delta_tidal_volume];
    //Serial.println("SetPoint:");
    //Serial.println(setpoint);
  }
//Depending on the state, runt the device: 

switch (state) {

case '0':
  moveTo(traj_ptr->nextStep(), traj_ptr->getDeltaTime());
  break;

case 'X':
    stop();
    break;
    
case 'L': 
    stop();
    // If assigned ptr, delete contents
      if (traj_ptr != 0) {
        delete traj_ptr;
      }
      traj_ptr = tf.build(rr, ie, setpoint, hold, delta_t);
      state = '0';
      break;
  
}
  

}

void moveTo( int pos, int delta_t){

  // Write current position instruction to console.
  //Serial.println("pos_InMoveTo: ");
  //Serial.println(pos);

 //MOVE STEPPERS HERE TO POS=> MOVE THE STEPPER TO THE NEXT SETPOINT AND WAIT DELTA T FOR IT TO REACH. 
  
  //Serial.println("delta_t_InMoveto");
  //Serial.println(delta_t);
  delay(delta_t);
  
}

void stop(){
Serial.println("Stopping");
// If we have already generated a trajectory, follow it till the end.
  if (traj_ptr != 0) {
    while (traj_ptr->getCurrentStep() != 0) {
      moveTo(traj_ptr->nextStep(), traj_ptr->getDeltaTime());
    }
  // If not, got to 0 position.
  } else {
    moveTo(0, 100);
  }
}
  
