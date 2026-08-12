#include "courier/odom.hpp"

namespace odom{

float prevAngle = 0;
float prevHoriz = 0;
float prevVert = 0;

float thetaDelta = 0;
float vertDelta = 0;
float horizDelta = 0;

float theta;
float vert;
float horiz;

float xPos = 0;
float yPos = 0;
float sV;
float sS;

float MCLParticles = 144;

std::vector<float> DeltaCoords;

void updateDeltas(){
  thetaDelta = theta - prevAngle;
  vertDelta = vert - prevVert;
  horizDelta = horiz - prevHoriz;
}

void updatePrev(){
  prevAngle = theta;
  prevVert = vert;
  prevHoriz = horiz;
}

void odomCalc(){
  //  thetaDelta Will need to be in radians
  float deltaX;
  float deltaY;
  float rot;
  

  //float angleDelta = (leftDelta - rightDelta) / (sL + sR); // if we lacked a IMU we could use this

  float angleDelta = thetaDelta;

  if (std::abs(angleDelta) > 0.0174533){

    DeltaCoords = {2*std::sin(angleDelta/2)*((horizDelta/angleDelta) + sS), 2*std::sin(angleDelta/2)*((vertDelta/angleDelta) + sV)};

  }else{
    DeltaCoords = {horizDelta, vertDelta}; 
  }
  
  rot = prevAngle + (angleDelta/2);
  //rot = rot*(180/M_PI);
  deltaX = DeltaCoords[0]*std::cos(rot) - DeltaCoords[1]*std::sin(rot); 
  deltaY = DeltaCoords[0]*std::sin(rot) + DeltaCoords[1]*std::cos(rot);

  xPos = xPos + deltaX; 
  yPos = yPos + deltaY;
}

void mclInit(){
  for (int i = 0; i < MCLParticles; i++) {
    float x = rand() % 144;
    float y = rand() % 144;
    particles.push_back(particle(x, y, 1.0 / MCLParticles));
  }
}


void mclStep(){
  // move each particle by odom deltas along with noise and set heading to theta 
  
  // simulate readings for each particle and then compare them to actual readings

  // weight each particle (1/sigma*sqrt(2*pi)*e^-((reading - actual)^2/(2*sigma^2)))

  // Resample particles based on weight using Stochastic Universal Sampling
}



void odomDrive(void* param){
initParams* params = static_cast<initParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;

pros::IMU* imu =  (params->imu);
pros::MotorGroup* vert_m = (params->driveMotor);


int go = 1;
while (go==1){
  horiz = 0;
  vert = vert_m->get_position() * (diameter * M_PI) * driveRatio / 360;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();
  if (params->mcl == true){

  }

  pros::delay(10);
}
}

void odomY(void* param){
odomParams* params = static_cast<odomParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float diameter = params->YwheelDiameter;

pros::IMU* imu = (params->imu);
pros::Rotation* vert_r = (params->vert);


int go = 1;
while (go==1){
  horiz = 0; 
  vert = (vert_r->get_position() * (diameter * M_PI)/ 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}

void odomX(void* param){
initParams* params = static_cast<initParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float Xdiameter = params->XwheelDiameter;
float Ydiameter = params->YwheelDiameter;
float driveRatio = params->DriveRatio;


pros::MotorGroup* vert_m = (params->driveMotor);
pros::IMU* imu = (params->imu);
pros::Rotation* horiz_m = (params->horiz);


int go = 1;
while (go==1){
  horiz = (horiz_m->get_position() * (Xdiameter * M_PI)/ 360)/100; //This is temporary becuase I dont feel like seting up a traking wheel, and dont have a rotation sensor
  vert = (vert_m->get_position() * (Ydiameter * M_PI) * driveRatio / 360);// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}

void odomXY(void* param){
odomParams* params = static_cast<odomParams*>(param);
// sV_in, float sS_in, int imu_port, int tracking_port

sS = params->sS_in;
sV = params->sV_in;
float YwheelDiameter = params->YwheelDiameter;
float XwheelDiameter = params->XwheelDiameter;

pros::IMU* imu = (params->imu);
pros::Rotation* horiz_m = (params->horiz);
pros::Rotation* vert_m = (params->vert);


int go = 1;
while (go==1){
  horiz = (horiz_m->get_position() * (XwheelDiameter * M_PI) / 360) / 100;
  vert = (vert_m->get_position() * (YwheelDiameter * M_PI) / 360) / 100;// I used 4 inch wheels, so the 4 would be changed to what every size wheels And the 3/5 is the gear ratio
  theta = imu->get_heading() * (M_PI/180); // gets the inertial and converts to radians
  updateDeltas();
  odomCalc();
  updatePrev();

  pros::delay(10);
}
}


//functions to initiallize the odom system

bool init_odom(enum odom::config con, initParams params){
  initParams* heap_params = new initParams(params);

  if (con == odom::DRIVE){
    //setup for drive only
    pros::Task odo(odomDrive, heap_params);
  }else if (con == odom::XTRACK){
    //setup for xtrack only
    pros::Task odo(odomX, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}

bool init_odom(enum odom::config con, odomParams params){
  odomParams* heap_params = new odomParams(params);

  if (con == odom::YTRACK){
    //setup for ytrack only
    pros::Task odo(odomY, heap_params);
  }else if (con == odom::XYTRACK){
    //setup for xytrack
    pros::Task odo(odomXY, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}


bool init_odom(enum odom::config con, chassis chass, TaskParams params){
  TaskParams* heap_params = new TaskParams(params);
  chassis* heap_chass = new chassis(chass);

  if (con == odom::DRIVE){
    //setup for drive only
    initParams* init_params = new initParams(heap_chass->imu, heap_chass->leftMotors, heap_chass->horiz);
    pros::Task odo(odomDrive, heap_params);
  }else if (con == odom::XTRACK){
    //setup for xtrack only
    pros::Task odo(odomX, heap_params);
  }else if (con == odom::YTRACK){
    //setup for ytrack only
    pros::Task odo(odomY, heap_params);
  }else if (con == odom::XYTRACK){
    //setup for xytrack
    pros::Task odo(odomXY, heap_params);
  }else{
    delete heap_params;
    return false;
  }
  return true;

}

std::vector<double> getPos(){
  return {-xPos, yPos, theta * (180/M_PI)};
}

Point getPoint(){
  Point out(-xPos, yPos);
  return out;
}

float getAng(){
  return theta * (180/M_PI);
}

std::vector<double> getVals(){
  return {vert, horiz, theta * (180/M_PI)};  
}

void setOdom(float x, float y){
  xPos = -x;
  yPos = y;
}


} //Namespace odom
