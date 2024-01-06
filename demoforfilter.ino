
#include <ICM_20948.h>
#include <MecatroUtils.h>
#include <MatrixMath.h>

// Include the current library
//#include "MecatroUtils.h"
//#include "ICM_20984.h"

// Include the AS5600 library (for the encoders) and Sparkfun I2C Mux (for multiplexer)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5
//c'est pas une var, mais le remplacement d'un carac. On risque de faire une erreur de type.

// Define the Multiplexer pins corresponding to each encoder
#define LEFT_ENCODER_PIN 3
#define RIGHT_ENCODER_PIN 5

QWIICMUX multiplexer;
AS5600 rightEncoder, leftEncoder;
ICM_20948 mySensor;

bool isgoing = true;
float deltat = 0.005;
float T_left = 0;
float T_right = 0;
float y_e_left; //the measured angular velocity
float y_e_left_old = 0;
float y_e_right; //the measured angular velocity
float y_e_right_old = 0;
float tau = 0.08;
float x_left = 0; //the x_k to be updated very time
float x_right = 0;
float omega_left;
float omega_right;
float C_1 = 143;
float C_2 = 143;
float n = 0;
const float pi = 3.14159265359;

//float for test
// float y_old = 0;
// float y;
// float x = 0;
// float omega;
// float z;


void setup()
{
  // Setup serial communication with the PC - for debugging and logging.
  Serial.begin(1000000);
  // Start I2C communication
  Wire.begin();
  // Set I2C clock speed to 400kHz (fast mode)
  Wire.setClock(400000);
  
  // Initialize tele
  unsigned int const nVariables = 2;
  String variableNames[nVariables] = {"Tension" , "Right"};
  mecatro::initTelemetry(nVariables, variableNames);

  // Init multiplexer
  if (!multiplexer.begin())
  {
    Serial.println("Error: I2C multiplexer not found. Check wiring.");
  }
  else
  {
    bool isInit = true;
    // Set multiplexer to use port RIGHT_ENCODER_PIN to talk to right encoder.
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }
    // Set multiplexer to use port LEFT_ENCODER_PIN to talk to left encoder.
    multiplexer.setPort(LEFT_ENCODER_PIN);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      Serial.println("Error: could not connect to left encoder. Check wiring.");
      isInit = false;
    }

    if (isInit)
    {
      // Configure motor control and feedback loop call.
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }
  }
}


void loop()
{
  // Don't forget to call this, otherwise nothing will happen !
  // This function never returns, put all your code inside mecatro::controlLoop.
  mecatro::run();
}


// This function is called periodically, every CONTROL_LOOP_PERIOD ms.
// Put all your code here.
void mecatro::controlLoop()
{


  // mecatro::log(0, leftEncoder.getCumulativePosition()*AS5600_RAW_TO_DEGREES);//variables float pas de fonc 
  // mecatro::log(1, rightEncoder.getCumulativePosition()*AS5600_RAW_TO_DEGREES);

  if (isgoing) {
    multiplexer.setPort(LEFT_ENCODER_PIN);
    C_1 = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    C_2 = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    isgoing = false;
  }

  // Raw encoder measurement - from 0 to 360 degrees
  //Serial.print(rightEncoder.rawAngle() * AS5600_RAW_TO_DEGREES);

  // Software feature: the encoder itself does not measure multi-turn information nor rotation speed.
  // These features are thus implemented as software, taking two consequtive measurements and computing
  // their difference.
  // This of course assumes that the encoder has performed less than half of a turn between two calls (otherwise there
  // is no way to know how many turns were performed, or in which direction).
  // This is not a problem here: with a typical update rate of 5ms in this function, the maximum speed would be 60000rpm !
  // Serial.print("°, cumulative position ");
  // Serial.print(rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
  // Serial.print("° speed ");
  // Serial.print(rightEncoder.getAngularSpeed());
  // Serial.print("°/s ");

  // Check magnet positioning - this is for debug purposes only and is not required in normal operation.
  // if (rightEncoder.magnetTooStrong())
  // {
  //   // Serial.print(" ; warning: magnet too close.");
  // }
  // if (rightEncoder.magnetTooWeak())
  // {
  //   Serial.print(" ; warning: magnet too far.");
  // }
  // Serial.println();

  // Set multiplexer to use port LEFT_ENCODER_PIN to talk to left encoder.
  //* multiplexer.setPort(LEFT_ENCODER_PIN);
  //* float CumAngleLeft = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
  //* multiplexer.setPort(RIGHT_ENCODER_PIN);
  //* float CumAngleRight = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;


  //---------------------------------------important
// mutiplexer.setPort(LEFT_ENCODER_PIN);
  // mutiplexer.setPort(RIGHT_ENCODER_PIN);
  multiplexer.setPort(LEFT_ENCODER_PIN);
  y_e_left = (leftEncoder.getCumulativePosition()*AS5600_RAW_TO_DEGREES)-C_1;
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  y_e_right = (rightEncoder.getCumulativePosition()*AS5600_RAW_TO_DEGREES)-C_2;

  x_left = (2*pow(tau,2)*x_left-deltat*y_e_left-x_left*tau*deltat-y_e_left_old*deltat)/(2*pow(tau,2)+deltat*tau);
  x_right = (2*pow(tau,2)*x_right-deltat*y_e_right-x_right*tau*deltat-y_e_right_old*deltat)/(2*pow(tau,2)+deltat*tau);
  omega_left = x_left + y_e_left/tau;
  omega_right = x_right + y_e_right/tau;

  y_e_left_old = y_e_left;
  y_e_right_old = y_e_right;
  if (n< 300){
    T_right = 0;
  }
  else{
    T_right = 0.5;
  }

  n++;

  mecatro::log(0, 12*T_right);//variables float pas de fonc 
  mecatro::log(1, omega_right*pi/180);//on radius/s
  //---------------------------------------


  mecatro::setMotorDutyCycle(0, T_right);
  // if (n < 300) {
  //   T_right = 0.4;
  //   T_left = 0.3;
  //   mecatro::setMotorDutyCycle(T_left, T_right);
  // } else {
  //   T_right = 0.8;
  //   T_left = 0.7;
  //   mecatro::setMotorDutyCycle(T_left, T_right); 
  // }
  // n++; 


}