//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST        300
#define ZUMO_SLOW        120
#define ZUMO_Turn        300
#define X_CENTER         (pixy.frameWidth/2)

Pixy2 pixy;
ZumoMotors motors;
ZumoBuzzer buzzer;

PIDLoop headingLoop(3500, 0, 2000 , false);

void setup() 
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  pixy.init();
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  // look straight and down
  pixy.setServos(500, 950);
}


void loop()
{
  int8_t res;
  int32_t error; 
  int left, right;
  char buf[96];

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();

  // If error or nothing detected, stop motors
  //**Stop code??**
  
  // We found the vector...
  if (res&LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    // Perform PID calcs on heading error.
    headingLoop.update(error);
    // separate heading into left and right wheel velocities.
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT)
      {
        
        if (abs(pixy.line.vectors->m_x1 - pixy.line.vectors->m_x0)<=10){
          left +=ZUMO_FAST;
          right +=ZUMO_FAST;
        }
        else{
          left +=ZUMO_FAST;
          right -=ZUMO_FAST;
        }
        
        
      }
      else // otherwise, pedal to the metal!
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }    
    }
    
    else  // If the vector is pointing down, or down-ish, we need to go backwards to follow.
    {
    
      int8_t reverseVector();
      if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT)
      {
        left +=ZUMO_FAST;
        right -=ZUMO_FAST; 
      }
      
      else // otherwise, pedal to the metal!
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }

      
      /*left+= ZUMO_FAST;
      right+= ZUMO_FAST;*/
    } 
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
  }

  // If intersection, do nothing (we've already set the turn), but acknowledge with a beep.
  if (res&LINE_INTERSECTION)
  {
    if (pixy.line.intersections->m_n==4){
      delay(200);
      left = -ZUMO_Turn;
      right = ZUMO_Turn;
      motors.setLeftSpeed(left);
      motors.setRightSpeed(right);
      delay(800);
      Serial.print("yes");
     /*
      motors.setLeftSpeed(300);
      motors.setRightSpeed(-300);
      delay(1000);
      */
    }
  }
  if (res<=0){
      if (left>right){
        motors.setLeftSpeed(ZUMO_Turn);
        motors.setRightSpeed(-ZUMO_Turn);
      }
      else if (right>left){
        motors.setLeftSpeed(-ZUMO_Turn);
        motors.setRightSpeed(ZUMO_Turn);
       }
      else{
        motors.setLeftSpeed(0);
        motors.setRightSpeed(0);
       }
    buzzer.playFrequency(500, 50, 15);
    Serial.print("stop ");
    Serial.println(res);
    return;
  }
}
