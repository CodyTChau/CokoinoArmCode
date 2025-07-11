/*
 * This code applies to cokoino mechanical arm
 * Through this link you can download the source code:
 * https://github.com/Cokoino/CKK0006
 * Company web site:
 * http://cokoino.com/
 *                                     ________
 *                         ----|servo4| 
 *                        |            --------
 *                    |servo3|   
 *                        |
 *                        |
 *                    |servo2|
 *                        |
 *                        |
 *                  ___________
 *                  |  servo1 |
 *         ____________________
 *         ____________________
 * Fanctions:
 * arm.servo1.read();   //read the servo of angle
 * arm.servo2.read();
 * arm.servo3.read();
 * arm.servo4.read();
 * 
 * arm.servo1.write(angle);   //servo run
 * arm.servo2.write(angle);
 * arm.servo3.write(angle);
 * arm.servo4.write(angle);
 * 
 * arm.left(speed);    //perform the action 
 * arm.right(speed);
 * arm.up(speed);
 * arm.down(speed);
 * arm.open(speed);
 * arm.close(speed);
 * 
 * arm.captureAction();    //capture the current action,return pointer array
 * arm.do_action(int *p,int speed);  //P is a pointer to the array
 * 
 * arm.JoyStickL.read_x(); //Returns joystick numerical
 * arm.JoyStickL.read_y();
 * arm.JoyStickR.read_x();
 * arm.JoyStickR.read_y();
 */
#include "src/CokoinoArm.h"
#define buzzerPin 9

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX, TX

CokoinoArm arm;
int xL,yL,xR,yR;

int currentAngle2 = 90; // Start at a neutral mid-point (adjust as needed)

int currentAngle3 = 90; // Neutral position for servo3

int currentAngle4 = 90;  // Start at neutral for claw

int xL_center = 512, yL_center = 512;
int xR_center = 512, yR_center = 512;
int deadZone = 30; // you can increase this to 40+ if needed

const int act_max=170;    //Default 10 action,4 the Angle of servo
int act[act_max][4];    //Only can change the number of action
int num=0,num_do=0;
///////////////////////////////////////////////////////////////
void turnUD(void) {
  int deviation = xL - xL_center;

  if (abs(deviation) > deadZone) {
    int targetDelta = 0;

    if (deviation < 0) {
      targetDelta = map(abs(deviation), deadZone, xL_center, 1, 5);  // Arm up
    } else {
      targetDelta = -map(abs(deviation), deadZone, 1023 - xL_center, 1, 5);  // Arm down
    }

    currentAngle2 = constrain(currentAngle2 + targetDelta, 0, 180);
    arm.servo2.write(currentAngle2);
    delay(15);  // Smooth transition
  }
}

void controlServo3WithRightStick() {
  if (xR < 500 || xR > 524) {  // Dead zone
    int delta = 0;

    if (xR < 512) {
      delta = -map(xR, 0, 512, 5, 1);  // Move down
    } else {
      delta = map(xR, 512, 1023, 1, 5);  // Move up
    }

    currentAngle3 = constrain(currentAngle3 + delta, 0, 180);
    arm.servo3.write(currentAngle3);
    delay(15);  // Smooth movement
  }
}
///////////////////////////////////////////////////////////////
int currentAngle1 = 90;  // Add this near your other currentAngle variables

void turnLR(void) {
  int deviation = yL - yL_center;

  if (abs(deviation) > deadZone) {
    int delta = 0;

    if (deviation < 0) {
      delta = -map(abs(deviation), deadZone, yL_center, 1, 5);
    } else {
      delta = map(abs(deviation), deadZone, 1023 - yL_center, 1, 5);
    }

    currentAngle1 = constrain(currentAngle1 + delta, 0, 180);
    arm.servo1.write(currentAngle1);
    delay(15);  // Smooth transition
  }
}
//////////////////////////////////////////////////////////
void controlClawWithRightY() {
  if (yR < 500 || yR > 524) {  // Dead zone
    int delta = 0;

    if (yR < 512) {
      delta = -map(yR, 0, 512, 5, 1);  // Close claw
    } else {
      delta = map(yR, 512, 1023, 1, 5);  // Open claw
    }

    currentAngle4 = constrain(currentAngle4 + delta, 0, 180);
    arm.servo4.write(currentAngle4);
    delay(15);  // Smooth transition
  }
}
///////////////////////////////////////////////////////////////
// void date_processing(int *x,int *y){
//   if(abs(512-*x)>abs(512-*y))
//     {*y = 512;}
//   else
//     {*x = 512;}
// }
///////////////////////////////////////////////////////////////
// void C_action(void){
//   if(yR>800){
//     int *p;
//     p=arm.captureAction();
//     for(char i=0;i<4;i++){
//       act[num][i]=*p;
//       p=p+1;     
//     }
//     num++;
//     num_do=num;
//     if(num>=act_max){
//       num=0;
//       buzzer(600,400);
//     }
//     while(yR>600){yR = arm.JoyStickR.read_y();}
//   }
// }
///////////////////////////////////////////////////////////////
// void Do_action(void){
//   if(yR<220){
//     buzzer(200,300);
//     for(int i=0;i<num_do;i++){
//       arm.do_action(act[i],15);
//     }
//     num=0;
//     while(yR<420){yR = arm.JoyStickR.read_y();}
//     for(int i=0;i<2000;i++){
//       digitalWrite(buzzerPin,HIGH);
//       delayMicroseconds(200);
//       digitalWrite(buzzerPin,LOW);
//       delayMicroseconds(300);        
//     }
//   }
// }
///////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);         // for debugging
  BTSerial.begin(9600);       // for HC-05
  arm.ServoAttach(4,5,6,7);
  arm.JoyStickAttach(A0,A1,A2,A3);
  delay(1000);

  xL_center = arm.JoyStickL.read_x();
  yL_center = arm.JoyStickL.read_y();
  xR_center = arm.JoyStickR.read_x();
  yR_center = arm.JoyStickR.read_y();
}
///////////////////////////////////////////////////////////////
void loop() {
  xL = arm.JoyStickL.read_x();
  yL = arm.JoyStickL.read_y();
  xR = arm.JoyStickR.read_x();
  yR = arm.JoyStickR.read_y();

  xL = constrain(xL, 0, 1023);
  yL = constrain(yL, 0, 1023);
  xR = constrain(xR, 0, 1023);
  yR = constrain(yR, 0, 1023);

  Serial.print("xL: "); Serial.print(xL);
  Serial.print(" yL: "); Serial.print(yL);
  Serial.print(" xR: "); Serial.print(xR);
  Serial.print(" yR: "); Serial.println(yR);

  // Only run turnUD if xL has moved enough
  if (abs(xL - xL_center) > deadZone) {
    turnUD();
  }

  // Only run turnLR if yL has moved enough
  if (abs(yL - yL_center) > deadZone) {
    turnLR();
  }

  // Only run servo3 control if xR has moved enough
  if (abs(xR - xR_center) > deadZone) {
    controlServo3WithRightStick();
  }

  // Only run claw if yR has moved enough
  if (abs(yR - yR_center) > deadZone) {
    controlClawWithRightY();
  }

    if (BTSerial.available()) {
    char cmd = BTSerial.read();
    handleBluetoothCommand(cmd);
  }
}
void handleBluetoothCommand(char cmd) {
  switch (cmd) {
    case 'L': arm.left(10); break;
    case 'R': arm.right(10); break;
    case 'U': arm.up(10); break;
    case 'D': arm.down(10); break;
    case 'O': arm.open(10); break;
    case 'C': arm.close(10); break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      break;
  }
}