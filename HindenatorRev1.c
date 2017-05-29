/*HINDENATOR CODE REVISION 2
05/02/16
 */

#include "NXTServo-lib-UW.c"
//task definitions
task yaw_left();
task yaw_right();
task pitch_up();
task pitch_down();
task throttle_up();
task throttle_down();
task wall_avoid();
task ground_avoid();
task center_controls();
task get_mail();
task arm();
task land();

//global mail variable
int mail = 0;

//global status variable
bool running = true;

//globals that store control surface position
int rudderpos = 90;
int elevatorpos = 90;
int throttlepos = 0;

/*
global control constants, exist to avoid hard coding
  variables that may be changed on the fly
*/
const int maxcontrollpos = 30;
const int maxthrottlepos = 90;
const int controllchange = 10;
const int throttlechange = 30;
const int elevator_servo = 3;
const int rudder_servo = 6;

//function definitions
void deflect_elevator(int direction);
void deflect_rudder(int direction);
void increment_throttle(int direction);
void stop_control_tasks();
void resume_control_tasks();

task main() {
    nxtDisplayTextLine(0, "waiting for commands...");

    //turning off motor feedback
    nMotorPIDSpeedCtrl[motorA] = 0;
    nMotorPIDSpeedCtrl[motorB] = 0;
    nMotorPIDSpeedCtrl[motorC] = 0;

    //initializing threads
    resume_control_tasks();
    startTask(get_mail);
    startTask(arm);

    //initializing servos to 90
    SensorType[S4] = sensorI2CCustom9V;
    setServoPosition(S4, elevator_servo, 90);
    setServoPosition(S4, rudder_servo, 90);

    //keeps threads alive
    while (running) {

    }
    //sets control surfaces to a centered position
    setServoPosition(S4, elevator_servo, 90);
    setServoPosition(S4, rudder_servo, 90);
    //ends program
    stopAllTasks();
}
//mail checking tasks, look for specific values and execute commands
task yaw_left() {
    //MailBox Number:51
    while (running) {
        if (mail == 51) {
            nxtDisplayTextLine(0, "yawing left");
            deflect_rudder(1);
        }
    }
}

task yaw_right() {
    //MailBox Number:53
    while (running) {
        if (mail == 53) {
            nxtDisplayTextLine(0, "yawing right");
            deflect_rudder(-1);
        }
    }
}

task pitch_up() {
    //Mailbox Number: 55
    while (running) {
        if (mail == 55) {
            nxtDisplayTextLine(0, "pitching up");
            deflect_elevator(1);
        }
    }
}

task pitch_down() {
    //Mailbox Number:49
    while (running) {
        if (mail == 49) {
            nxtDisplayTextLine(0, "pitching down");
            deflect_elevator(-1);
        }
    }
}

task throttle_up() {
    //MailBox Number: 54
    while (running) {
        if (mail == 54) {
            nxtDisplayTextLine(0, "throttling up");
            increment_throttle(1);
        }

    }
}

task throttle_down() {
    //MailBox Number: 57
    while (running) {
        if (mail == 57) {
            nxtDisplayTextLine(0, "throttling down");
            increment_throttle(-1);
        }

    }
}

task center_controls() {
    //MailBox Number: 52
    SensorType[S4] = sensorI2CCustom9V;
    while (running) {
        if (mail == 52) {
            nxtDisplayTextLine(0, "centering controls");
            setServoPosition(S4, rudder_servo, 90);
            setServoPosition(S4, elevator_servo, 90);
        }
    }
}

task land() {
    SensorType[S4] = sensorI2CCustom9V;
    SensorType[S1] = sensorSONAR;
    while (running) {
        //MailBox Number: 48
        if (mail == 48) {
            //stops everything else
            stop_control_tasks();
            stopTask(ground_avoid);
            stopTask(wall_avoid);
            eraseDisplay();
            displayString(0, "landing");
            //center rudder
            setServoPosition(S4, rudder_servo, 90);
            //pitch down
            setServoPosition(S4, elevator_servo, 60);
            wait1Msec(3000);
            //center controls
            setServoPosition(S4, elevator_servo, 90);
            motor[motorA] = 0;
            motor[motorB] = 0;
            motor[motorC] = 0;
            //waits until it gets close to the ground
            while (SensorValue[S1] > 40) {
            }
            //pitch up to avoid ground
            setServoPosition(S4, elevator_servo, 120);
            wait1Msec(3000);
            //end program (exit condition)
            running = false;
        }
    }
}

task wall_avoid() {
    SensorType[S2] = sensorSONAR;
    SensorType[S4] = sensorI2CCustom9V;
    while (running) {
        //wait to detect a wall
        if (SensorValue[S2] < 150) {
            //center controls
            setServoPosition(S4, elevator_servo, 90);
            setServoPosition(S4, rudder_servo, 90);
            nxtDisplayTextLine(0, "wall detected");
            //disable manual control
            stop_control_tasks();
            stopTask(ground_avoid);
            //full reverse throttle
            motor[motorA] = -1 * maxthrottlepos / 2;
            motor[motorB] = -1 * maxthrottlepos / 2;
            motor[motorC] = -1 * maxthrottlepos / 2;
            //wait until the craft has backed up a sufficient amount
            while (SensorValue[S2] < 150) {
            }
            motor[motorA] = 0;
            motor[motorB] = 0;
            motor[motorC] = 0;
            nxtDisplayTextLine(0, "wall avoided");
            //resume manual control
            startTask(ground_Avoid);
            resume_control_tasks();
        }
    }
}

task ground_avoid() {
    SensorType[S1] = sensorSONAR;
    SensorType[S4] = sensorI2CCustom9V;
    while (running) {
        //waiting to detect the ground
        if (SensorValue[S1] < 50) {
            nxtDisplayTextLine(0, "ground detected");
            //disable manual control
            stop_control_tasks();
            mail = 0;
            //pitch up
            setServoPosition(S4, elevator_servo, 120);
            //full throttle
            motor[motorA] = maxthrottlepos;
            motor[motorB] = maxthrottlepos;
            motor[motorC] = maxthrottlepos;
            //wait until ground is no longer detected
            while (SensorValue[S1] < 50) {
            }
            motor[motorA] = throttlepos;
            motor[motorB] = throttlepos;
            motor[motorC] = throttlepos;
            setServoPosition(S4, elevator_servo, 90);
            nxtDisplayTextLine(0, "ground avoided");
            //resume manual control
            resume_control_tasks();
        }
    }
}

task get_mail() {
    while (running) {
        mail = message;
        //wait to give threads time to see mail
        wait1Msec(10);
        //clear so there are no duplicate calls
        ClearMessage();
    }
}

task arm() {
    bool armed = false;
    while (running) {
        if (mail == 50) {
            //if autonomous functions are not armed, arm them
            if (armed == false) {
                displayString(3, "ARMED");
                armed = true;
                startTask(ground_avoid);
                startTask(wall_avoid);
                startTask(land);
                //if autonomous functions are armed, disarm them
            } else if (armed == true) {
                displayString(3, "DISARMED");
                armed = false;
                stopTask(wall_avoid);
                stopTask(ground_avoid);
                stopTask(land);
            }
            wait1Msec(12);
        }
    }
}

void deflect_elevator(int direction) {
    SensorType[S4] = sensorI2CCustom9V;
    elevatorpos += direction*controllchange;
    //if the elevator position is within acceptable bounds
    if (elevatorpos >= 90 - maxcontrollpos && elevatorpos <= 90 + maxcontrollpos) {
        nxtDisplayTextLine(1, "moving elevator");
        setServoPosition(S4, elevator_servo, elevatorpos);
        //else undo initial increment and display
    } else {
        elevatorpos += -1 * direction*controllchange;
        nxtDisplayTextLine(1, "at bounds");
    }
}
//same as elevator fucntion but actuates a different servo
void deflect_rudder(int direction) {
    SensorType[S4] = sensorI2CCustom9V;
    rudderpos += direction*controllchange;
    if (rudderpos >= 90 - maxcontrollpos && rudderpos <= 90 + maxcontrollpos) {
        nxtDisplayTextLine(1, "moving rudder");
        setServoPosition(S4, rudder_servo, rudderpos);
    } else {
        rudderpos += -1 * direction*controllchange;
        nxtDisplayTextLine(1, "at bounds");
    }
}
//same as elevator but changes motor power values
void increment_throttle(int direction) {
    throttlepos += throttlechange*direction;
    if (throttlepos <= maxthrottlepos && throttlepos >= -1 * (maxthrottlepos)) {
        motor[motorA] = throttlepos;
        motor[motorB] = throttlepos;
        motor[motorC] = throttlepos;
    } else {
        nxtDisplayTextLine(1, "at bounds");
        throttlepos += -1 * throttlechange*direction;
    }
    wait1Msec(12);
}
//function that stops the manual control threads so autonomous processes are not interupted 
void stop_control_tasks() {
    stopTask(yaw_left);
    stopTask(yaw_right);
    stopTask(pitch_up);
    stopTask(pitch_down);
    stopTask(throttle_up);
    stopTask(throttle_down);
    stopTask(center_controls);
}
//function that resumes manual control threads
void resume_control_tasks() {
    startTask(yaw_left);
    startTask(yaw_right);
    startTask(pitch_up);
    startTask(pitch_down);
    startTask(throttle_up);
    startTask(throttle_down);
    startTask(center_controls);
}
