/* ------------------------------------------------------------
 * MiniCar.ino
 * Latest edited: 2012/12/15 by Jing
 * ------------------------------------------------------------ */

// Include Header
#include "math.h"
#include "Ultrasound.h"
#include "Motors.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
// Declare functions
void whichDirectToTurn();
void compassPID();
void noCompassPID();
double getAngle();
void setTurnDirection();
void goStraightAFewTime(int);
// Initial ultrasound
int usTrigPin = 7, usDangerDistance = 25;
Ultrasound us(usTrigPin, &usDangerDistance);
// Motors
Motors motors;
// Direction
HMC5883L compass;
int16_t cx, cy, cz;
// Adjust number
int xp = - (229 + -257) / 2, yp = - (94 + -624) / 2, yp2 = (229 + -257) / (229 + -257);
// adjAngle & adjAngle2: Used for adjust angle.
int startAngle, currentAngle, adjAngle, adjAngle2;

// PID arguments
long proportional;
int last_proportional, derivative, integral, power_difference, i, pid = 0;

void setup()
{
    Wire.begin();
    //Serial.begin(9600);

    motors.setRotation(true);

    compass.initialize();
    delay(1500);
    startAngle = getAngle();
    // Calculate the adjust argument.
    adjAngle = 180 - startAngle;
    adjAngle2 = 180 + startAngle;
    // make angle of begin is 180
    startAngle = 180;
    delay(500);
}

// turnLeft is a flag for car that which direct turn.
int angleTemp, turnLeft = 1;

void loop()
{
    if (us.isDanger())
    {
        whichDirectToTurn();
        // Turn right/left until it's not in front.
        setTurnDirection();
        // delay 100 milli seconds while it's dangerous.
        while (us.isDanger())
            delay(150);
        // Go straight a few time (600 ms)
        goStraightAFewTime(600);
    }
    else
    {
        // unsigned long lastTime = millis();
        // int timeout = 1500;
        // set motors are forward.
        motors.setRotation(true);
        // Do PID controller in 1.5 seconds if pid flag is up(pid=1).
        // When is it stop?
        //  - Time up and the difference of angle between 5 and -5 degree.
        //  - Watch something in front.
        do
        {
            compassPID();
            //noCompassPID();
        }
        while ((((((currentAngle - startAngle) > 2 || (currentAngle - startAngle) < -2))) && !us.isDanger()) && pid);
        // turn off PID controller
        pid = 0;
        // Do straight in 1.5 seconds.
        do
        {
            motors.setMotors(255, 255);
        }
        while (!pid && !us.isDanger());
    }
}

void whichDirectToTurn()
{
    // compare current direction and previous direction
    // to decision car should turn right or turn left.
    // if turnLeft = 1 then turn left.
    currentAngle = getAngle();
    if ((currentAngle - startAngle) < 0)
        turnLeft = 0;
    else if ((currentAngle - startAngle) > 0)
        turnLeft = 1;
    //turnLeft = !turnLeft;
}

void setTurnDirection()
{
    if (turnLeft)
    {
        motors.setMotors(170, 0);
    }
    else
    {
        motors.setMotors(0, 170);
    }
}

void goStraightAFewTime(int timeout)
{
    angleTemp = startAngle;
    startAngle = getAngle();
    // delay a few time and fetch angle again.
    delay(10);
    startAngle = getAngle();

    unsigned long lastTime = millis();

    motors.setRotation(true);

    // go straight in 1 second.
    do
    {
        // if find something in front.
        if (us.isDanger())
        {
            // Because has obstruction in front, the car must turn its direction.
            // we must escape this action.
            break;
        }
        // If nothing in front, the car will go straight.
        compassPID();
    }
    while ((millis() - lastTime) < timeout);
    /* -------- Return to normal ------- */
    startAngle = angleTemp;
    // turn on PID controller
    pid = 1;
}

void compassPID()
{
    // Fetch current angle.
    currentAngle = getAngle();

    /* ----- Compass PID Control Start ----- */
    // proportional = error
    proportional = (int) (currentAngle - startAngle);
    // derivative
    derivative = proportional - last_proportional;
    // integral
    integral += proportional;

    last_proportional = proportional;

    // power_difference = U = Kp * P + Ki * I + Dd * D
    //power_difference = (int) proportional * 132 / 1000 + integral / 400 + derivative * 47 / 10;
    //power_difference = (int) proportional * 7 / 3 + integral / 5000 + derivative * 3 / 2;
    power_difference = (int) proportional * 232 / 10 + derivative * 33 / 10;
    //power_difference = (int) proportional * 250 / 100 + derivative * 100 / 10;

    // Motors speed.
    int maxs = 255;

    if (power_difference > maxs)
        power_difference = maxs;
    if (power_difference < -maxs)
        power_difference = -maxs;

    if (power_difference < 0)
    {
        motors.setMotors(maxs + power_difference, maxs);
    }
    else
    {
        motors.setMotors(maxs, maxs - power_difference);
    }
    /* ------ Compass PID Control End ------ */
}

// No PID controller. Only compare current angle and start angle.
void noCompassPID()
{
    // Fetch current angle.
    currentAngle = getAngle();
    // Decide motor 1 and motor 2 speed by compare angle.
    if ((currentAngle - startAngle) > 6)
        motors.setMotors(250, 0);
    else if ((currentAngle - startAngle) > 2)
        motors.setMotors(250, 150);
    else if ((currentAngle - startAngle) < -6)
        motors.setMotors(0, 250);
    else if ((currentAngle - startAngle) < -2)
        motors.setMotors(150, 250);
}

// Get and calculate current angle.
double getAngle()
{
    // get x, y, z.
    compass.getHeading(&cx, &cy, &cz);
    // calculate the angle of 3 vectors.
    int angle = atan2((double) ((cx + yp) * yp2), (double) ((cz + xp) * 1)) * (180 / M_PI) + 180;

    // Make angle from physical to virtual.
    if (angle >= 0 && angle < adjAngle2)
    {
        angle = angle + adjAngle;
        // Ensure no native number.
        if (angle < 0)
        {
            angle = 360 + angle;
        }
    }
    else if (angle >= adjAngle2 && angle < 360)
    {
        angle = angle - adjAngle2;
    }

    return angle;
}
