#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define LENGTH 1200
#define TIME_STEP 0.1

float PID_Step(float measurement, float setpoint, float Kp, float Ki, float Kd, float Kaw, float T_C, float T, float max, float min, float max_rate)
{
    /* This function implements a PID controller. 
     * It takes in the current measurement, desired setpoint, and gain constants 
     * for proportional, integral, derivative, and anti-windup terms, as well as time constants 
     * for derivative filtering and the time step, and finally max, min and max rate of change
     * of the command. 
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   Kp: proportional gain constant
     *   Ki: integral gain constant
     *   Kd: derivative gain constant
     *   Kaw: anti-windup gain constant
     *   T_C: time constant for derivative filtering
     *   T: time step
     *   max: max command
     *   min: min command
     *   max_rate: max rate of change of the command
     *
     * Returns:
     *   command_sat: the control output of the PID controller (saturated based on max. min, max_rate)
     */

    /* Some variables are static as they need to retain their value
     * from the previous iteration for the calculations in the current iteration.
     */
    static float integral = 0;
    static float err_prev = 0;
    static float deriv_prev = 0;
    static float command_sat_prev = 0;
    static float command_prev = 0;
    float err;
    float command;
    float command_sat;
    float deriv_filt;

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation - including anti-windup */
    integral += Ki*err*T + Kaw*(command_sat_prev - command_prev)*T;
    
    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - err_prev + T_C*deriv_prev)/(T + T_C);
    err_prev = err;
    deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    command = Kp*err + integral + Kd*deriv_filt;

    /* Remember command at previous step */
    command_prev = command;

    /* Saturate command */
    if (command > max)
    {
        command_sat = max;
    }
    else if (command < min)
    {
        command_sat = min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > command_sat_prev + max_rate*T)
    {
        command_sat = command_sat_prev + max_rate*T;
    }
    else if (command_sat < command_sat_prev - max_rate*T)
    {
        command_sat = command_sat_prev - max_rate*T;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    command_sat_prev = command_sat;

    return command_sat;
}

float Object_Step(float F, float m, float k, float F_max, float F_min, float T){
    /* The Object_Step function calculates the position of an object moving in 1D, 
     * pushed by a force F and subject to viscous damping with coefficient k, given its
     * mass, max force, min force, and time step.
     *
     * Inputs:
     *   F: forced applied to the object
     *   m: mass of the object
     *   k: damping constant
     *   F_max: max force
     *   F_min: min force
     *   T: time step
     *
     * Returns:
     *   z: position of the object in meters
     */

    /* v(t) and z(t) are static as they need to retain their value
     * from the previous iteration for the calculations in the current iteration.
     */
    static float v = 0;
    static float z = 0;

    /* dv/dt and saturated command */
    float dv_dt;
    float F_sat;

    /* Apply saturation */
    if (F > F_max)
    {
        F_sat = F_max;
    }
    else if (F < F_min)
    {
        F_sat = F_min;
    }
    else
    {
        F_sat = F;
    }

    /* Calculate derivative dv(t)/dt */
    dv_dt = (F_sat - k*v)/m;

    /* Integrate derivative to find v(t) and z(t) */
    v += dv_dt*T;
    z += v*T;

    return z;
}

int main()
{
    /* Define variables  */
    float t = 0;
    int i = 0;
    float command = 0;
    float stp = 100;
    float z = 0;

    /* Open file for logger */
    FILE *file = fopen("data_PID_C.txt", "w");

    /* Implement iteration using while loop */
    while(i < LENGTH)
    {
        /* Change setpoint at t = 60 seconds */
        if (t < 60)
        {
            stp = 100;
        }
        else
        {
            stp = 200;
        }
        /* Run PID_Step to get command, 
        and then run Object_Step using the command from the PID_Step  */
        command = PID_Step(z, stp, 1, 0.1, 5, 0.1, 1, TIME_STEP, 100, -100, 40);
        z = Object_Step(command, 10, 0.5, 100, -100, TIME_STEP);

        /* Log variables in the text file */
        fprintf(file,"%f %f %f %f\n", t, command, z, stp);

        /* Increment time and counter */
        t = t + TIME_STEP;
        i = i + 1;
    }

    fclose(file);
    exit(0);
}