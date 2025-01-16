package org.firstinspires.ftc.teamcode._regCode.base;


import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDControl {

    //------------------------------------------------------------------------------------------------
    // PID Constants
    //------------------------------------------------------------------------------------------------
    private double Ki; // Integral constant value multiplied with the integral
    private double Kd; // Derivative constant value multiplied with the derivative
    private double Kp; // Proportion constant value multiplied with the proportion

    //------------------------------------------------------------------------------------------------
    // PID Values
    //------------------------------------------------------------------------------------------------

    private double P; // Proportion value
    private double I; // Integral value
    private double D; // Derivative value
    private double deltaTimeSeconds; // change in time for the derivative


    private ElapsedTime elapsedTime;

    private  double lastErrorPosition;

    public PIDControl(double Kp, double Ki, double Kd, double deltaTimeSeconds)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.deltaTimeSeconds = deltaTimeSeconds;
        elapsedTime = new ElapsedTime();
    }

    //------------------------------------------------------------------------------------------------
    // Public Methods
    //------------------------------------------------------------------------------------------------
    /*
     * Meant to be called in a loop (i.e while-loop or the loop function in OpMode class)
     * -> takes in current value and target value
     * -> outputs correction value as a double
     */
    public double OnUpdatePower(double currentTheta, double targetTheta){
        double error = (targetTheta - currentTheta);
        P = Proportion(error); // Set proportion val to current error
        D = Derivative(error); // Set derivative of error with respect to time
        I = Integral(error); // return the sum of all errors

        return (P * Kp) + (I * Ki) + (D * Kd);
    }

    //------------------------------------------------------------------------------------------------
    // Private Methods
    //------------------------------------------------------------------------------------------------

    private double Integral(double error){
        return I + error; // Added error to integral value
    }

    private double Derivative(double error){
        double DeDt = (error - lastErrorPosition)/deltaTimeSeconds; // Get approximate derivative
        lastErrorPosition = error; // Set input error to previous error
        return DeDt; // return derivative
    }

    private double Proportion(double error){
        return error; //return error
    }
}
