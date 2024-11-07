package org.firstinspires.ftc.teamcode.base;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDAutoMove {
    private double Ki;
    private double Kd;
    private double Kp;

    private double P;
    private double I;
    private double D;
    private double deltaTimeSeconds = 0.05;
    private ElapsedTime elapsedTime;

    private  double lastErrorPosition;

    public PIDAutoMove(double Kp, double Ki, double Kd, double deltaTimeSeconds)
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

    public double OnUpdatePower(double currentTheta, double targetTheta){
        double error = (targetTheta - currentTheta);
        P = Proportion(error);
        if(elapsedTime.milliseconds() % deltaTimeSeconds * 1000 == 0) {
            D = Derivative(error);
            I = Integral(error);
        }

        return P + I + D;
    }

    //------------------------------------------------------------------------------------------------
    // Private Methods
    //------------------------------------------------------------------------------------------------

    private double Integral(double error){
        return I*((I/Ki) + error);
    }

    private double Derivative(double error){
        double DeDt = (error - lastErrorPosition)/deltaTimeSeconds;
        lastErrorPosition = DeDt;
        return Kd * (DeDt);
    }

    private double Proportion(double error){
        return error * Kp;
    }
}
