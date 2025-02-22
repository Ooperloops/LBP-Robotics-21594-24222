package org.firstinspires.ftc.teamcode.all_purpose;

import com.qualcomm.robotcore.hardware.Servo;

public class ReverseServoWrapper {
    private Servo currentServo;
    public ReverseServoWrapper(Servo servo){
        currentServo = servo;
    }

    public void setPosition(double position){
        double newPos = (1- position);
        currentServo.setPosition(newPos);
    }
}
