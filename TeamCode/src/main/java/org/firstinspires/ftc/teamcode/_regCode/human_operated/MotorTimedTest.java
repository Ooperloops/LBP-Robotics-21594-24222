package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;


@Autonomous(name = "Motor Fast Tuner", group = "TeleOp")
public class MotorTimedTest extends LinearOpMode {

    private HardwareManager hardwareManager;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareManager = new HardwareManager(hardwareMap);
        //waitForStart();
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < 300){
            hardwareManager.leftAscentMotor.setPower(1.0);
            idle();
        }

        hardwareManager.leftAscentMotor.setPower(0.0);
    }
}