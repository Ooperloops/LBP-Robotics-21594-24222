package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;


@Autonomous(name = "Motor Test HALF POWER", group = "TeleOp")
public class MotorTestHalf extends LinearOpMode {

    private HardwareManager hardwareManager;
    private ElapsedTime timeElapsed;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        hardwareManager = new HardwareManager(hardwareMap);
        timeElapsed = new ElapsedTime();
        hardwareManager.backRightWheel.setPower(0.5);
        while(opModeIsActive() && timeElapsed.milliseconds() < 5000){
            idle();
        }
        hardwareManager.backRightWheel.setPower(0);
    }
}