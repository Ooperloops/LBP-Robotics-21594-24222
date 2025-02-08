package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;
import org.firstinspires.ftc.teamcode._regCode.base.HumanOperated;


@Autonomous(name = "Motor Test FULL POWER", group = "TeleOp")
public class MotorTest extends LinearOpMode {

    private HardwareManager hardwareManager;
    private ElapsedTime timeElapsed;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        hardwareManager = new HardwareManager(hardwareMap);
        timeElapsed = new ElapsedTime();
        hardwareManager.backRightWheel.setPower(1);
        while(opModeIsActive() && timeElapsed.milliseconds() < 5000){
            idle();
        }
        hardwareManager.backRightWheel.setPower(0);
    }
}