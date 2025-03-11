package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;


@Autonomous(name = "Motor Min Max Test Tuner", group = "TeleOp")
public class MotorTest extends OpMode {

    private HardwareManager hardwareManager;

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }

    @Override
    public void loop() {
        hardwareManager.liftMotorLeft.setPower(gamepad1.left_stick_y);
        hardwareManager.liftMotorRight.setPower(gamepad1.left_stick_y);

        telemetry.addData("CurrentMotorTicks", hardwareManager.liftMotorLeft.getCurrentPosition());
        telemetry.update();
    }
}