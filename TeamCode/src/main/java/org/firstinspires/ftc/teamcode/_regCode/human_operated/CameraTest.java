package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.ComputerVision;
import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;


@TeleOp(name = "Camera Test", group = "TeleOp")
public class CameraTest extends OpMode {

    private HardwareManager hardwareManager;
    private ComputerVision computerVision;

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
        computerVision = new ComputerVision(hardwareManager.camera, hardwareMap);
        computerVision.StartCamView();
    }

    @Override
    public void loop() {

    }
}