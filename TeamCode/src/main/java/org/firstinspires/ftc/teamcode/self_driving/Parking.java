package org.firstinspires.ftc.teamcode.self_driving;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.HumanOperated;
import org.firstinspires.ftc.teamcode.base.SelfDriving;


@Autonomous(name = "Parking near observation (EXPERIMENTAL)", group = "autonomous")
public class Parking extends SelfDriving {

    @Override
    protected void runAutonomous() {
        rotate(90);
        move(4);
    }
}