package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode._regCode.complexAuto.NetworkedAuto;
import org.firstinspires.ftc.teamcode._regCode.complexAuto.StartPos;

@Autonomous(name = "DUMB PARK AUTO")
public class DumbParkAuto extends SelfDriving {

    @Override
    protected void runAutonomous() {
        rotate(90.0);
        timedMove(5000, 0.7, true);
    }
}