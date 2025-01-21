package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode._regCode.complexAuto.NetworkedAuto;
import org.firstinspires.ftc.teamcode._regCode.complexAuto.StartPos;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Near Basket", group = "RoadRunner")
public class AutoNearBasket extends NetworkedAuto {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */

    @Override
    protected void initVariables() {
        startPosition = StartPos.FARBASKET;

        HighBasket = 0;
        Pushing = 0;
        HighRung = 0;
        Parking = true;

    }
}