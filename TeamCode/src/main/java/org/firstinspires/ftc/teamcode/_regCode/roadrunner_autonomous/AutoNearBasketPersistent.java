package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Random;

@Autonomous(name = "Auto Near Basket Persistent", group = "RoadRunner")
public class AutoNearBasketPersistent extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    private final int NumberOfTriesInSub = 3;

    SampleMecanumDrive drive;

    @Override
    public void runAutonomous() {
        MoveUpwardSlide(1);
        sleep(1);
        MoveUpwardSlide(0);
    }

    private TrajectorySequence GoToBasketFromSub(Pose2d currentPos){
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(currentPos)
                .splineTo(new Vector2d(-58.39, 0.49), Math.toRadians(205.91))
                .splineTo(new Vector2d(-38.67, -15.76), Math.toRadians(-71.13))
                .splineTo(new Vector2d(-35.34, -37.70), Math.toRadians(265.91))
                .splineTo(new Vector2d(-51.31, -52.84), Math.toRadians(224.22))
                .build();
        return trajectory0;
    }

    private TrajectorySequence GoToSubRandomFromBasket(Pose2d currentPos){
        double XPosMin = -8.26, XPosMax = 8.26;
        Random rand = new Random();

        double RandomXPos = rand.nextDouble() * (XPosMax - XPosMin + 1) + XPosMin;

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(currentPos)
                .splineTo(new Vector2d(-36.17, -43.53), Math.toRadians(39.16))
                .splineTo(new Vector2d(-40.62, -8.96), Math.toRadians(94.66))
                .splineTo(new Vector2d(-54.50, 0.90), Math.toRadians(149.44))
                .splineTo(new Vector2d(-27.70, RandomXPos), Math.toRadians(-0.94))
                .build();
        return trajectory0;
    }
}