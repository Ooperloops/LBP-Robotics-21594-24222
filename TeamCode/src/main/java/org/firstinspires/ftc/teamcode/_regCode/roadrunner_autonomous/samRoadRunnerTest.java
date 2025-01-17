package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Sam's Test", group = "RoadRunner")
public class samRoadRunnerTest extends SelfDriving {
    @Override
    public void runAutonomous() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        //Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));




        //TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
        //        .strafeRight(10)
        //        .forward(5)
        //        .build();

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(12.40, -59.08, Math.toRadians(90.00)))

                .splineTo(new Vector2d(34.13, -38.39), Math.toRadians(56.82))
                .splineTo(new Vector2d(38.39, -28.99), Math.toRadians(33.16))
                .splineTo(new Vector2d(38.39, 1.83), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.62, 39.41), Math.toRadians(180.00))
                .splineTo(new Vector2d(6.68, 39.41), Math.toRadians(161.57))
                .splineTo(new Vector2d(5.50, 55.56), Math.toRadians(-26.57))
                .splineTo(new Vector2d(34.86, 59.96), Math.toRadians(-60.95))
                .splineTo(new Vector2d(56.29, 33.39), Math.toRadians(270.00))
                .splineTo(new Vector2d(54.68, 14.75), Math.toRadians(270.00))
                .splineTo(new Vector2d(54.24, -11.82), Math.toRadians(-45.00))
                .splineTo(new Vector2d(56.44, -33.10), Math.toRadians(270.00))
                .splineTo(new Vector2d(53.65, -59.96), Math.toRadians(270.00))
                .build();

        drive.setPoseEstimate(trajectory0.start());
        gi
        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectorySequence(myTrajectory);
        drive.followTrajectorySequence(trajectory0);
    }
}