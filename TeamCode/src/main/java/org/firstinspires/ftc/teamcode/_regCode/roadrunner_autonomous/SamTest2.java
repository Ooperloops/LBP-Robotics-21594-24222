package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Sam's Test 2", group = "RoadRunner")
public class SamTest2 extends SelfDriving {
    @Override
    public void runAutonomous() {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        //Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));




        //TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
        //        .strafeRight(10)
        //        .forward(5)
        //        .build();

        //Zero arm and claw servo position
        //hardwareManager.liftServo.setPosition(0);
        hardwareManager.rightClawServo.setPosition(0);
        hardwareManager.leftClawServo.setPosition(0.25);
        hardwareManager.clawRotationServo.setPosition(0.5);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(15.49, -62.61, Math.toRadians(90.00)))
                .splineTo(new Vector2d(2.42, -37.50), Math.toRadians(90.00))
                .build();
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(2.42, -37.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(9.61, -37.36), Math.toRadians(-23.71))
                .splineTo(new Vector2d(23.71, -44.70), Math.toRadians(0.00))
                .splineTo(new Vector2d(35.30, -27.82), Math.toRadians(64.54))
                .splineTo(new Vector2d(43.08, -14.61), Math.toRadians(53.78))
                .splineTo(new Vector2d(45.14, -9.47), Math.toRadians(90.00))
                .lineTo(new Vector2d(45.43, -60.84))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(45.87, -60.84, Math.toRadians(90.00)))
                .splineTo(new Vector2d(47.19, -13.72), Math.toRadians(88.39))
                .splineTo(new Vector2d(57.03, -5.80), Math.toRadians(90.00))
                .lineTo(new Vector2d(58.64, -56.88))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(new Pose2d(58.64, -56.88, Math.toRadians(90.00)))
                .splineTo(new Vector2d(58.94, -15.93), Math.toRadians(90.00))
                .splineTo(new Vector2d(61.87, -4.33), Math.toRadians(90.00))
                .lineTo(new Vector2d(63.05, -61.43))
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(new Pose2d(63.05, -61.43, Math.toRadians(90.00)))
                .splineTo(new Vector2d(1.97, -35.45), Math.toRadians(90.00))
                .splineTo(new Vector2d(0.07, -57.00), Math.toRadians(253.74))
                .splineTo(new Vector2d(22.64, -48.70), Math.toRadians(0.00))
                .splineTo(new Vector2d(38.37, -51.18), Math.toRadians(90.00))
                .build();





        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectorySequence(myTrajectory);
        drive.followTrajectorySequence(trajectory0);
        drive.followTrajectorySequence(trajectory1);
        drive.followTrajectorySequence(trajectory2);
        drive.followTrajectorySequence(trajectory3);
    }
}