package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Sam's Test", group = "RoadRunner")
public class samRoadRunnerTest extends SelfDriving {
    @Override
    public void runAutonomous() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);
    }
}