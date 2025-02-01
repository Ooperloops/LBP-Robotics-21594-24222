package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Score High Chamber", group = "RoadRunner")
public class ScoreSpecimenHigh extends SelfDriving {
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
        //Claw(true);
        //hardwareManager.clawRotationServo.setPosition(0.5);
        /*
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(15.05, -61.72, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    // Run your action in here!
                    Arm(102.6);
                    hardwareManager.clawRotationServo.setPosition(0.5);
                })
                .splineTo(new Vector2d(0.37, -35.60), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
                    // This marker runs after the previous splineTo()
                    // Run your action in here!
                    MoveUpwardSlide(0.3);
                })
                .build();

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
                .strafeLeft(8)
                .build();

        */

        drive.setPoseEstimate(new Pose2d(34.51, -59.36, Math.toRadians(90.00)));
        TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(-36.73, -58.81))
                .build();

        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(34.51, -59.36))
                .build();







        for(int i = 0; i < 5; i++){
            drive.followTrajectorySequence(strafeLeft);
            drive.followTrajectorySequence(strafeRight);
        }
        //drive.followTrajectorySequence(myTrajectory);
        //drive.followTrajectorySequence(trajectory0);
        //scoreHighBar();
        //drive.followTrajectorySequence(trajectory1);

    }
}
