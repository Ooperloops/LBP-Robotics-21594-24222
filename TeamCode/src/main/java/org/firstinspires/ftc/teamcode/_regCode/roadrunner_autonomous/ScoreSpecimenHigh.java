package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Sam's Test 2", group = "RoadRunner")
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
        hardwareManager.rightClawServo.setPosition(0);
        hardwareManager.leftClawServo.setPosition(0.25);
        hardwareManager.clawRotationServo.setPosition(0.5);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(15.05, -61.72, Math.toRadians(90.00)))
                .splineTo(new Vector2d(0.37, -35.60), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    // Run your action in here!
                    Arm(102.6);
                })
                .build();







        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectorySequence(myTrajectory);
        drive.followTrajectorySequence(trajectory0);
        scoreHighChamber();
    }
}
