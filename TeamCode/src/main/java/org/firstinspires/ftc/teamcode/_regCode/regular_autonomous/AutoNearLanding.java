package org.firstinspires.ftc.teamcode._regCode.regular_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;

@Autonomous(name = "Auto Near Landing", group = "autonomous")
public class AutoNearLanding extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory0 = drive.trajectoryBuilder(new Pose2d(new Vector2d(0,0), Math.toRadians(0)))
                .splineTo(new Vector2d(40.04, -5.21), Math.toRadians(-90.00))
                .build();

        drive.followTrajectory(trajectory0);

    }
}