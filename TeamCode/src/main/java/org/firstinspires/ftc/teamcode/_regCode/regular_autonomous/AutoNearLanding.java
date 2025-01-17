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

        Trajectory trajectory0 = drive.trajectoryBuilder(new Pose2d(-12.84, -59.22, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-41.31, -12.71), Math.toRadians(121.46))
                .splineTo(new Vector2d(-14.09, 37.84), Math.toRadians(61.70))
                .splineTo(new Vector2d(-12.29, -58.81), Math.toRadians(-87.92))
                .build();
        drive.setPoseEstimate(trajectory0.start());

        drive.followTrajectory(trajectory0);

    }
}