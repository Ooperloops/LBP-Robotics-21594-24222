package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Near Basket", group = "RoadRunner")
public class AutoNearBasket extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-12.15, -62.84, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-64.64, -70.61), Math.toRadians(224.01))
                .build();



        drive.setPoseEstimate(traj.start());

        drive.followTrajectorySequence(traj);
        sleep(1000);
        ScoreHighBasket();
    }
}