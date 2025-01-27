package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Score Specimen Low", group = "RoadRunner")
public class ScoreSpecimenLow extends SelfDriving {
    @Override
    protected void runAutonomous() {
        MoveUpwardSlide(1);
    }

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */

}