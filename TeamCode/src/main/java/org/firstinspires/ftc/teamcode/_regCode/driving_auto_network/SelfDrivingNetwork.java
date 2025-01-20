package org.firstinspires.ftc.teamcode._regCode.driving_auto_network;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a neural-network style autonomous:
 * Each action (i.e Parking, Score High Basket, etc.) each have a specified double associated with it.
 * The higher the double, the higher said action is prioritized by the robot.
 * Like a neural network, there are inputs that effect what action output maybe given a certain time.
 * Input includes: time elapsed, distance from the
 *
 */

enum Actions{
    PARK,
    PUSH,
    GO_FISHING,
    SPECIMEN
}

public abstract class SelfDrivingNetwork extends SelfDriving {

    protected SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //------------------------------------------------------------------------------------------------
    // Weight Variables
    //------------------------------------------------------------------------------------------------

    protected double time__park = 1;
    protected double time__push = 1;
    protected double time__fish = 1;
    protected double time__specimen = 1;

    protected int s;

    //------------------------------------------------------------------------------------------------
    // Input Variables
    //------------------------------------------------------------------------------------------------

    @Override
    protected void runAutonomous(){
        initVariables();

        for(int i = 0; i < s ;i++){
            HangSpecimenLow(new Pose2d(11.59, -60.34, Math.toRadians(90.00)));
        }
    }

    protected abstract void initVariables();

    //------------------------------------------------------------------------------------------------
    // Actions
    //------------------------------------------------------------------------------------------------

    private void Park(){

    }
    private void Push(){

    }
    private void GoToSub(){

    }
    private void HangSpecimenLow(Pose2d prevPose){
        TrajectorySequence GoToHangingStation = drive.trajectorySequenceBuilder(prevPose)
                .addDisplacementMarker(0, () -> {
                    Arm(180);
                })
                .splineTo(new Vector2d(6.32, -42.70), Math.toRadians(90.49))
                .addDisplacementMarker(() -> {
                    Arm(160);
                })
                .lineTo(new Vector2d(6.32, -44.12))
                .addDisplacementMarker(() -> {
                    Claw(false);
                })
                .build();
        drive.setPoseEstimate(GoToHangingStation.start());
        drive.followTrajectorySequence(GoToHangingStation);

    }
    private void HangSpecimenHigh(){

    }

}
