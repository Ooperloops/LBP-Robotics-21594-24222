package org.firstinspires.ftc.teamcode._regCode.complexAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/**
`* Networked Auto is a class that makes creating different autonomous scripts easier
 *
 */
public abstract class NetworkedAuto extends SelfDriving {
    //------------------------------------------------------------------------------------------------
    // Weight Variables
    //------------------------------------------------------------------------------------------------
    protected int HighBasket = 0;
    protected int Pushing = 0;
    protected int HighRung = 0;
    protected boolean Parking = false;
    protected StartPos startPosition;
    protected LoadedPosition loadedPosition;

    //------------------------------------------------------------------------------------------------
    // Private enums
    //------------------------------------------------------------------------------------------------
    public enum LoadedPosition{
        LOADED_SPECIMEN,
        LOADED_SAMPLE
    }
    //------------------------------------------------------------------------------------------------
    // Trajectory Variables
    //------------------------------------------------------------------------------------------------
    private TrajectorySequence GetSpec;
    private TrajectorySequence PushSpecFirst;
    //------------------------------------------------------------------------------------------------
    // Start Method
    //------------------------------------------------------------------------------------------------

    @Override
    protected void runAutonomous(){
        hardwareManager.ResetLiftWheelCount();
        initVariables();
        switch(startPosition){
            // set enum to specific position where robot starts
            // --> done to improve readability
            case FARBASKET:
                drive.setPoseEstimate(new Pose2d(11.59, -62.7, Math.toRadians(90.0)));
                break;
            case MIDBASKET:
                drive.setPoseEstimate(new Pose2d(-12.99, -62.7, Math.toRadians(90.00)));
                break;
            case NEARBASKET:
                drive.setPoseEstimate(new Pose2d(-35.63, -62.7, Math.toRadians(90.00)));
                break;
        }

        switch(loadedPosition){
            case LOADED_SAMPLE:
                // TODO: create auto to score a loaded sample on the high bucket
                break;
            case LOADED_SPECIMEN:
                ScoreLoadedSpecimen(); // scores loaded specimen on the high rung
                break;
        }


        for(int i = 0; i < Pushing; i++){
            // Pushes yellow samples under the bucket (needs testing)
            Push(i);
        }

        for(int i = 0; i < HighBasket; i++){
            // TODO: implement high basket scoring for auto
        }

        for(int i = 0; i < HighRung; i++){
            // Start scoring my hanging specimen at high rung
            HangSpecimenHigh();
        }

        if (Parking) { Park();}

    }

    protected abstract void initVariables();

    //------------------------------------------------------------------------------------------------
    // Actions
    //------------------------------------------------------------------------------------------------

    private void Park(){
        // Parks bot at observation zone
        TrajectorySequence parkTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(59, -60))
                .build();

        drive.followTrajectorySequence(parkTraj);

    }
    private void Push(int current){
        TrajectorySequence pushTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                // --- Moves past the yellow samples to get into pushing position -------
                .splineTo(new Vector2d(-35.62, -38.40), Math.toRadians(90.00))
                .splineTo(new Vector2d(-36.87, -16.04), Math.toRadians(92.34))
                .splineTo(new Vector2d(-44.00 - (9 * current), -10.07), Math.toRadians(90.00))
                // ----------------------------------------------------------------------
                .back(20) // reverse back 20 inches
                .turn(10) // rotate 10 degrees clockwise
                .strafeTo(new Vector2d(-60, -60)) // strafe to the push zone below the buckets
                .build();

        drive.followTrajectorySequence(pushTraj);

    }
    private void ScoreHighBucket(){
        // TODO: create RR script trajectory that goes to the sub and gets a sample for bucket scoring
    }

    private void HangSpecimenHigh(){
        InitPushFirstSamp(); // initialize pushing trajectory
        drive.followTrajectorySequence(PushSpecFirst); // push coloured sample into observation zone
        InitToBarTrajectory(); // initialize grabbing specimen from wall trajectory
        drive.followTrajectorySequence(GetSpec); // bot grabs wall specimen
        sleep(500); // short delay for human player correction
        Claw(true); // close the claw
        MoveUpwardSlide(0.02); // move slide up to remove specimen from wall
        sleep(100);
        MoveUpwardSlide(0);
        ScoreLoadedSpecimen(); // Go to bar and hang specimen
    }

    private void ScoreLoadedSpecimen(){
        Claw(true); // close the claw

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    ArmToPosition(armPosition.UPSTRAIGHT); // make arm perpendicular to drivebase
                    hardwareManager.clawRotationServo.setPosition(0.5); // set wrist to proper position for hanging
                })
                .splineTo(new Vector2d(0.37, -34.00), Math.toRadians(90.00)) // go near the sub
                .addDisplacementMarker(() -> {
                    MoveUpwardSlide(0.3); // move slide up to hang specimen
                })
                .addDisplacementMarker(() -> {
                    Claw(false); // open the claw
                    MoveUpwardSlide(0); // retract the lift back down
                })
                .back(15)
                .build();


        drive.followTrajectorySequence(trajectory0);
    }

    //------------------------------------------------------------------------------------------------
    // Trajectory Initializers
    //------------------------------------------------------------------------------------------------
    private void InitToBarTrajectory(){
        GetSpec = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(()->{
                    ArmToPosition(armPosition.SPECIMEN_READY); // Rotates arm behind itself
                    Claw(false);                }) // opens the claw
                .lineTo(new Vector2d(48.0, -59.5)) // reverses to the wall where the specimen is
                .build();

    }

    private void InitPushFirstSamp(){
        PushSpecFirst = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                // Moves past the coloured samples to get into pushing position
                .splineTo(new Vector2d(35.62, -47.98), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.59, -18.40), Math.toRadians(76.35))
                .splineTo(new Vector2d(48, -4.79), Math.toRadians(90.00))
                .build();

    }

}
