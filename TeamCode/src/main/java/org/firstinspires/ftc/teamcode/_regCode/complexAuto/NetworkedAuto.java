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
                break;
            case LOADED_SPECIMEN:
                ScoreLoadedSpecimen();
                break;
        }


        for(int i = 0; i < Pushing; i++){
            // Push the stuff there
            Push(i);
        }

        for(int i = 0; i < HighBasket; i++){

        }

        if(HighRung > 0) {InitToBarTrajectory();}
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
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-35.62, -38.40), Math.toRadians(90.00))
                .splineTo(new Vector2d(-36.87, -16.04), Math.toRadians(92.34))
                .splineTo(new Vector2d(-44.00 - (9 * current), -10.07), Math.toRadians(90.00))
                .back(20)
                .turn(10)
                .strafeTo(new Vector2d(-60, -60))
                .build();

        drive.followTrajectorySequence(trajectory0);

    }
    private void GoToSub(){

    }
    private void HangSpecimenLow(){
        /*
        Arm(180);
        TrajectorySequence GoToHangingStation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(6.32, -42.70))
                .addDisplacementMarker(()->{
                    Arm(160);
                })
                .lineTo(new Vector2d(6.32, -48.12))
                .addDisplacementMarker(()->{
                    Arm(160);
                })
                .build();
        drive.setPoseEstimate(GoToHangingStation.start());
        drive.followTrajectorySequence(GoToHangingStation);
        */
    }

    private void HangSpecimenHigh(){
        InitPushFirstSamp();
        drive.followTrajectorySequence(PushSpecFirst);
        InitToBarTrajectory();
        drive.followTrajectorySequence(GetSpec);
        sleep(500);
        Claw(true);
        MoveUpwardSlide(0.02);
        sleep(100);
        MoveUpwardSlide(0);
        ScoreLoadedSpecimen();
    }

    private void ScoreLoadedSpecimen(){
        Claw(true);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    ArmToPosition(armPosition.UPSTRAIGHT);
                    hardwareManager.clawRotationServo.setPosition(0.5);
                })
                .splineTo(new Vector2d(0.37, -34.00), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
                    MoveUpwardSlide(0.3);
                })
                .addDisplacementMarker(() -> {
                    Claw(false);
                    MoveUpwardSlide(0);
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
                    ArmToPosition(armPosition.SPECIMEN_READY);
                    Claw(false);                })
                //.splineTo(new Vector2d(48, -43), Math.toRadians(90.00))
                .lineTo(new Vector2d(48.0, -59.5))
                .build();

    }

    private void InitPushFirstSamp(){
        PushSpecFirst = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(35.62, -47.98), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.59, -18.40), Math.toRadians(76.35))
                .splineTo(new Vector2d(48, -4.79), Math.toRadians(90.00))
                .build();


    }

}
