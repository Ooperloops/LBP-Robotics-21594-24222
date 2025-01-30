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
    private TrajectorySequence GoToLandingThenHangSpec;
    private TrajectorySequence s;
    //------------------------------------------------------------------------------------------------
    // Start Method
    //------------------------------------------------------------------------------------------------

    @Override
    protected void runAutonomous(){
        initVariables();
        switch(startPosition){
            case FARBASKET:
                drive.setPoseEstimate(new Pose2d(11.59, -61.03, Math.toRadians(90.0)));
                break;
            case MIDBASKET:
                drive.setPoseEstimate(new Pose2d(-12.99, -61.03, Math.toRadians(90.00)));
                break;
            case NEARBASKET:
                drive.setPoseEstimate(new Pose2d(-35.63, -61.03, Math.toRadians(90.00)));
                break;
        }

        switch(loadedPosition){
            case LOADED_SAMPLE:
                break;
            case LOADED_SPECIMEN:
                ScoreLoadedSpecimen();
                break;
        }

        if(HighBasket > 0) {}
        for(int i = 0; i < HighBasket; i++){
            // Startscoring HighBasket
            HangSpecimenHigh();
        }

        if(HighRung > 0) {InitToBarTrajectory();}
        for(int i = 0; i < HighRung; i++){
            // Start scoring my hanging specimen at high rung
            HangSpecimenHigh();
        }

        if(Pushing > 0) {}
        for(int i = 0; i < Pushing; i++){
            // Startscoring HighBasket
        }

        if (Parking) { Park();}

    }

    protected abstract void initVariables();



    //------------------------------------------------------------------------------------------------
    // Actions
    //------------------------------------------------------------------------------------------------

    private void Park(){
        TrajectorySequence parkTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(60.34, -61.03))
                .build();

        drive.followTrajectorySequence(parkTraj);

    }
    private void Push(){

    }
    private void GoToSub(){

    }
    private void HangSpecimenLow(){
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

    }

    private void HangSpecimenHigh(){
        drive.followTrajectorySequence(GoToLandingThenHangSpec);
        scoreHighBar();
    }

    private void ScoreLoadedSpecimen(){
        Claw(true);
        hardwareManager.clawRotationServo.setPosition(0.5);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    Arm(102.6);
                })
                .splineTo(new Vector2d(0.37, -35.60), Math.toRadians(90.00))

                .build();
        drive.followTrajectorySequence(trajectory0);
        scoreHighBar();
    }

    //------------------------------------------------------------------------------------------------
    // Trajectory Initializers
    //------------------------------------------------------------------------------------------------
    private void InitToBarTrajectory(){
        GoToLandingThenHangSpec = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(()->{
                    ArmToPosition(armPosition.SPECIMEN_READY);
                    Claw(false);                })
                .splineTo(new Vector2d(6.18, -38.81), Math.toRadians(90.00))
                .strafeTo(new Vector2d(43.39, -53.39))
                .addDisplacementMarker(()-> {
                    Claw(true);
                    MoveUpwardSlide(0.02);
                    MoveUpwardSlide(0);
                })
                .strafeTo(new Vector2d(43.39, -53.39))
                .build();

    }

}
