package org.firstinspires.ftc.teamcode._regCode.complexAuto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;



/**
`* Networked Auto is a class that makes creating different autonomous scripts easier
 *
 */
public abstract class NetworkedAuto extends SelfDriving {
    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected MecanumDrive drive;
    //------------------------------------------------------------------------------------------------
    // Weight Variables
    //------------------------------------------------------------------------------------------------
    protected int HighBasket = 0;
    protected int HighRung = 0;
    protected long startDelayMili = 0;
    protected boolean Parking = false;
    protected boolean MoveOutOfWay = false;
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
    private Action GetSpec;
    private Action PushSpecFirst;
    private Pose2d currentPose;
    //------------------------------------------------------------------------------------------------
    // Start Method
    //------------------------------------------------------------------------------------------------

    @Override
    protected void runAutonomous(){
        //Initialize drive class


        hardwareManager.ResetLiftWheelCount();
        initVariables();
        sleep(startDelayMili);

        switch(startPosition){
            // set enum to specific position where robot starts
            // --> done to improve readability
            case FARBASKET:
                currentPose = new Pose2d(11.59, -62.7, Math.toRadians(90.0));
                break;
            case MIDBASKET:
                currentPose = new Pose2d(-12.99, -62.7, Math.toRadians(90.00));
                break;
            case NEARBASKET:
                currentPose = new Pose2d(-35.63, -62.7, Math.toRadians(90.00));
                break;
        }
        drive = new MecanumDrive(hardwareMap, currentPose);
        drive.localizer.update();

        switch(loadedPosition){
            case LOADED_SAMPLE:
                // TODO: create auto to score a loaded sample on the high bucket
                break;
            case LOADED_SPECIMEN:
                ScoreLoadedSpecimen(0); // scores loaded specimen on the high rung
                break;
        }

        for(int i = 0; i < HighBasket; i++){
            // TODO: implement high basket scoring for auto
        }

        for(int i = 0; i < HighRung; i++){
            // Start scoring my hanging specimen at high rung
            HangPushCycle(i);
        }



        if (Parking) { Park();}
        if (MoveOutOfWay) {MoveOutOfTheWay();}

    }

    protected abstract void initVariables();

    //------------------------------------------------------------------------------------------------
    // Actions
    //------------------------------------------------------------------------------------------------

    private void Park(){
        // Parks bot at observation zone
        Action parkTraj = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(90.0))
                .strafeTo(new Vector2d(59, -60))
                .build();
        Actions.runBlocking(parkTraj);
        drive.localizer.update();

    }
    private void ScoreHighBucket(){
        // TODO: create RR script trajectory that goes to the sub and gets a sample for bucket scoring
    }

    private void HangPushCycle(int i){
        // initialize pushing trajectory
        PushSpecFirst = drive.actionBuilder(drive.localizer.getPose())
                // Moves past the coloured samples to get into pushing position
                .splineTo(new Vector2d(35.62, -47.98), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.59, -18.40), Math.toRadians(76.35))
                .splineTo(new Vector2d(48 + (9 * i), -4.79), Math.toRadians(90.00))
                .build();
        Actions.runBlocking(PushSpecFirst); // push coloured sample into observation zone
        drive.localizer.update();
        // initialize grabbing specimen from wall trajectory
        GetSpec = drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(()->{
                    ArmToPosition(armPosition.SPECIMEN_READY); // Rotates arm behind itself
                    Claw(false);                }) // opens the claw
                .setTangent(Math.toRadians(90.00))
                .strafeTo(new Vector2d(47, -38))
                .lineToY( -60) // reverses to the wall where the specimen is
                .waitSeconds(0.5) // short delay for human player correction
                .stopAndAdd(() -> {
                    Claw(true); // close the claw
                    MoveUpwardSlide(0.02); // move slide up to remove specimen from wall
                })
                .waitSeconds(0.1)
                .stopAndAdd(() -> {
                    MoveUpwardSlide(0);
                    ScoreLoadedSpecimen(0.90); // Go to bar and hang specimen
                })
                .build();
        Actions.runBlocking(GetSpec); // bot grabs wall specimen
        drive.localizer.update();
    }


    private void ScoreLoadedSpecimen(double displacement){
        Claw(true); // close the claw

        Action trajectory0 = drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(() -> {
                    //ArmToPosition(armPosition.UPSTRAIGHT); // make arm perpendicular to drivebase
                    //hardwareManager.leftArmServo.setPosition(0.5);
                    //hardwareManager.rightArmServo.setPosition(0.5);
                    //TODO: move vertical servo claw to proper position
                })
                .splineTo(new Vector2d(0.37, -35.00 + displacement), Math.toRadians(90.00)) // go near the sub
                .stopAndAdd(() -> {
                    //MoveUpwardSlide(0.33); // move slide up to hang specimen
                    //Claw(false); // open the claw
                    //MoveUpwardSlide(0); // retract the lift back down
                })
                .lineToY(-45)
                .build();
        Actions.runBlocking(trajectory0);
        drive.localizer.update();
    }

    private void MoveOutOfTheWay(){
        Action trajectory0 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-48, -48))
                .build();// go near the sub
        Actions.runBlocking(trajectory0);
        drive.localizer.update();
    }

    //------------------------------------------------------------------------------------------------
    // Trajectory Initializers
    //------------------------------------------------------------------------------------------------
    private void InitToBarTrajectory(){
        GetSpec = drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(()->{
                    ArmToPosition(armPosition.SPECIMEN_READY); // Rotates arm behind itself
                    Claw(false);                }) // opens the claw
                .splineTo(new Vector2d(47, -38), Math.toRadians(90.00))
                .lineToY( -60) // reverses to the wall where the specimen is
                .build();

    }

    private void InitPushFirstSamp(){
        PushSpecFirst = drive.actionBuilder(drive.localizer.getPose())
                // Moves past the coloured samples to get into pushing position
                .splineTo(new Vector2d(35.62, -47.98), Math.toRadians(90.00))
                .splineTo(new Vector2d(36.59, -18.40), Math.toRadians(76.35))
                .splineTo(new Vector2d(48, -4.79), Math.toRadians(90.00))
                .build();

    }


}
