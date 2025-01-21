package org.firstinspires.ftc.teamcode._regCode.base;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Base class for all Self-Driving scripts, a.k.a Autonomous.
 * Providing all the required tools to do precise movements.
 */
public abstract class SelfDriving extends LinearOpMode {
    protected final double WHEEL_CIRCUMFERENCE = Math.PI * 0.102; // M
    protected final int COUNTS_PER_MOTOR_REVOLUTION = 900;
    protected final double COUNTS_PER_METER =
            COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

    protected HardwareManager hardwareManager;
    protected PIDControl movementPID;

    //------------------------------------------------------------------------------------------------
    // Active Intake
    //------------------------------------------------------------------------------------------------
    protected double COUNTS_LIFT_OUTPUT = 4800; // TODO: must change
    protected double LIFT_INCH_DIFFERENCE = 25.25; // TODO: must change
    protected double COUNTS_PER_INCH =
            COUNTS_LIFT_OUTPUT / LIFT_INCH_DIFFERENCE;
    protected double liftServoAngle = 0;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected final double MOVEMENT_POWER = 0.5;
    protected final double TURN_POWER  = 0.3;

    protected SampleMecanumDrive drive;

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

    //------------------------------------------------------------------------------------------------
    // Movement (Deprecated: unusable)
    //------------------------------------------------------------------------------------------------

    protected void move(double metersDistance) {
        if (!opModeIsActive())
            return;
        movementPID = new PIDControl(5, 0, 5, 0.05);

        hardwareManager.resetWheelCounts();
        double totalCounts = Math.abs(COUNTS_PER_METER * metersDistance);
        hardwareManager.imu.resetYaw();
        double initialAngleTheta = hardwareManager.getCurrentDegreeHeading();
        while (opModeIsActive() && hardwareManager.getAverageWheelCounts() <= totalCounts) {
            double PIDoutput = movementPID.OnUpdatePower(
                    hardwareManager.getCurrentDegreeHeading(),
                    initialAngleTheta
            );

            double powerMultiplier = (PIDoutput/180);
            hardwareManager.frontLeftWheel.setPower(1 - powerMultiplier);
            hardwareManager.backLeftWheel.setPower(1 - powerMultiplier);
            hardwareManager.frontRightWheel.setPower(1 + powerMultiplier);
            hardwareManager.backRightWheel.setPower(1 + powerMultiplier);
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    //------------------------------------------------------------------------------------------------
    // Strafing (Deprecated: unusable)
    //------------------------------------------------------------------------------------------------
    protected void strafe(double metersDistance) {
        if(!opModeIsActive())
            return;

        int direction = (metersDistance > 0) ? 1 : -1;
        hardwareManager.resetWheelCounts();
        hardwareManager.frontLeftWheel.setPower(direction * 1);
        hardwareManager.frontRightWheel.setPower(direction * -1);
        hardwareManager.backLeftWheel.setPower(direction * -1);
        hardwareManager.backRightWheel.setPower(direction * 1);

        double totalCounts = Math.abs(COUNTS_PER_METER * metersDistance);
        while(opModeIsActive() && hardwareManager.getAverageWheelCounts() <= totalCounts){
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    //------------------------------------------------------------------------------------------------
    // Rotation (Deprecated: unusable)
    //------------------------------------------------------------------------------------------------
    protected void rotate(double degreeAngle) {
        if (!opModeIsActive())
            return;

        hardwareManager.imu.resetYaw();
        double initialAngle = hardwareManager.getCurrentDegreeHeading();

        double motorOffset = degreeAngle > 0 ? 1 : -1;
        double leftPower = TURN_POWER * motorOffset;
        double rightPower = TURN_POWER * -motorOffset;

        hardwareManager.frontLeftWheel.setPower(leftPower);
        hardwareManager.frontRightWheel.setPower(rightPower);
        hardwareManager.backLeftWheel.setPower(leftPower);
        hardwareManager.backRightWheel.setPower(rightPower);

        while(opModeIsActive() && hasReachedDesiredAngle(initialAngle, degreeAngle)) {
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    protected boolean hasReachedDesiredAngle(double initialAngle, double turnAngle) {
        double targetAngle = initialAngle - turnAngle;
        double currentAngle = hardwareManager.getCurrentDegreeHeading();

        return turnAngle > 0
                ? currentAngle > targetAngle
                : currentAngle < targetAngle;
    }

    //------------------------------------------------------------------------------------------------
    // Intake
    //------------------------------------------------------------------------------------------------
    public void Claw(boolean open){
        hardwareManager.leftClawServo.setPosition((open) ? 0.25 : 0);
        hardwareManager.rightClawServo.setPosition((open) ? 0 : 0.25);
    }

    public void Arm(double angle){
        double angleToPos = angle * (1.0/360.0);
        double filteredAngle = Range.clip(angleToPos, 0, 0.55);
        hardwareManager.liftServo.setPosition(filteredAngle);
    }

    //------------------------------------------------------------------------------------------------
    // Outtake Slide
    //------------------------------------------------------------------------------------------------
    public void MoveUpwardSlide(double GoToInch){
        ResetLiftWheelCount();

        double mainDirection = (GoToInch > 0) ? 0.5 : -0.5;
        hardwareManager.liftMotorLeft.setPower(mainDirection);

        double totalCounts = Math.abs(GoToInch * COUNTS_PER_INCH);
        while(opModeIsActive() && (double) Math.abs(hardwareManager.liftMotorLeft.getCurrentPosition()) <= totalCounts){
            idle();
        }
        hardwareManager.liftMotorLeft.setPower(0);
    }


    //------------------------------------------------------------------------------------------------
    // Auto Macros
    //------------------------------------------------------------------------------------------------
    public void ScoreHighBasket(){
        Arm(0);
        sleep(1500);
        MoveUpwardSlide(30);
        sleep(1500);
        Arm(120);
        sleep(1500);
        Claw(true);
        sleep(1000);
        Claw(false);
        sleep(1000);
        Arm(0);
        sleep(1500);
        MoveUpwardSlide(-30);
    }

    public void GrabLow(){
        Claw(true);
        sleep(1000);
        Arm(170);
        sleep(2000);
        Claw(false);
        sleep(1000);
        Arm(0);
        sleep(1000);
    }

    protected void ResetLiftWheelCount(){
        hardwareManager.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset motor ticks
        hardwareManager.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Run motor by power
    }

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
        sleep(1000);
        Arm(180);
        TrajectorySequence GoToHangingStation = drive.trajectorySequenceBuilder(prevPose)
                .splineTo(new Vector2d(6.32, -42.70), Math.toRadians(90.49))
                //.lineTo(new Vector2d(6.32, -48.12))
                .build();
        drive.setPoseEstimate(GoToHangingStation.start());
        drive.followTrajectorySequence(GoToHangingStation);
        Arm(160);

    }
    private void HangSpecimenHigh(){

    }

    private void SetServoPosition(){
        while(opModeIsActive()){
            //hardwareManager.liftServo.setPosition(liftServoAngle);
        }
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        SetServoPosition();
        waitForStart();
        runAutonomous();

        for(int i = 0; i < s ;i++){
            HangSpecimenLow(new Pose2d(11.59, -60.34, Math.toRadians(90.00)));
        }
    }

    protected abstract void runAutonomous();
}
