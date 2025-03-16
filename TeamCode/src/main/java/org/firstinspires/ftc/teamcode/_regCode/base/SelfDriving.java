package org.firstinspires.ftc.teamcode._regCode.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;

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

    protected enum armPosition{
        UPSTRAIGHT,
        SPECIMEN_READY
    }

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected final double MOVEMENT_POWER = 0.5;
    protected final double TURN_POWER  = 0.3;


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
            hardwareManager.leftFront.setPower(1 - powerMultiplier);
            hardwareManager.leftBack.setPower(1 - powerMultiplier);
            hardwareManager.rightFront.setPower(1 + powerMultiplier);
            hardwareManager.rightBack.setPower(1 + powerMultiplier);
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    protected void timedMove(double milliseconds, double power,boolean forwards){
        ElapsedTime timeElapsed = new ElapsedTime();
        double tempPower = (forwards) ? power : -power;
        while(timeElapsed.milliseconds() < milliseconds){
            hardwareManager.doToAllWheels((wheel) -> wheel.setPower(tempPower));
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
        hardwareManager.leftFront.setPower(direction * 1);
        hardwareManager.rightFront.setPower(direction * -1);
        hardwareManager.leftBack.setPower(direction * -1);
        hardwareManager.rightBack.setPower(direction * 1);

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

        hardwareManager.leftFront.setPower(leftPower);
        hardwareManager.rightFront.setPower(rightPower);
        hardwareManager.leftBack.setPower(leftPower);
        hardwareManager.rightBack.setPower(rightPower);

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
    public void Claw(boolean closed){
        hardwareManager.clawServo.setPosition((closed) ? 0 : 0.15);
    }
    public void Arm(double angle){
        hardwareManager.leftArmServo.setPosition(angle * (1.0 / 360.0));
        hardwareManager.rightArmServo.setPosition(angle * (1.0 / 360.0));
    }
    public void ArmToPosition(armPosition armPosition){
        // Inputs an angle that the arm should be at relative to its starting position
        switch(armPosition){
            case UPSTRAIGHT:
                Arm(70);
                break;
            case SPECIMEN_READY:
                Arm(0);
                break;

        }
    }
    //------------------------------------------------------------------------------------------------
    // Outtake Slide
    //------------------------------------------------------------------------------------------------
    public void MoveUpwardSlide(double RaiseToPercent){
        // Input a decimal of the maximum height of the lift
        // 0 = fully retracted & 1 = fully up

        // limits the value to only be between 0 and 1, representing 0% to 100%
        double percent = Range.clip(RaiseToPercent, 0.0, 1.0);
        double targetCount = percent * 4380;

        // Reset the encoders
        hardwareManager.ResetLiftWheelCount();

        double mainDirection = (targetCount > hardwareManager.liftMotorLeft.getCurrentPosition()) ? 1 : -1;
        hardwareManager.liftMotorLeft.setPower(mainDirection);
        hardwareManager.liftMotorRight.setPower(mainDirection);
        while(opModeIsActive() && targetCount - 15 > hardwareManager.liftMotorLeft.getCurrentPosition() || hardwareManager.liftMotorLeft.getCurrentPosition() > targetCount + 15 ){
            if(mainDirection > 0 && hardwareManager.liftMotorLeft.getCurrentPosition() >=  4380){
                break;
            } else if (mainDirection < 0 && hardwareManager.liftMotorLeft.getCurrentPosition() <=  60) {
                hardwareManager.ResetLiftWheelCount();
                break;
            }
            idle();
        }
        hardwareManager.liftMotorLeft.setPower(0);
        hardwareManager.liftMotorRight.setPower(0);
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        waitForStart();
        runAutonomous();
    }

    protected abstract void runAutonomous();
}
