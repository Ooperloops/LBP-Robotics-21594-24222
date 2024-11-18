package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;

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
    protected PIDAutoMove movementPID;

    //------------------------------------------------------------------------------------------------
    // Active Intake Slide
    //------------------------------------------------------------------------------------------------
    protected final int COUNTS_PER_ACTIVE_INTAKE_MOTOR_REVOLUTION = 480;
    protected final double ACTIVE_INTAKE_WHEEL_CIRCUMFERENCE = Math.PI; // Inches
    protected final double COUNTS_PER_METER_ACTIN_SLIDE =
            COUNTS_PER_ACTIVE_INTAKE_MOTOR_REVOLUTION / ACTIVE_INTAKE_WHEEL_CIRCUMFERENCE;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------
    protected final double MOVEMENT_POWER = 0.5;
    protected final double TURN_POWER  = 0.3;

    //------------------------------------------------------------------------------------------------
    // Movement
    //------------------------------------------------------------------------------------------------

    protected void move(double metersDistance) {
        if (!opModeIsActive())
            return;
        movementPID = new PIDAutoMove(5, 1, 5, 0.05);

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
            hardwareManager.frontLeftWheel.setPower(1*(1 - powerMultiplier));
            hardwareManager.backLeftWheel.setPower(1*(1 - powerMultiplier));
            hardwareManager.frontRightWheel.setPower(1*(1 + powerMultiplier));
            hardwareManager.backRightWheel.setPower(1*(1 + powerMultiplier));
            idle();
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    //------------------------------------------------------------------------------------------------
    // Strafing
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
            telemetry.addLine().addData("Average Wheel Counts", hardwareManager.getAverageWheelCounts());
        }

        hardwareManager.doToAllWheels((wheel) -> wheel.setPower(0));
    }

    //------------------------------------------------------------------------------------------------
    // Rotation
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
    // Active Intake
    //------------------------------------------------------------------------------------------------
    public void MoveSlide(double slideTravelDistance){
        //hardwareManager.intakeSlide.
        while(opModeIsActive() && hardwareManager.intakeSlide.getCurrentPosition() == 1){

        }
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
