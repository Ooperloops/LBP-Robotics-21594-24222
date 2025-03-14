package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A specific class designed for setting up all of the robot's
 * hardware binding classes, and presiding over groups of them to
 * be able to do synchronized commands.
 */
public class HardwareManager {

    //------------------------------------------------------------------------------------------------
    // Computer Vision
    //------------------------------------------------------------------------------------------------
    public final WebcamName camera;
    //------------------------------------------------------------------------------------------------
    // Active Intake
    //------------------------------------------------------------------------------------------------
    //Initialize/Declaring variables.  Make sure to put which hardware you are using.

    // Lift
    public final DcMotor liftMotorLeft;
    public final DcMotor liftMotorRight;
    public void ResetLiftWheelCount(){
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset motor ticks
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Run motor by power
    }

    // Claw
    public final Servo clawServo;
    public final Servo angleClawServo;
    public final Servo horizontalClawServo;

    public void angleToClawServoAngle(double angle){
        horizontalClawServo.setPosition(angle * 1.0/360.0);
    }

    // Arm
    public final Servo leftArmServo;
    public final ReverseServoWrapper rightArmServo;

    // Ascent
    public final DcMotor leftAscentMotor;
    public final DcMotor rightAscentMotor;

    //------------------------------------------------------------------------------------------------
    // Wheels
    //------------------------------------------------------------------------------------------------
    public final DcMotor leftFront;
    public final DcMotor rightFront;
    public final DcMotor leftBack;
    public final DcMotor rightBack;

    public double getAverageWheelCounts() {
        return (Math.abs(leftFront.getCurrentPosition())) +
                Math.abs(rightFront.getCurrentPosition()) +
                Math.abs(leftBack.getCurrentPosition()) +
                Math.abs(rightBack.getCurrentPosition()) / 4.0;
    }

    public void resetWheelCounts() {
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }


    /**
     * For each wheel motor run that specific callback.
     * Converting this:
     *      leftFront.doTheSameCommand();
     *      rightFront.doTheSameCommand();
     *      leftBack.doTheSameCommand();
     *      rightBack.doTheSameCommand();
     *
     * To this:
     *      doToAllWheels((wheel) -> wheel.doTheSameCommand());
     */
    public void doToAllWheels(WheelCallback callback) {
        callback.run(leftFront);
        callback.run(rightFront);
        callback.run(leftBack);
        callback.run(rightBack);
    }

    public interface WheelCallback {
        void run(DcMotor motor);
    }

    //------------------------------------------------------------------------------------------------
    // Sensors
    //------------------------------------------------------------------------------------------------
    public IMU imu;
    /**
     * Returns a normalized robot yaw orientation in Degrees (°)
     *
     *               <- FORWARD ->
     *                     0
     * LEFT  -90           +         90 RIGHT
     *                 -180/180
     *                 BACKWARD
     */

    public double getCurrentDegreeHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public HardwareManager(HardwareMap hardwareMap) {

        // Wheels
        //The green text is what you want to write on the DriveHub (the tablet thing), it can be anything.
        leftFront = hardwareMap.dcMotor.get("FrontLeftM");
        rightFront = hardwareMap.dcMotor.get("FrontRightM");
        leftBack = hardwareMap.dcMotor.get("BackLeftM");
        rightBack = hardwareMap.dcMotor.get("BackRightM");

        //Set which way it will rotate, make sure to include hardware specific arguements (DcMotorSimple).
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //This brake allows a motor to hold its position when it is not moving.
        doToAllWheels((wheel) -> wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        // Lift Control
        liftMotorLeft = hardwareMap.dcMotor.get("LeftLiftM");
        liftMotorRight = hardwareMap.dcMotor.get("RightLiftM");

        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder programming for PID on the lift.
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Arm
        leftArmServo = hardwareMap.servo.get("LeftArmS");
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        Servo h = hardwareMap.servo.get("RightArmS");
        h.setDirection(Servo.Direction.REVERSE);
        rightArmServo = new ReverseServoWrapper(h);

        // Claw
        clawServo = hardwareMap.servo.get("clawS");
        clawServo.setDirection(Servo.Direction.REVERSE);
        angleClawServo = hardwareMap.servo.get("angClawS");
        horizontalClawServo = hardwareMap.servo.get("horzClawS");

        // Ascent
        leftAscentMotor = hardwareMap.dcMotor.get("ascLeftM");
        rightAscentMotor = hardwareMap.dcMotor.get("ascRightM");

        rightAscentMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftAscentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAscentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Camera
        camera = hardwareMap.get(WebcamName.class, "Camera");

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }
}
