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
    public final DcMotor liftMotorLeft;
    public final DcMotor liftMotorRight;
    public final Servo leftClawServo;
    public final Servo rightClawServo;

    public final Servo clawServo;

    public final Servo wristServo;

    //private final Servo armServoP;
   //public final ReverseServoWrapper armServo;
    public final Servo leftArmServo;
    public final ReverseServoWrapper rightArmServo;

    private final Servo clawRotationServoP;
    public final ReverseServoWrapper clawRotationServo;

    public void ResetLiftWheelCount(){
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset motor ticks
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Run motor by power
    }

    //------------------------------------------------------------------------------------------------
    // Wheels
    //------------------------------------------------------------------------------------------------
    public final DcMotor frontLeftWheel;
    public final DcMotor frontRightWheel;
    public final DcMotor backLeftWheel;
    public final DcMotor backRightWheel;

    public double getAverageWheelCounts() {
        return (Math.abs(frontLeftWheel.getCurrentPosition())) +
                Math.abs(frontRightWheel.getCurrentPosition()) +
                Math.abs(backLeftWheel.getCurrentPosition()) +
                Math.abs(backRightWheel.getCurrentPosition()) / 4.0;
    }

    public void resetWheelCounts() {
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        doToAllWheels((wheel) -> wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }


    /**
     * For each wheel motor run that specific callback.
     * Converting this:
     *      frontLeftWheel.doTheSameCommand();
     *      frontRightWheel.doTheSameCommand();
     *      backLeftWheel.doTheSameCommand();
     *      backRightWheel.doTheSameCommand();
     *
     * To this:
     *      doToAllWheels((wheel) -> wheel.doTheSameCommand());
     */
    public void doToAllWheels(WheelCallback callback) {
        callback.run(frontLeftWheel);
        callback.run(frontRightWheel);
        callback.run(backLeftWheel);
        callback.run(backRightWheel);
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
        frontLeftWheel = hardwareMap.dcMotor.get("FrontLeftM");
        frontRightWheel = hardwareMap.dcMotor.get("FrontRightM");
        backLeftWheel = hardwareMap.dcMotor.get("BackLeftM");
        backRightWheel = hardwareMap.dcMotor.get("BackRightM");

        //Set which way it will rotate, make sure to include hardware specific arguements (DcMotorSimple).
        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // Arm and Claw
       // armServoP = hardwareMap.servo.get("LeftArmS");
       // armServo = new ReverseServoWrapper(armServoP);

        leftArmServo = hardwareMap.servo.get("LeftArmS");
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        Servo h = hardwareMap.servo.get("RightArmS");
        h.setDirection(Servo.Direction.REVERSE);
        rightArmServo = new ReverseServoWrapper(h);

        //leftArmServo.setDirection(Servo.Direction.REVERSE);
        //rightArmServo.setDirection(Servo.Direction.REVERSE);

        leftClawServo = hardwareMap.servo.get("LeftClawServo");
        rightClawServo = hardwareMap.servo.get("RightClawServo");
        clawRotationServoP = hardwareMap.servo.get("ClawRotationServo");
        clawServo = hardwareMap.servo.get("ClawServo");
        wristServo = hardwareMap.servo.get("WristServo");

        clawRotationServo = new ReverseServoWrapper(clawRotationServoP);

        //clawRotationServo.setDirection(Servo.Direction.FORWARD);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
