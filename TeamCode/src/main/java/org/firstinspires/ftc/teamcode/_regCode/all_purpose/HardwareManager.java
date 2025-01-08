package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A specific class designed for setting up all of the robot's
 * hardware binding classes, and presiding over groups of them to
 * be able to do synchronized commands.
 */
public class HardwareManager {
    //------------------------------------------------------------------------------------------------
    // Active Intake
    //------------------------------------------------------------------------------------------------

    public final DcMotor liftMotor;
    public final Servo leftLiftServo;
    public final Servo rightLiftServo;
    public final Servo leftClawServo;
    public final Servo rightClawServo;


    //------------------------------------------------------------------------------------------------
    // Wheels
    //------------------------------------------------------------------------------------------------
    public final DcMotor frontLeftWheel;
    public final DcMotor frontRightWheel;
    public final DcMotor backLeftWheel;
    public final DcMotor backRightWheel;


    /**
     * let `n` be return value;
     *      n < 0 = Motors went reversed.
     *      n > 0 = Motors went forward.
     */

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
        frontLeftWheel = hardwareMap.dcMotor.get("FrontLeftM");
        frontRightWheel = hardwareMap.dcMotor.get("FrontRightM");
        backLeftWheel = hardwareMap.dcMotor.get("BackLeftM");
        backRightWheel = hardwareMap.dcMotor.get("BackRightM");

        // Active Intake
        //intakeWheel = hardwareMap.dcMotor.get("IntakeWheel");
        //intakeServo = hardwareMap.servo.get("IntakeServo");
        //intakeSlide = hardwareMap.dcMotor.get("IntakeSlide");

        // Lift Control
        liftMotor = hardwareMap.dcMotor.get("LiftMotor");

        rightLiftServo = hardwareMap.servo.get("RightLiftServo");
        leftLiftServo = hardwareMap.servo.get("LeftLiftServo");

        leftLiftServo.setDirection(Servo.Direction.REVERSE);

        leftClawServo = hardwareMap.servo.get("LeftClawServo");
        rightClawServo = hardwareMap.servo.get("RightClawServo");

        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        doToAllWheels((wheel) -> wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));


        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
