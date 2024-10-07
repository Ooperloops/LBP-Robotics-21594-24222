package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.all_purpose.HardwareManager;

/**
 * Base class for all human-operated scripts, a.k.a TeleOp.
 * Any inherited classes manipulates the given protected power
 * values of each motor and servos, and then calls `setHardwarePower`
 * which ensures that the values obeys the UPPER and LOWER limits,
 * before sending them to each hardware binding classes.
 *
 * (USER_INPUT) -> (Class extends HumanOperated) -> (Base Class HumanOperated) -> (Hardware Manager)
 *      |                   |                             |                              |
 *     \/                   |                             |                              `-> Magic Class that converts Java value to actual
 *   GamePad1 or            |                             |                                  voltage to be used by DcMotors and Servos.
 *   GamePad2               |                             |
 *                          |                             `> Ensures the desired power setting is within min and max
 *                          |                                 before sending it to the actual hardware binding class.
 *                         \/
 *                    Interprets the human control into whatever
 *                    control schema we decide. E.g Tank Control
 *                    Split Control, etc..
 */
public abstract class HumanOperated extends OpMode {
    protected HardwareManager hardwareManager;

    protected double frontLeftWheelP = 0;
    protected double frontRightWheelP = 0;
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------

    protected final double MOTOR_UPPER_POWER_LIMIT = 1;
    protected final double MOTOR_LOWER_POWER_LIMIT = -1;
    protected final double SERVO_UPPER_POWER_LIMIT = 0.8; // VEX Servos Actual Limitation
    protected final double SERVO_LOWER_POWER_LIMIT = -0.8; // VEX Servos Actual Limitation

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------

    protected void useDefaultMovementControls() {
        // Allow for forward / backward movement command
        // to be receive from left and right joystick.


        /** [HOW THIS WORKS]
         * DcMotors need a power input between (-1.00 to 1.00)
         * This can be done by calling the .setPower(); method on a DcMotor variable
         * -----
         * Each of the joysticks' have two axes (x and y)
         * if a the left joystick of a gamepad is moved up then gamepad#.left_stick_y is positive
         * if a the left joystick of a gamepad is moved down then gamepad#.left_stick_y is negative
         * etc.
         * -----
         * Forward and Backward drive is done by setting the power of all the wheels as the value
         * of the y-axis value of the gamepad's left joystick
         *
         */
        double drive = gamepad1.left_stick_y;

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        frontLeftWheelP  = drive + strafe - rotate;
        frontRightWheelP = drive - strafe + rotate;
        backLeftWheelP   = drive - strafe - rotate;
        backRightWheelP  = drive + strafe + rotate;
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }

    public void setHardwarePower() {
        hardwareManager.frontLeftWheel.setPower(limitMotorPower(frontLeftWheelP));
        hardwareManager.frontRightWheel.setPower(limitMotorPower(frontRightWheelP));
        hardwareManager.backLeftWheel.setPower(limitMotorPower(backLeftWheelP));
        hardwareManager.backRightWheel.setPower(limitMotorPower(backRightWheelP));
    }

    protected double limitMotorPower(double input){
        // Limits the DcMotor output power with a certain interval
        return Range.clip(input, MOTOR_LOWER_POWER_LIMIT, MOTOR_UPPER_POWER_LIMIT);
    }

    protected double limitServoPower(double input) {
        // Limits the Servo output power with a certain interval
        return Range.clip(input, SERVO_LOWER_POWER_LIMIT, SERVO_UPPER_POWER_LIMIT);
    }
}
