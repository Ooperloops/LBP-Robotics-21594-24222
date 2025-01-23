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
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;

    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------

    protected final double MOTOR_UPPER_POWER_LIMIT = 0.5;
    protected final double MOTOR_LOWER_POWER_LIMIT = -0.5;
    protected final double SERVO_UPPER_POWER_LIMIT = 0.5; // VEX Servos Actual Limitation
    protected final double SERVO_LOWER_POWER_LIMIT = -0.5; // VEX Servos Actual Limitation

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------

    public void intakeArmControl() {
        hardwareManager.intakeArmRight.setPower(limitMotorPower(gamepad2.left_stick_y));
        hardwareManager.intakeArmLeft.setPower(limitMotorPower(gamepad2.left_stick_y));

        if(gamepad2.right_trigger > 0){
            hardwareManager.extenderArm.setPower(1);
        } else if(gamepad2.left_trigger > 0){
            hardwareManager.extenderArm.setPower(-1);
        } else {
            hardwareManager.extenderArm.setPower(0);
        }

        if(gamepad2.a){
            hardwareManager.rightClawServo.setPosition(0.150);
            hardwareManager.leftClawServo.setPosition(0);
        } else if (gamepad2.b) {
            hardwareManager.rightClawServo.setPosition(0);
            hardwareManager.leftClawServo.setPosition(0.150);
        }
    }

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
        double drive = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        backLeftWheelP   = drive + rotate;
        backRightWheelP  = drive - rotate;
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
        hardwareManager.leftClawServo.setPosition(0);
        hardwareManager.rightClawServo.setPosition(0.15);
    }

    public void setHardwarePower() {
        hardwareManager.backLeftWheel.setPower(backLeftWheelP);
        hardwareManager.backRightWheel.setPower(backRightWheelP);

    }

    protected double limitMotorPower(double input){
        return Range.clip(input, MOTOR_LOWER_POWER_LIMIT, MOTOR_UPPER_POWER_LIMIT);
    }

    protected double limitServoPower(double input) {
        return Range.clip(input, SERVO_LOWER_POWER_LIMIT, SERVO_UPPER_POWER_LIMIT);
    }
}
