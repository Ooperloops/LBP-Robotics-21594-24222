package org.firstinspires.ftc.teamcode._regCode.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;

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
    protected boolean slowToggle = false;
    //------------------------------------------------------------------------------------------------
    // Wheel power values
    //------------------------------------------------------------------------------------------------
    //Preferably initialize your variables here rather than just declaring.
    protected double frontLeftWheelP = 0;
    protected double frontRightWheelP = 0;
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;
    //------------------------------------------------------------------------------------------------
    // Lift Variable Values
    //------------------------------------------------------------------------------------------------
    // Variables for lift motor power
    protected double liftP = 0;

    //------------------------------------------------------------------------------------------------
    // Lift servo position values
    //------------------------------------------------------------------------------------------------
    protected double increment = 0.0027;
    private double ArmServoPos = 0;

    // Use if were incrementing positions for the two servos on the wrist
    private double HorzClawPos = 0;
    private double AngClawPos = 0;
    private double conF = (1.0/360.0);




    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------

    protected double MOTOR_UPPER_POWER_LIMIT = 1;
    protected double MOTOR_LOWER_POWER_LIMIT = -1;
    protected final double SERVO_UPPER_POWER_LIMIT = 0.8; // VEX Servos Actual Limitation
    protected final double SERVO_LOWER_POWER_LIMIT = -0.8; // VEX Servos Actual Limitation
    protected double MOTOR_SHRINK_MULTIPLIER = 1;

    //------------------------------------------------------------------------------------------------
    // Defaults
    //------------------------------------------------------------------------------------------------

    protected void useDefaultMovementControls() {
        // Allow for forward / backward movement command
        // to be receive from left and right joystick.


        /** [HOW THIS WORKS]
         * DcMotors need a power input between (-1.00 to 1.00)
         * This can be done by calling the .setPower(); method on a DcMotor variable
         =      * -----
         * Each of the joysticks' have two axes (x and y)
         * if a the left joystick of a gamepad is moved up then gamepad#.left_stick_y is positive
         * if a the left joystick of a gamepad is moved down then gamepad#.left_stick_y is negative
         * etc.
         * -----
         * Forward and Backward drive is done by setting the power of all the wheels as the value
         * of the y-axis value of the gamepad's left joystick
         *
         */
        double drive = (-gamepad1.left_stick_y != 0)
                ? -gamepad1.left_stick_y
                : -gamepad1.right_stick_y;

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        /* The plus and minus signs refer to the direction you must move the stick in order for
        whatever you have programmed to move. This is better than reversing direction in
        HardwareManager
        */
        frontLeftWheelP  = - drive - strafe - rotate;
        frontRightWheelP = - drive + strafe + rotate;
        backLeftWheelP   = - drive + strafe - rotate;
        backRightWheelP  = - drive - strafe + rotate;
    }

    protected void zeroAllServos(){
        hardwareManager.leftArmServo.setPosition(ArmServoPos);
        hardwareManager.rightArmServo.setPosition(ArmServoPos);
        hardwareManager.horizontalClawServo.setPosition(HorzClawPos);
        hardwareManager.clawServo.setPosition(0);
        hardwareManager.angleClawServo.setPosition(AngClawPos);
    }

    public void setArmPosition(){
        hardwareManager.leftArmServo.setPosition(ArmServoPos);
        hardwareManager.rightArmServo.setPosition(ArmServoPos);
    }

    public void armControls(){
            ArmServoPos =
                    Range.clip(ArmServoPos + (-gamepad2.right_stick_y * increment), 0, 0.65);

    }


    public void clawControls(){
        if (gamepad2.a) {
            ArmServoPos = 0;
            AngClawPos =0;
            HorzClawPos = 0.42;
        }else if(gamepad2.x){
            ArmServoPos = 0.1307;
            AngClawPos =0;
            HorzClawPos = 0.42;
        }else if(gamepad2.y) {
            ArmServoPos = 0.2607;
            AngClawPos =0.6;
            HorzClawPos = 0.42;
        }else if (gamepad2.b){
            ArmServoPos = 0.5701;
            AngClawPos =0;
            HorzClawPos = 0.065;
        }

        // Horizontal Wrist Servo Control

        if(gamepad2.dpad_left){
            HorzClawPos = Range.clip(HorzClawPos + 0.005, 0, 0.42);
        } else if (gamepad2.dpad_right) {
            HorzClawPos = Range.clip(HorzClawPos - 0.005, 0, 0.42);
        }


        // Rotational Wrist Servo Control
        if(gamepad2.right_bumper){
            AngClawPos = Range.clip(AngClawPos + increment, 0, 0.6);
        } else if (gamepad2.left_bumper){
            AngClawPos = Range.clip(AngClawPos - increment, 0, 0.6);
        }

        // Claw Control
        if(gamepad2.right_trigger > 0){ // Open
            hardwareManager.clawServo.setPosition(0.15);
        } else if (gamepad2.left_trigger > 0){ // Closed
            hardwareManager.clawServo.setPosition(0);
        }
        hardwareManager.horizontalClawServo.setPosition(HorzClawPos);
        hardwareManager.angleClawServo.setPosition(AngClawPos);
    }

    public void liftControl(){
        if(-gamepad2.left_stick_y > 0 && hardwareManager.liftMotorLeft.getCurrentPosition() >= 4250){
            // If lift ticks surpass max...
            liftP = 0; // ...force stop the motors
        } else if (-gamepad2.left_stick_y < 0 && hardwareManager.liftMotorLeft.getCurrentPosition() <= 20) {
            // If lift ticks surpass min...
            liftP = 0; // ...force stop the motors
            hardwareManager.ResetLiftWheelCount();
        } else {
            liftP = -gamepad2.left_stick_y;
        }
    }

    public void ServoTelemetry(){
        telemetry.addData("ARM POSITION", ArmServoPos)
                .addData("VERTICAL POSITION", HorzClawPos)
                .addData("ANGULAR POSITION", AngClawPos);
        telemetry.update();
    }

    public void setLiftPower(){
        hardwareManager.liftMotorLeft.setPower(liftP);
        hardwareManager.liftMotorRight.setPower(liftP);
    }
    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);

        zeroAllServos();
    }

    public void setHardwarePower() {

        // Left bumper multiplies motor power by a small decimal (slow toggle)
        if(gamepad1.a && !slowToggle){
            MOTOR_SHRINK_MULTIPLIER = 0.4;
            slowToggle = true;
        } else if (gamepad1.a && slowToggle){
            MOTOR_SHRINK_MULTIPLIER = 1;
            slowToggle = false;
        }
        //MOTOR_SHRINK_MULTIPLIER = (gamepad1.left_bumper) ? 0.4 : 1;

        // Limit motor powers of all wheels
        hardwareManager.leftFront.setPower(shrinkMotorPower(frontLeftWheelP));
        hardwareManager.rightFront.setPower(shrinkMotorPower(frontRightWheelP));
        hardwareManager.leftBack.setPower(shrinkMotorPower(backLeftWheelP));
        hardwareManager.rightBack.setPower(shrinkMotorPower(backRightWheelP));
    }

    protected double limitMotorPower(double input){
        // Limits the DcMotor output power within a certain interval
        return Range.clip(input, MOTOR_LOWER_POWER_LIMIT, MOTOR_UPPER_POWER_LIMIT);
    }

    protected double shrinkMotorPower(double input){
        return MOTOR_SHRINK_MULTIPLIER * input;
    }

    protected double limitServoPower(double input) {
        // Limits the Servo output power with a certain interval
        return Range.clip(input, SERVO_LOWER_POWER_LIMIT, SERVO_UPPER_POWER_LIMIT);
    }
}