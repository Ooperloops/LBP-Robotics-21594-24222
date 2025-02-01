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
    //------------------------------------------------------------------------------------------------
    // Wheel power values
    //------------------------------------------------------------------------------------------------
    //Preferably initialize your variables here rather than just declaring.
    protected double frontLeftWheelP = 0;
    protected double frontRightWheelP = 0;
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;
    //------------------------------------------------------------------------------------------------
    // Lift power values
    //------------------------------------------------------------------------------------------------
    // Variables for lift motor power
    protected double leftLiftP = 0;
    protected double rightLiftP = 0;

    // Speed update delta variable
    protected double spdDelta = 0.01; // In seconds

    //Initialize PID controller
    private PIDControl pidControl = new PIDControl(3000, 0.00, 0.5, spdDelta);

    // Variables to store previous lift motor position
    protected double prevLeftMotorPos = 0;
    protected double prevRightMotorPos = 0;

    // Time elapsed class for correction
    protected ElapsedTime timeElapsed;
    protected double prevTime = 1;
    protected double powerPerSpeed = 1.0/12000.0;

    //------------------------------------------------------------------------------------------------
    // Lift servo position values
    //------------------------------------------------------------------------------------------------
    protected double liftServoPosition = 0.04;
    protected double rightClawServoPosition = 0;
    protected double leftClawServoPosition = 0.25;
    protected double increment = 0.0027;
    boolean initActive;




    //------------------------------------------------------------------------------------------------
    // Config
    //------------------------------------------------------------------------------------------------

    protected double MOTOR_UPPER_POWER_LIMIT = 1;
    protected double MOTOR_LOWER_POWER_LIMIT = -1;
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
        frontLeftWheelP  = drive + strafe + rotate;
        frontRightWheelP = drive - strafe - rotate;
        backLeftWheelP   = drive - strafe + rotate;
        backRightWheelP  = drive + strafe - rotate;
    }
    public void liftControlPID(boolean isPlayerOne) {
        // If isPlayerOne, then set the active gamepad to controller 1
        Gamepad activeGamepad = (isPlayerOne) ? gamepad1 : gamepad2;

        //-----------------------------------
        // Regular lift control
        //-----------------------------------

        if(-activeGamepad.left_stick_y > 0 && hardwareManager.liftMotorLeft.getCurrentPosition() >=  5650){
            leftLiftP = 0;
        } else if (-activeGamepad.left_stick_y < 0 && hardwareManager.liftMotorLeft.getCurrentPosition() <=  25) {
            leftLiftP = 0;
            hardwareManager.ResetLiftWheelCount();
        }else{
            leftLiftP = -activeGamepad.left_stick_y;
        }
        //-----------------------------------
        // Set PID (Deprecated)
        //-----------------------------------
        // convert time elapsed in milliseconds to seconds
        //double elapsedTImeToSeconds = timeElapsed.milliseconds() / 1000;


        /*
        if(elapsedTImeToSeconds - prevTime >= spdDelta) { // if spdDelta amount has pass then start PID correction.
            telemetry.addData("Time: ", elapsedTImeToSeconds);

            // Get speed of left and right motor
            double currentLeftMotorPos = hardwareManager.liftMotorLeft.getCurrentPosition();
            double currentRightMotorPos = hardwareManager.liftMotorRight.getCurrentPosition();

            //Get the speed of both motors
            double leftMotorSpeed = // Get speed...
                    (currentLeftMotorPos - prevLeftMotorPos) / spdDelta; // ...through difference in position
            double rightMotorSpeed = // Repeat for rightMotorSpeed
                    (currentRightMotorPos - prevRightMotorPos) / spdDelta;

            // Add PID correction to motor power
            leftLiftP += powerPerSpeed * pidControl.OnUpdatePower(Shrink(leftMotorSpeed) /* current speed , Shrink(rightMotorSpeed) /* target speed );

            telemetry.addData("Left Motor Speed: ", (double)leftMotorSpeed)
                    .addData("Right Motor Speed: ", (double)rightMotorSpeed)
                    .addData("Right Motor Ticks", hardwareManager.liftMotorRight.getCurrentPosition());

            // Set previous position values for next iteration of the Set PID section
            prevLeftMotorPos = currentLeftMotorPos;
            prevRightMotorPos = currentRightMotorPos;

            prevTime = elapsedTImeToSeconds;
        }
        //-----------------------------------
        */
        //Set the power of both motors
        hardwareManager.liftMotorLeft.setPower(leftLiftP);
        hardwareManager.liftMotorRight.setPower(rightLiftP);



    }

    public void armServos () {
        liftServoPosition = liftServoPosition + (increment * -gamepad2.right_stick_y);
        liftServoPosition = Range.clip(liftServoPosition, 0.02777777777, 0.68);
        /*
        if(!initActive) {

            if (liftServoPosition == 0 || liftServoPosition <= 0.075) {
                hardwareManager.clawRotationServo.setPosition(0);
            } else if (liftServoPosition >= 0.175) {
                hardwareManager.clawRotationServo.setPosition(0.27777777777);
            } else if (liftServoPosition >= 0.45) {
                hardwareManager.clawRotationServo.setPosition(0.5);
            } else if (liftServoPosition < 0.45) {
                hardwareManager.clawRotationServo.setPosition(0.27777777777);
            } else if (liftServoPosition < 0.175) {
                hardwareManager.clawRotationServo.setPosition(0);
            }
        }*/
        if(liftServoPosition >= 0.47222222222){
            hardwareManager.clawRotationServo.setPosition(0.30555555555);
        }else if(liftServoPosition >= 0.14){
            hardwareManager.clawRotationServo.setPosition(0.25);
        } else {
            hardwareManager.clawRotationServo.setPosition(0.65);
        }

        if(gamepad2.y) {liftServoPosition = 0.285;}

    }

    public void clawControls(){
        /*
            The following are macros for specific positions for the claw to be in
         */

        // All of these input values are placed in a single if-statement to avoid
        // conflict with multiple button presses
        if(gamepad2.left_trigger > 0.3){ // Close right claw piece
            leftClawServoPosition = 0.25;
        }else if(gamepad2.right_trigger > 0.3){ // Close left claw piece
            rightClawServoPosition = 0;
        } else if(gamepad2.b ){ // Fully close the claw
            rightClawServoPosition = 0;
            leftClawServoPosition = 0.25;
        } else if(gamepad2.x){ // Fully open the claw
            rightClawServoPosition = 0.25;
            leftClawServoPosition = 0;
        }

        // Manual control for the claw
        if(gamepad2.dpad_left){ // if left Dpad is pressed
            rightClawServoPosition = Range.clip(rightClawServoPosition - increment, 0, 0.25);
            leftClawServoPosition = Range.clip(leftClawServoPosition + increment, 0, 0.25);
        }else if(gamepad2.dpad_right){
            rightClawServoPosition = Range.clip(rightClawServoPosition + increment, 0, 0.25);
            leftClawServoPosition = Range.clip(leftClawServoPosition - increment, 0, 0.25);
        }
        hardwareManager.rightClawServo.setPosition(rightClawServoPosition);
        hardwareManager.leftClawServo.setPosition(leftClawServoPosition);
    }

    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);

        initActive = true;

        //-------------------------------------------------
        // Set the default position of the claw and wrist
        //-------------------------------------------------

        // Close the claw
        hardwareManager.rightClawServo.setPosition(0);
        hardwareManager.leftClawServo.setPosition(0.25);

        // Set wrist all the way back
        hardwareManager.clawRotationServo.setPosition(0);

        //Move arm all the way back
        hardwareManager.armServo.setPosition(0.0);

        //Init ElapsedTime for PID
        timeElapsed = new ElapsedTime();
    }

    public void setHardwarePower() {

        // Left trigger lowers maximum and minimum power (slow toggle)
        MOTOR_UPPER_POWER_LIMIT = (gamepad1.left_trigger > 0) ? 0.5 : 1;
        MOTOR_LOWER_POWER_LIMIT = (gamepad1.left_trigger > 0) ? -0.5 : -1;

        // Limit motor powers of all wheels
        hardwareManager.frontLeftWheel.setPower(limitMotorPower(frontLeftWheelP));
        hardwareManager.frontRightWheel.setPower(limitMotorPower(frontRightWheelP));
        hardwareManager.backLeftWheel.setPower(limitMotorPower(backLeftWheelP));
        hardwareManager.backRightWheel.setPower(limitMotorPower(backRightWheelP));

        hardwareManager.armServo.setPosition(liftServoPosition);
    }

    protected double limitMotorPower(double input){
        // Limits the DcMotor output power within a certain interval
        return Range.clip(input, MOTOR_LOWER_POWER_LIMIT, MOTOR_UPPER_POWER_LIMIT);
    }

    protected double limitServoPower(double input) {
        // Limits the Servo output power with a certain interval
        return Range.clip(input, SERVO_LOWER_POWER_LIMIT, SERVO_UPPER_POWER_LIMIT);
    }
}
