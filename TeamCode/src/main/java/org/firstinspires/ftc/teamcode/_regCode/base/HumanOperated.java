package org.firstinspires.ftc.teamcode._regCode.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    protected double frontLeftWheelP = 0;
    protected double frontRightWheelP = 0;
    protected double backLeftWheelP = 0;
    protected double backRightWheelP = 0;

    //protected double activeIntakeServoPosition = 0;
    protected double liftServoPosition = 0;
    //protected double leftLiftServoPosition = 0;
    protected double rightClawServoPosition = 0;
    protected double leftClawServoPosition = 0.25;
    protected double increment = 0.0027;

    /*

    double leftPower = 0;  // Power for left motor
    double rightPower = 0; // Power for right motor

    // PID constants
    double kP = 0.1;  // Proportional gain
    double kI = 0.01; // Integral gain
    double kD = 0.01; // Derivative gain

    // PID state variables
    double prevError = 0;
    double integral = 0;

    long lastTime = System.currentTimeMillis();

    */

    boolean initActive;

    // PID controller variables
    private double kp = 1.0; // Proportional coefficient
    private double ki = 0.0; // Integral coefficient
    private double kd = 0.0; // Derivative coefficient
    private double previousError = 0.0;
    private double integral = 0.0;

    // Motor encoders
    private int encoderMotor1 = 0;
    private int encoderMotor2 = 0;

    // Update motor speed

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

        /**
         *
         *   (Front of Robot)
         *       ^
         *       |
         *  0|      |0
         *   |      |
         *   |------|
         *   |      |
         *  0--------0
         */
        double drive = (-gamepad1.left_stick_y != 0)
                ? -gamepad1.left_stick_y
                : -gamepad1.right_stick_y;

        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        frontLeftWheelP  = drive + strafe + rotate;
        frontRightWheelP = drive - strafe - rotate;
        backLeftWheelP   = drive - strafe + rotate;
        backRightWheelP  = drive + strafe - rotate;
    }

    //@Override
    public void liftControlPIDCode(double targetPower) {
        //Gamepad controller = (isPlayerOne) ? gamepad1 : gamepad2;
        /*

        // Get gamepad input for lift control
        double userPower = controller.right_stick_y;

        // Get encoder positions
        int leftPosition = hardwareManager.liftMotorLeft.getCurrentPosition();
        int rightPosition = hardwareManager.liftMotorRight.getCurrentPosition();

        // Calculate the error (difference in encoder positions)
        double error = leftPosition - rightPosition;

        // Get current time and calculate delta time
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds

        // PID calculations
        integral += error * deltaTime;
        double derivative = (error - prevError) / deltaTime;

        double correction = kP * error + kI * integral + kD * derivative;

        // Adjust power for each motor
        leftPower = userPower - correction;  // Reduce power to correct overspeed
        rightPower = userPower + correction; // Increase power to catch up

        // Limit motor power to valid range [-1, 1]
        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        // Set motor power
        hardwareManager.liftMotorLeft.setPower(leftPower);
        hardwareManager.liftMotorRight.setPower(rightPower);

        // Update previous error and time
        prevError = error;
        lastTime = currentTime;

        // Telemetry for debugging
        telemetry.addData("User Power", userPower);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Error", error);
        telemetry.update();

         */
        double error = encoderMotor1 - encoderMotor2; // Speed difference
        integral += error;
        double derivative = error - previousError;
        double correction = kp * error + ki * integral + kd * derivative;
        previousError = error;

        // Apply power with correction
        hardwareManager.liftMotorLeft.setPower(targetPower - correction);
        hardwareManager.liftMotorRight.setPower(targetPower + correction);
    }

    public void liftControlPID() {
        double targetPower = gamepad2.right_stick_y;
        if (Math.abs(targetPower) > 0.1) { // Dead zone threshold
            liftControlPIDCode(targetPower);
        } else {
            hardwareManager.liftMotorLeft.setPower(0);
            hardwareManager.liftMotorRight.setPower(0);
        }
    }

    public void armServos () {
        /*
        hardwareManager.liftMotorLeft.setPower(gamepad2.right_stick_y);
        if(gamepad2.left_stick_y > 0){
            rightLiftServoPosition = Range.clip(rightLiftServoPosition + increment, 0, 0.55);
            leftLiftServoPosition = Range.clip(leftLiftServoPosition + increment, 0, 0.55);
        }else if(gamepad2.left_stick_y < 0){
            rightLiftServoPosition = Range.clip(rightLiftServoPosition - increment, 0, 0.55);
            leftLiftServoPosition = Range.clip(leftLiftServoPosition - increment, 0, 0.55);
        }

        hardwareManager.rightLiftServo.setPosition(rightLiftServoPosition);
        hardwareManager.leftLiftServo.setPosition(leftLiftServoPosition);
         */
        if(gamepad2.left_stick_y > 0){
            liftServoPosition = Range.clip(liftServoPosition + increment, 0, 0.55);
            if (initActive == true){
                initActive = false;
            }
        }else if(gamepad2.left_stick_y < 0){
            liftServoPosition = Range.clip(liftServoPosition - increment, 0, 0.55);
        }

        hardwareManager.liftServo.setPosition(liftServoPosition);
    if(initActive == false) {
        if (liftServoPosition == 0 || liftServoPosition <= 0.075) {
            hardwareManager.clawRotationServo.setPosition(0);
        } else if (liftServoPosition == 0.175) {
            hardwareManager.clawRotationServo.setPosition(0.27777777777);
        } else if (liftServoPosition == 0.45) {
            hardwareManager.clawRotationServo.setPosition(0.5);
        } else if (liftServoPosition < 0.45) {
            hardwareManager.clawRotationServo.setPosition(0.27777777777);
        } else if (liftServoPosition < 0.175) {
            hardwareManager.clawRotationServo.setPosition(0);
        }
      }
    }

    public void clawControls(){
        /*
            Macros,
            Right Trigger = ca
         */
        if(gamepad2.right_trigger > 0.3){ // Right Close
            leftClawServoPosition = 0.25;
        }else if(gamepad2.left_trigger > 0.3){ // Left Close
            rightClawServoPosition = 0;
        } else if(gamepad2.b ){ // Full Close
            rightClawServoPosition = 0;
            leftClawServoPosition = 0.25;
        } else if(gamepad2.x){ // Full Open
            rightClawServoPosition = 0.25;
            leftClawServoPosition = 0;
        }

        if(gamepad2.dpad_left){
            rightClawServoPosition = Range.clip(rightClawServoPosition - increment, 0, 0.25);
            leftClawServoPosition = Range.clip(leftClawServoPosition + increment, 0, 0.25);
        }else if(gamepad2.dpad_right){
            rightClawServoPosition = Range.clip(rightClawServoPosition + increment, 0, 0.25);
            leftClawServoPosition = Range.clip(leftClawServoPosition - increment, 0, 0.25);
        }
        hardwareManager.rightClawServo.setPosition(rightClawServoPosition);
        hardwareManager.leftClawServo.setPosition(leftClawServoPosition);
    }

    /*
    public void liftControl(boolean isPlayerOne) {
        Gamepad controller = (isPlayerOne) ? gamepad1 : gamepad2;
        hardwareManager.liftMotorLeft.setPower(controller.right_stick_y);
        hardwareManager.liftMotorRight.setPower(-controller.right_stick_y);
    }
     */





    //------------------------------------------------------------------------------------------------
    // Inheritance
    //------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
        //hardwareManager.intakeServo.setPosition(0);

        initActive = true;

        //hardwareManager.leftLiftServo.setPosition(0);
        hardwareManager.liftServo.setPosition(0);
        hardwareManager.rightClawServo.setPosition(0);
        hardwareManager.leftClawServo.setPosition(0.25);
        hardwareManager.clawRotationServo.setPosition(0.5);
    }

    public void setHardwarePower() {
        hardwareManager.frontLeftWheel.setPower(limitMotorPower(frontLeftWheelP));
        hardwareManager.frontRightWheel.setPower(limitMotorPower(frontRightWheelP));
        hardwareManager.backLeftWheel.setPower(limitMotorPower(backLeftWheelP));
        hardwareManager.backRightWheel.setPower(limitMotorPower(backRightWheelP));
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
