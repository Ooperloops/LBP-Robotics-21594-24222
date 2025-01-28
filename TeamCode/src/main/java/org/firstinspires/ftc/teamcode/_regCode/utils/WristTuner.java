package org.firstinspires.ftc.teamcode._regCode.utils;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;

/**
 * --> HOW TO USE LIFTTUNER <--
 *  1. Run the OpMode
 *  2. Raise the lift the maximum height that it can reach
 *  3. Measure the difference in height between the lift when it was retract to when it is raised
 *  4.
 *
 */

@TeleOp(name = "WristTuner", group = "Tuning")
public class WristTuner extends OpMode{
    private HardwareManager hardwareManager; // Get Hardware Manager

    public double servoPos = 0;
    public double liftServoPosition = 0;

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap); // Initialize HardwareManager
        hardwareManager.clawRotationServo.setPosition(servoPos);
        hardwareManager.armServo.setPosition(0);
        //ResetLiftWheelCount(); // Reset the count of the lift motor encoder
    }
    @Override
    public void loop() {

        if(gamepad1.right_stick_y > 0){
            liftServoPosition = Range.clip(liftServoPosition + 0.0027, 0, 0.4);
        }else if(gamepad1.right_stick_y < 0){
            liftServoPosition = Range.clip(liftServoPosition - 0.0027, 0, 0.4);
        }
        if(gamepad1.a){ //Raise the lift
            servoPos = Range.clip(servoPos + 0.0027, 0, 1);
            //hardwareManager.lowerLiftMotor.setPower(-0.1);
        } else if (gamepad1.b){ //Lower the lift
            servoPos = Range.clip(servoPos - 0.0027, 0, 1);
        }

        hardwareManager.clawRotationServo.setPosition(servoPos);
        hardwareManager.armServo.setPosition(liftServoPosition);


        // Print the current ticks on the lift motor via telemetry
        telemetry
                .addData("Current Wrist Rotation ", servoPos)
                .addData("Current Arm Rotation ", liftServoPosition);

        telemetry.update();
    }

    //5770
    //5770
    //5720

    //----------------------------------------------------------------------
    // Private Methods //
    //----------------------------------------------------------------------


}
