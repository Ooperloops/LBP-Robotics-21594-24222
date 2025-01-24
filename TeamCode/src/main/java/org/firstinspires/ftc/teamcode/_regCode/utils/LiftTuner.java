package org.firstinspires.ftc.teamcode._regCode.utils;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;

/**
 * --> HOW TO USE LIFTTUNER <--
 *  1. Run the OpMode
 *  2. Raise the lift the maximum height that it can reach
 *  3. Measure the difference in height between the lift when it was retract to when it is raised
 *  4.
 *
 */

@TeleOp(name = "LiftTuner", group = "Tuning")
public class LiftTuner extends OpMode{
    private HardwareManager hardwareManager; // Get Hardware Manager

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap); // Initialize HardwareManager
        //ResetLiftWheelCount(); // Reset the count of the lift motor encoder
    }
    @Override
    public void loop() {
        if(gamepad1.a){ //Raise the lift
            hardwareManager.liftMotorLeft.setPower(0.5);
            //hardwareManager.lowerLiftMotor.setPower(-0.1);
        } else if (gamepad1.b){ //Lower the lift
            hardwareManager.liftMotorLeft.setPower(-0.1);
            //hardwareManager.lowerLiftMotor.setPower(0.5);
        } else { // Do nothing
            hardwareManager.liftMotorLeft.setPower(0);
            //hardwareManager.lowerLiftMotor.setPower(0);
        }

        // Print the current ticks on the lift motor via telemetry
        telemetry
            .addData("Current Lift Ticks: ", hardwareManager.liftMotorLeft.getCurrentPosition());
        telemetry.update();
    }

    //5770
    //5770
    //5720

    //----------------------------------------------------------------------
    // Private Methods //
    //----------------------------------------------------------------------


}
