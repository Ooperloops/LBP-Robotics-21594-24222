package org.firstinspires.ftc.teamcode._regCode.utils;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;

/**
 * --> HOW TO USE LIFTTUNER <--
 *  1. Run the OpMode
 *  2. Raise the lift the maximum height that it can reach
 *  3. Measure the difference in height between the lift when it was retract to when it is raised
 *  4.
 *
 */

@TeleOp(name = "Arm servo angle getter", group = "Tuning")
public class ServoAngleGetter extends OpMode{
    private HardwareManager hardwareManager; // Get Hardware Manager

    @Override
    public void init() {
        hardwareManager = new HardwareManager(hardwareMap);
    }
    @Override
    public void loop() {
        double leftArmPosToAng = hardwareManager.leftArmServo.getPosition() * 180;
        double rightArmPosToAng = hardwareManager.rightArmServo.getPosition() * 180;

        telemetry
                .addData("Left Arm Position", "%.2f", hardwareManager.leftArmServo.getPosition())
                .addData("Right Arm Position", "%.2f", hardwareManager.rightClawServo.getPosition())
                .addData("Left Arm Angle", "%.2f", leftArmPosToAng)
                .addData("Left Arm Angle", "%.2f",rightArmPosToAng);
        telemetry.update();
    }

    //----------------------------------------------------------------------
    // Private Methods
    //----------------------------------------------------------------------


}
