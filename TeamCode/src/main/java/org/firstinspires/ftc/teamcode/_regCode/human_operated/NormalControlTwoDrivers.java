package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._regCode.base.HumanOperated;


@TeleOp(name = "Normal Two Drivers", group = "TeleOp")
public class NormalControlTwoDrivers extends HumanOperated {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void loop() {
        useDefaultMovementControls(


        );
        ActiveIntakeTwoPlayer();
        liftControl(false);
        setHardwarePower();
    }
}