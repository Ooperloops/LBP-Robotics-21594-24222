package org.firstinspires.ftc.teamcode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.HumanOperated;


@TeleOp(name = "Normal Control", group = "TeleOp")
public class NormalControl extends HumanOperated {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void loop() {
        useDefaultMovementControls();
        setHardwarePower();
        intakeArmControl();

    }
}