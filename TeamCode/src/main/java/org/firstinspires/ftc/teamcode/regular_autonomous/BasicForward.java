package org.firstinspires.ftc.teamcode.regular_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.HumanOperated;
import org.firstinspires.ftc.teamcode.base.SelfDriving;


@TeleOp(name = "Basic Forward", group = "autonomous")
public class BasicForward extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        move(6);
    }
}