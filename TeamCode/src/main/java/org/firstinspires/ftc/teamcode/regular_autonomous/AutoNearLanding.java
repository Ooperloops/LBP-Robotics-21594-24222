package org.firstinspires.ftc.teamcode.regular_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.SelfDriving;

@Autonomous(name = "Auto Near Landing", group = "autonomous")
public class AutoNearLanding extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        rotate(90);
        move(2);
    }
}