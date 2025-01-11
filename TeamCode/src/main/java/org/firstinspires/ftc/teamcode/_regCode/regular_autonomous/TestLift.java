package org.firstinspires.ftc.teamcode._regCode.regular_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;

@Autonomous(name = "TestLift", group = "autonomous")
public class TestLift extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {

        ScoreHighBasket();
    }
}