package org.firstinspires.ftc.teamcode.regular_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.SelfDriving;

@Autonomous(name = "Auto Near Basket", group = "autonomous")
public class AutoNearBasket extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        MoveWheel(false, 2);
        move(0.2);
        rotate(-90);
        move(2);
        rotate(135);
        MoveUpwardSlide(true);
        MoveScoreBasket();
        MoveUpwardSlide(false);
        rotate(45);
        move(0.77);
        rotate(-90);
    }
}