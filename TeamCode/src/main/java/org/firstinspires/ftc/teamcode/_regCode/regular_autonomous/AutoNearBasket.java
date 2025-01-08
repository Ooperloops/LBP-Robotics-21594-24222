package org.firstinspires.ftc.teamcode._regCode.regular_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;

@Autonomous(name = "Auto Near Basket", group = "autonomous")
public class AutoNearBasket extends SelfDriving {

    /* > The following code overrides the "loop" function
     * > The "loop" function is constantly called as the TeleOp is running
     */
    @Override
    public void runAutonomous() {
        //hardwareManager.intakeServo.setPosition(0);
        move(0.2);
        rotate(-90);
        move(2);
        rotate(135);
        MoveUpwardSlide(true);
        MoveScoreBasket();
        Delay(2000);
        MoveUpwardSlide(false);
        rotate(45);
        move(2);
        rotate(-90);
    }
}