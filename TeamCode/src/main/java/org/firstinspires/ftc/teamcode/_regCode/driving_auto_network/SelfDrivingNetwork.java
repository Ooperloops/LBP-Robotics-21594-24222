package org.firstinspires.ftc.teamcode._regCode.driving_auto_network;

import org.firstinspires.ftc.teamcode._regCode.base.SelfDriving;

/*
 * This is a neural-network style autonomous:
 * Each action (i.e Parking, Score High Basket, etc.) each have a specified double associated with it.
 * The higher the double, the higher said action is prioritized by the robot.
 * Like a neural network, there are inputs that effect what action output maybe given a certain time.
 * Input includes: time elapsed, distance from the
 *
 */

enum Actions{
    PARK,
    PUSH,
    GO_FISHING,
    SPECIMEN
}

public class SelfDrivingNetwork extends SelfDriving {
    //------------------------------------------------------------------------------------------------
    // Weight Variables
    //------------------------------------------------------------------------------------------------

    protected double time__park = 1;
    protected double time__push = 1;
    protected double time__fish = 1;
    protected double time__specimen = 1;

    @Override
    public void runAutonomous(){

    }
}
