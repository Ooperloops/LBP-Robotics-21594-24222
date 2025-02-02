package org.firstinspires.ftc.teamcode._regCode.roadrunner_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._regCode.complexAuto.NetworkedAuto;
import org.firstinspires.ftc.teamcode._regCode.complexAuto.StartPos;

@Autonomous(name = "Cycled Auto City Comp MID Basket Side", group = "RoadRunner")
public class CycledAutoExLeft extends NetworkedAuto {

    /* > The following code overrides the "initVariables" function
     * > The programmers must set all of the following variables to their liking for a new auto script
     *
     * Variables:
     * startPosition [enum] -> an enum with all the possible start positions that our robot can be in
     * loadedPosition [enum] -> an enum if bot has either a sample or specimen preloaded
     * HighBasket [int] = # of times the bot tries to score high basket
     * Pushing [int] = # of times you want to push coloured samples in observation zn.
     * HighRung [int] = # of high rung specimens the bot could try to hang
     * Parking [bool] = park or not
     */

    @Override
    protected void initVariables() {
        startPosition = StartPos.MIDBASKET;
        loadedPosition = LoadedPosition.LOADED_SPECIMEN;
        HighBasket = 0;
        Pushing = 0;
        HighRung = 1;
        Parking = false;

    }
}