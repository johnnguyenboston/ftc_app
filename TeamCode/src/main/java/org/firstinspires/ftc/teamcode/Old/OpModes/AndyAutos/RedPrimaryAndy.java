package org.firstinspires.ftc.teamcode.OpModes.AndyAutos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.AndyAutonomous;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 3/16/18.
 */

@Disabled

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Primary (Andy)")

public class RedPrimaryAndy extends AndyAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initTeamcolor(TeamColor.RED);
        initStartPlace(true);
        super.runOpMode();
    }
}
