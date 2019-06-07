package org.firstinspires.ftc.teamcode.OpModes.AndyAutos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.AndyAutonomous;
import org.firstinspires.ftc.teamcode.Robot.AutonomousNew;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 3/16/18.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Primary (Andy)")

public class BluePrimaryAndy extends AndyAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initTeamcolor(TeamColor.BLUE);
        initStartPlace(true);
        super.runOpMode();
    }
}