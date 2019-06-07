package org.firstinspires.ftc.teamcode.OpModes.MainAutos;

import org.firstinspires.ftc.teamcode.Robot.AutonomousNew;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 3/12/18.
 */


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Primary (New)")

public class BluePrimaryNew extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        initTeamcolor(TeamColor.BLUE);
        initStartPlace(true);
        super.runOpMode();
    }
}
