package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.AutonomousNew;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 3/12/18.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Primary (Triangle)")

public class RedPrimaryTriangle extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        initTeamcolor(TeamColor.RED);
        initStartPlace(true);
        super.runOpMode();
    }
}
