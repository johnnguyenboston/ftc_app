package org.firstinspires.ftc.teamcode.OpModes.MainAutos;

import org.firstinspires.ftc.teamcode.Robot.AndyAutonomous;
import org.firstinspires.ftc.teamcode.Robot.AutonomousNew;
import org.firstinspires.ftc.teamcode.Robot.AutonomousSecondaryNew;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 3/16/18.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Secondary (New)")

public class BlueSecondaryNew extends AutonomousSecondaryNew{
    @Override
    public void runOpMode() throws InterruptedException {
        initTeamcolor(TeamColor.BLUE);
        initStartPlace(false);
        super.runOpMode();
    }
}
