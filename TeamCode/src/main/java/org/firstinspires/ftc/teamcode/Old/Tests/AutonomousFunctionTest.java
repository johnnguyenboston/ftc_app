package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.VuforiaToOpenCV;

/**
 * Created by David on 2/11/18.
 */
//@Disabled
@Autonomous(name = "Function Test")
public class AutonomousFunctionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", " Initializing");
        telemetry.update();
        AutonomousRobot autoRobot = new AutonomousRobot(this, TeamColor.RED);
        autoRobot.setStartPosition(true);
        autoRobot.servoController.dumperDown();
        autoRobot.relicTrackables.activate();

        VuforiaToOpenCV matGetter = new VuforiaToOpenCV(autoRobot.vuforia);

        new Thread(matGetter).start();

        telemetry.addData("Status", " Ready");
        telemetry.update();

        waitForStart();
        autoRobot.servoController.collectorsOut();

        autoRobot.deposit(0,0);

        sleep(10000);
    }
}
