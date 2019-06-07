package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by August on 3/11/2018.
 */
// @Disabled
@Autonomous(name = "MAALS Driving Test")
public class MAALSDrivingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousRobot autonomousRobot = new AutonomousRobot(this, TeamColor.RED);

        autonomousRobot.servoController.dumperDown();

        MAALSRunner maalsRunner = new MAALSRunner(this, autonomousRobot);
        MAALSDriver maalsDriver = new MAALSDriver(this, autonomousRobot, maalsRunner);

        waitForStart();

        new Thread(maalsRunner).start();
        new Thread(maalsDriver).start();

        maalsDriver.setTarget(0,5000,0);
        while (!isStopRequested()){
            telemetry.addData("X", maalsRunner.getX());
            telemetry.addData("Y", maalsRunner.getY());
            telemetry.addData("Move dist sideways", maalsDriver.getMoveDistSideways());
            telemetry.addData("Move dist forwards", maalsDriver.getMoveDistForwards());
            telemetry.addData("Angle from gyro", autonomousRobot.imu1.getAngularOrientation().firstAngle);
            telemetry.addData("Loop time driver", maalsDriver.deltaTime());
            telemetry.addData("Loop time runner", maalsRunner.deltaTime());
            telemetry.addData("Get time", maalsRunner.timeToReadValues());
            telemetry.addData("Calculate time", maalsRunner.timeToReadValues());
            telemetry.update();
        }
    }
}
//