package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;


/**
 * Created by August on 11/25/2017.
 */
//@Disabled
@Autonomous (name = "Rob Auto Test")
public class RobAutoTest extends LinearOpMode {
    TeamColor teamcolor = TeamColor.RED;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autoRobot = new AutonomousRobot(this, teamcolor);
        autoRobot.setStartPosition(true);
        autoRobot.servoController.dumperDown();
        autoRobot.relicTrackables.activate();
        telemetry.addData("Status", " Ready");
        telemetry.update();
        waitForStart();
        autoRobot.timer.startTimer();
        autoRobot.knockOffJewel();
        int count = 0;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(autoRobot.relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && count < 250 && !isStopRequested()) {
            vuMark = RelicRecoveryVuMark.from(autoRobot.relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            count++;
            sleep(5);
        }
        int firstDepositX;
        int secondDepositX;
        int thridDepositX;
        int FIRST_COLUMN_X = 7000;
        int COLUMN_WIDTH = 2400;
        double heading = -90;
        if (!autoRobot.startingPrimary) {
            heading = 0;
            if (teamcolor == TeamColor.BLUE) {
                heading = 180;
            }
        }
        if (teamcolor == TeamColor.BLUE) {
            FIRST_COLUMN_X = -11800;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            firstDepositX = FIRST_COLUMN_X;
            secondDepositX = firstDepositX + COLUMN_WIDTH;
            thridDepositX = firstDepositX + 2*COLUMN_WIDTH;
        } else if (vuMark == RelicRecoveryVuMark.LEFT) {
            firstDepositX = FIRST_COLUMN_X + 2*COLUMN_WIDTH;
            secondDepositX = firstDepositX - COLUMN_WIDTH;
            thridDepositX = firstDepositX - 2*COLUMN_WIDTH;
        } else {
            firstDepositX = FIRST_COLUMN_X + COLUMN_WIDTH;
            secondDepositX = firstDepositX + COLUMN_WIDTH;
            thridDepositX = firstDepositX - COLUMN_WIDTH;
        }
        autoRobot.servoController.collectorsOut();
        autoRobot.drive_distance(-firstDepositX, 0, 0, 0.3, true, true, false);
        autoRobot.displayPosition();
//        int startX = autoRobot.positionTracker.getX();
//        int startY = autoRobot.positionTracker.getY();
        autoRobot.turn(heading, 5);
//        autoRobot.positionTracker.setLocation(-startY, startX); // make sure MAALS doesn't change when we turn
        autoRobot.displayPosition();
//        autoRobot.deposit(heading);
        autoRobot.timer.stopTimer();
        long timeLeft = 30000 - autoRobot.timer.getTime();
        int firstCollectionTime = (int)timeLeft - 4000;
        //first cube collect
        autoRobot.collectDavid(firstCollectionTime, 0.7, 0.6, heading);


        autoRobot.center_x(-autoRobot.getYPos(), secondDepositX, heading, 0.5, 5000, true, true);
        int xPositionError = secondDepositX - autoRobot.getXPos();
        if (Math.abs(xPositionError) > 700) {
            autoRobot.drive_distance(0, secondDepositX - autoRobot.getXPos(), heading, 0.2, true, true, false);
        }

//        autoRobot.deposit(-90);
        autoRobot.timer.stopTimer();
        //second cube collect
        timeLeft = 30000 - autoRobot.timer.getTime();
        int secondCollectionTime = (int)timeLeft - 4000;
        if (secondCollectionTime > 3000) {
            autoRobot.collectDavid(secondCollectionTime, 0.7, 0.4, heading);

            autoRobot.center_x(-autoRobot.getYPos(), thridDepositX, heading, 0.5, 5000, true, true);
            xPositionError = thridDepositX - autoRobot.getXPos();
            if (Math.abs(xPositionError) > 700) {
                autoRobot.drive_distance(0, thridDepositX - autoRobot.getXPos(), heading, 0.2, true, true, false);
            }

//            autoRobot.deposit(heading);
        }
        autoRobot.wheelBase.setPowers(0,0,0,0);
    }
}