package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.CubeDetector;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.VuforiaToOpenCV;
import org.opencv.core.Mat;


/**
 * Created by August on 11/25/2017.
 */
@Autonomous(name = "Mark Auto Test")
public class MarkAutoTest extends LinearOpMode {
    TeamColor teamcolor;
    boolean startingPrimary;

    public void initTeamcolor (TeamColor teamcolor) {
        this.teamcolor = teamcolor;
    }

    public void initStartPlace (boolean startingPrimary) {
        this.startingPrimary = startingPrimary;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autoRobot = new AutonomousRobot(this, teamcolor);
//        autoRobot.maalsRunner.setLocation(0,0,0);
        autoRobot.servoController.dumperDown();
        autoRobot.relicTrackables.activate();
        telemetry.addData("Status", " Ready");
        telemetry.update();

        waitForStart();

//        autoRobot.maalsRunner.setLocation(0,0,0);

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
        int thirdDepositX;
        int RIGHT_COLUMN_X = 9000;
        int COLUMN_WIDTH = 2280;
        double heading = -90;
        if (teamcolor == TeamColor.BLUE) {
            RIGHT_COLUMN_X = -9800;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            firstDepositX = RIGHT_COLUMN_X;
            secondDepositX = firstDepositX + COLUMN_WIDTH;
            thirdDepositX = firstDepositX + 2*COLUMN_WIDTH;
        } else if (vuMark == RelicRecoveryVuMark.LEFT) {
            firstDepositX = RIGHT_COLUMN_X + 2*COLUMN_WIDTH;
            secondDepositX = firstDepositX - COLUMN_WIDTH;
            thirdDepositX = firstDepositX - 2*COLUMN_WIDTH;
        } else {
            firstDepositX = RIGHT_COLUMN_X + COLUMN_WIDTH;
            secondDepositX = firstDepositX - COLUMN_WIDTH;
            thirdDepositX = firstDepositX + COLUMN_WIDTH;
        }
        autoRobot.servoController.collectorsOut();
        autoRobot.drive_off_ramp(-firstDepositX, 0, 0.35);
        autoRobot.turn(heading, 5);

        autoRobot.displayPosition();
        int startX1 = -autoRobot.collectorL.getCurrentPosition();
        int startX2 = -autoRobot.extender.getCurrentPosition();
        autoRobot.deposit(heading, autoRobot.getXPos());


//        if (true)
//            return;
        autoRobot.timer.stopTimer();
        long timeLeft = 30000 - autoRobot.timer.getTime();
        int firstCollectionTime = (int)timeLeft - 4000;

        //first cube collect
        autoRobot.collect_withCentering(firstCollectionTime,1,0.7,heading,9000, 7000);
        // shifts back if too far over if it would hit the balancing stone
        if (autoRobot.positionTracker.getX() > 10000 || autoRobot.positionTracker.getX() < 4000) {
            autoRobot.drive_distance(0, secondDepositX - autoRobot.positionTracker.getX(), heading, 0.3, true, true, false);
        }

        //drive back to cryptobox
        autoRobot.drive_to_position(800, secondDepositX, heading, 0.7, 5000,700,300, true);
//        autoRobot.center_x(-autoRobot.maalsRunner.getY() + 800, secondDepositX, heading, 0.7, 5000, true, true);
        //autoRobot.drive_distance(800 - autoRobot.maalsRunner.getY(), secondDepositX - autoRobot.maalsRunner.getX(), heading, 0.7, true, true, false);
        int xPositionError = secondDepositX - autoRobot.positionTracker.getX();
        if (Math.abs(xPositionError) > 700) {
            autoRobot.drive_distance(0, xPositionError, heading, 0.15, true, true, false);
        }
        autoRobot.displayPosition();

        autoRobot.deposit(heading, autoRobot.getXPos());
        autoRobot.timer.stopTimer();
        //second cube collect
        timeLeft = 30000 - autoRobot.timer.getTime();
        int secondCollectionTime = (int)timeLeft - 5000;
        if (secondCollectionTime > 3000) {
            autoRobot.collect_withCentering(secondCollectionTime, 1, 0.7, heading, 9000, 12000);

//            autoRobot.center_x(-autoRobot.maalsRunner.getY() + 800, thirdDepositX, heading, 0.7, 5000, true, true);
            autoRobot.drive_to_position(800, thirdDepositX, heading, 0.7, 5000,700,300, true);
            xPositionError = thirdDepositX - autoRobot.positionTracker.getX();
            if (Math.abs(xPositionError) > 700) {
                autoRobot.drive_distance(0, xPositionError, heading, 0.15, true, true, false);
            }
            autoRobot.displayPosition();
            autoRobot.deposit(heading, autoRobot.getXPos());
        }
        int endX1 = -autoRobot.collectorL.getCurrentPosition();
        int endX2 = -autoRobot.extender.getCurrentPosition();

        telemetry.addData("diffX1", endX1 - startX1);
        telemetry.addData("diffX2", endX2 - startX2);
        telemetry.update();
        sleep(10000);

        autoRobot.wheelBase.setPowers(0,0,0,0);
    }

}