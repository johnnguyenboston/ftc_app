package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Mat;


/**
 * Created by August on 11/25/2017.
 */
public class AutonomousNew extends LinearOpMode {
    TeamColor teamcolor;
    boolean startingPrimary;
    Timer autoTimer;

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
        autoRobot.servoController.dumperDown();
        autoRobot.relicTrackables.activate();
        telemetry.addData("Status", " Ready");
        telemetry.update();
        autoTimer = new Timer(this);


        waitForStart();
        autoTimer.startTimer();
        autoRobot.knockOffJewel();
        autoRobot.servoController.clampGentle();


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
        int RIGHT_COLUMN_X = 8000;
        int COLUMN_WIDTH = 2280;
        int FIRST_COLLECION_START_POS = 7000;
        int SECOND_COLLECTION_START_POS = 10000;
        double heading = -90;
        if (teamcolor == TeamColor.BLUE) {
            RIGHT_COLUMN_X = -13500;
            FIRST_COLLECION_START_POS = -8000;
            SECOND_COLLECTION_START_POS = -12000;
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
        int startX = autoRobot.positionTracker.getX();
        int startY = autoRobot.positionTracker.getY();
        autoRobot.servoController.clamp();
        autoRobot.servoController.dumperUp();
        autoRobot.turn(heading, 5);
        int startX1 = autoRobot.positionTracker.getEncoderX1();
        int startX2 = autoRobot.positionTracker.getEncoderX2();
        autoRobot.positionTracker.setLocation(-startY, startX);// make sure MAALS doesn't change when we turn
        autoRobot.deposit(heading, -2300);

        if (!autoRobot.positionTracker.isYWorking()) {
            sleep(100);
            return;
        }

        autoTimer.stopTimer();
        long timeLeft = 30000 - autoTimer.getTime();
        int firstCollectionTime = (int)timeLeft - 4000;
        //first cube collect
//        autoRobot.collect_withCentering(firstCollectionTime, 1, 0.7, heading, 9000, FIRST_COLLECION_START_POS);
        boolean startAtMax = false;
        if ((teamcolor == TeamColor.RED && vuMark == RelicRecoveryVuMark.LEFT) || (teamcolor == TeamColor.BLUE && (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.CENTER))) {
            startAtMax = true;
        }
        autoRobot.collectSlide(firstCollectionTime, 1, 0.7, heading, startAtMax);
        //drive back to cryptobox
        autoTimer.stopTimer();
        timeLeft = 30000 - autoTimer.getTime();

        int xTol = 300;
        if (timeLeft < 4000) {
            xTol = 600;
        }
        autoRobot.drive_to_position(800, secondDepositX, heading, 1, (int)Math.min(timeLeft - 1500, 6000), 700, xTol, true);
        autoRobot.deposit(heading, -2300);
        //second cube collect
        autoTimer.stopTimer();
        timeLeft = 30000 - autoTimer.getTime();
        int secondCollectionTime = (int)timeLeft - 5000;
        if (secondCollectionTime > 4000) {
//            autoRobot.collect_withCentering(secondCollectionTime, 1, 0.7, heading, 9000, SECOND_COLLECTION_START_POS);
            autoRobot.collectSlide(secondCollectionTime, 1, 0.7, heading, !startAtMax);
            autoTimer.stopTimer();
            timeLeft = 30000 - autoTimer.getTime();
            xTol = 300;
            if (timeLeft < 3000) {
                xTol = 600;
            }
            autoRobot.drive_to_position(800, thirdDepositX, heading, 1, (int)Math.min(timeLeft - 1500, 6000), 700, xTol, true);
            autoRobot.deposit(heading, -2300);
        }
        autoRobot.wheelBase.setPowers(0,0,0,0);
        autoRobot.collect(0);
        sleep(100);

        int endX1 = -autoRobot.collectorL.getCurrentPosition();
        int endX2 = -autoRobot.extender.getCurrentPosition();

        telemetry.addData("zeroDeltaCountX1", autoRobot.positionTracker.getZeroDeltaCountX1());
        telemetry.addData("zeroDeltaCountX2", autoRobot.positionTracker.getZeroDeltaCountX2());
        telemetry.addData("total", autoRobot.positionTracker.count);
        telemetry.update();
        sleep(10000);
    }
}

