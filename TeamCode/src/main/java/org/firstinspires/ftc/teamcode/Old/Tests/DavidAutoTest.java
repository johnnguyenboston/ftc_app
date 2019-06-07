package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.VuforiaToOpenCV;


/**
 * Created by August on 11/25/2017.
 */
//@Disabled
@Autonomous (name = "David Auto Test")
public class DavidAutoTest extends LinearOpMode {
    VuforiaToOpenCV matGetter;
    TeamColor teamcolor = TeamColor.RED;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autoRobot = new AutonomousRobot(this, teamcolor);
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
        int thirdDepositX;
        int RIGHT_COLUMN_X = 1400;
        int COLUMN_WIDTH = 2280;
        double heading = 0;
        double collectionHeading = -30;
        if (teamcolor == TeamColor.BLUE) {
            RIGHT_COLUMN_X = -7060;
            heading = 180;
            collectionHeading = 180 + 30;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            firstDepositX = RIGHT_COLUMN_X;
        } else if (vuMark == RelicRecoveryVuMark.LEFT) {
            firstDepositX = RIGHT_COLUMN_X + 2*COLUMN_WIDTH;
        } else {
            firstDepositX = RIGHT_COLUMN_X + COLUMN_WIDTH;
        }
        autoRobot.servoController.collectorsOut();
        autoRobot.drive_off_ramp(0,firstDepositX, 0.35);
        autoRobot.servoController.clamp();
        autoRobot.servoController.dumperUp();
        autoRobot.turn(heading, 5);
        autoRobot.deposit(heading, autoRobot.getXPos());

        autoRobot.turn(collectionHeading, 5);
        autoRobot.positionTracker.setLocation(2000,0);
        autoRobot.drive_distance_time(10000, 0, collectionHeading, 0.7, 5000, true, true, false);
        autoRobot.drive_to_position(0, 0, collectionHeading, 0.4, 5000, 700,300, true);
        autoRobot.deposit(collectionHeading, 0);

        if (true) {
            return;
        }
        autoRobot.wheelBase.setPowers(0,0,0,0);
    }

}