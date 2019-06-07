package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Mat;


/**
 * Created by August on 11/25/2017.
 */
public class AutonomousSecondaryNew extends LinearOpMode {
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
        int RIGHT_COLUMN_X = 1300;
        int COLUMN_WIDTH = 2280;
        int driveOffRampDistance = -8000;
        double heading = 0;
        if (teamcolor == TeamColor.BLUE) {
            RIGHT_COLUMN_X = -5800;
            driveOffRampDistance = -driveOffRampDistance;
            heading = 180;
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
        autoRobot.drive_to_position(driveOffRampDistance, 0, 0, 0.45, 3000, 700, 1000, false);
        int startX = autoRobot.positionTracker.getX();
        int startY = autoRobot.positionTracker.getY();
        autoRobot.servoController.clamp();
        autoRobot.servoController.dumperUp();
        autoRobot.turn(heading, 5);
        if (teamcolor == TeamColor.BLUE) {
            autoRobot.positionTracker.setLocation(startX, -startY);// make sure MAALS doesn't change when we turn
        } else {
            autoRobot.positionTracker.setLocation(startX, startY);// make sure MAALS doesn't change when we turn
        }
        autoRobot.drive_to_position(autoRobot.getYPos(), firstDepositX, heading, 0.7, 3000, 700, 300, true);
        autoRobot.deposit(heading, -9000);
        autoRobot.drive_to_position(autoRobot.getYPos(),teamcolor==TeamColor.RED ? 7400 : -7400, heading, 1, 1000,500,500,false);
        int preGrabX=autoRobot.getXPos();
        int preGrabY=autoRobot.getYPos();
        autoRobot.positionTracker.setLocation(preGrabX,preGrabY);
        autoRobot.collect(1);
        sleep(200);

        autoRobot.drive_to_pile(12000,preGrabX+(teamcolor==TeamColor.RED?3600:-3600),heading,0.5,3000,true,false);
        autoRobot.drive_distance(1000,0,heading,0.3,true,true,false);
        sleep(500);
        autoRobot.drive_to_pile(4000,preGrabX+(teamcolor==TeamColor.RED?3400:-3400),heading,0.5,3000,true,false);
        autoRobot.drive_distance(1000,0,heading,0.3,true,true,false);
        sleep(200);

        autoRobot.drive_to_position(preGrabY,preGrabX,heading,0.5,5000,700,500,false);
        autoRobot.drive_to_position(preGrabY,preGrabX,heading,0.2,2000,500,500,false);

        sleep(500);
        autoRobot.positionTracker.setLocation(preGrabX,preGrabY);
        autoRobot.drive_to_position(preGrabY,secondDepositX,heading,0.3,5000,200,500,true);
        autoRobot.deposit(heading,-9000);

        sleep(1000);
    }
}

