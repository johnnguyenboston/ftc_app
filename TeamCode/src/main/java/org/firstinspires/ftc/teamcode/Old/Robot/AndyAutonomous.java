package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Mat;


/**
 * Created by August on 11/25/2017.
 */
//@Disabled
public class AndyAutonomous extends LinearOpMode {

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
        int columnXPos, lastEncoderY, triangleXPos, secondDepositColumnX, triangleYPos, turnHeading;
        double heading;
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autonomousRobot = new AutonomousRobot(this, teamcolor);
        autonomousRobot.servoController.dumperDown();

        autonomousRobot.relicTrackables.activate();

        VuforiaToOpenCV matGetter = new VuforiaToOpenCV(autonomousRobot.vuforia);
        CubeDetector cubeDetector = new CubeDetector(this);
        Mat image;
        CameraDevice.getInstance().setFlashTorchMode(true);

        new Thread(matGetter).start();

        telemetry.addData("Status", " Ready");
        telemetry.update();

        waitForStart();

        // ------------------ AutonomousMassStates Start ---------------------- //

        boolean flipX = false;
        if (teamcolor == TeamColor.BLUE) {
            flipX = true;
        }


        int yValueCorrection;

//        autonomousRobot.yEncoderValue = - autonomousRobot.collectorR.getCurrentPosition();
//        yValueCorrection = autonomousRobot.yEncoderValue;


        autonomousRobot.knockOffJewel();
        autonomousRobot.jewelPivot.setPosition(1);

        //Read Vumark
        int count = 0;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(autonomousRobot.relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && count < 250 && !isStopRequested()) {
            vuMark = RelicRecoveryVuMark.from(autonomousRobot.relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            count++;
            sleep(5);
        }

        //decide what angle the robot should turn to deposit the glyph depending on the column.
        // change depending on the team color too!
        int columnHeading ;
        int triangleHeading;
        if (teamcolor == TeamColor.RED) {
            if (startingPrimary) {
                triangleHeading = 90;
            } else {
                triangleHeading = 180;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                columnHeading = triangleHeading + 60;
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                columnHeading = triangleHeading + 120;
            } else {
                columnHeading = triangleHeading + 60;
            }
        } else {
            if (startingPrimary){
                triangleHeading = 270;
            } else {
                triangleHeading = 180;
            }

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                columnHeading = triangleHeading - 120;
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                columnHeading = triangleHeading - 60;
            } else {
                columnHeading = triangleHeading - 60;
            }
        }

        autonomousRobot.servoController.clamp();
//        autonomousRobot.drive_off_ramp_andy(0, true);

        // auto fixes the y displacement after crabbing off the balancing stone. Only if the displacemnet is big enough to throw us off.
        if(true){

        } else {
//            autonomousRobot.drive_distance_rob( yValueCorrection - autonomousRobot.yEncoderValue, 0, 0, 0.25, true, true, false);
        }
        int crabExtraDistance;
        if (startingPrimary) {
//            autonomousRobot.drive_distance_rob(700, 0, 0, 0.3, true, true, false);
        }


//        autonomousRobot.turnAndy(triangleHeading, 5);
        //autonomousRobot.drive_time(-1000, -700, heading, 0.4, 3000, true, false, flipX);


        int targetY;
        int columnAlignment;
        autonomousRobot.drive_color(1000, -1, triangleHeading, 0.225, true);
        int firstLineY = 0;
        targetY = firstLineY;
//        int overshootY = -autonomousRobot.collectorR.getCurrentPosition();
//        if (Math.abs (firstLineY - overshootY) > 150){
//            autonomousRobot.drive_distance_rob(firstLineY - overshootY, 0, triangleHeading, 0.25, true, true, false);
//        }
//            telemetry.addData("overShootCorrection", firstLineY - overshootY);
//        telemetry.update();
//        //auto correct for y overshot after detecting a line.
        int secondLineY;
        int secondDepositColumnXValue;
        if(vuMark == RelicRecoveryVuMark.RIGHT && teamcolor == TeamColor.RED || vuMark == RelicRecoveryVuMark.LEFT && teamcolor == TeamColor.BLUE || vuMark == RelicRecoveryVuMark.CENTER){
            if(vuMark == RelicRecoveryVuMark.CENTER){

                columnAlignment = 1500;
            } else {
                columnAlignment = -200;
            }
            autonomousRobot.wheelBase.setPowers(0, 0, 0, 0);
        } else {
            //drives to second line if the VuMark is the far column depending on team color.

//            autonomousRobot.drive_no_color(1000, -1, triangleHeading, 0.2, false);
            autonomousRobot.drive_color(1000, -1, triangleHeading, 0.23, true);

            targetY = 0;
            columnAlignment = -2100;


        }
//        autonomousRobot.drive_y_distance( targetY + columnAlignment, triangleHeading, 0.3, true, true);
        //place glyph in cryptobox
//        autonomousRobot.turnAndy(columnHeading, 5);
//        autonomousRobot.drive_distance_rob(1000, 0 , columnHeading, 0.5, true, true, false);
        autonomousRobot.wheelBase.setPowers(0, 0, 0, 0);
        autonomousRobot.servoController.dumperUp();
        sleep(500);
        autonomousRobot.servoController.unClamp();
        autonomousRobot.drive_time(-1, 0, columnHeading, 0.5, 1800, true, true, false);
//        autonomousRobot.drive_distance_rob(700, 0, columnHeading, 0.5, false, true, true);
        int lastTurnHeading;
        if (startingPrimary){
            lastTurnHeading = 180;
        } else {
            if(teamcolor == TeamColor.RED){
                lastTurnHeading = 130;
            }
            lastTurnHeading = 130;
        }
//        autonomousRobot.turnAndy(lastTurnHeading, 5);
        autonomousRobot.servoController.dumperDown();
    }

}