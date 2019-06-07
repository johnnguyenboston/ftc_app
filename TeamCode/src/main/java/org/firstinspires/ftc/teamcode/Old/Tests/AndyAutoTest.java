package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.CubeDetector;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.VuforiaToOpenCV;
import org.opencv.core.Mat;


/**
 * Created by August on 11/25/2017.
 */
//@Disabled
@Autonomous (name = "Andy Auto Test")
public class AndyAutoTest extends LinearOpMode {

    TeamColor teamcolor = TeamColor.BLUE;
    boolean startingPrimary = true;

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

        MAALSRunner maalsRunner = new MAALSRunner(this, autonomousRobot);

        waitForStart();

        new Thread(maalsRunner).start();

        int startY = autonomousRobot.getYEncoder();

        // ------------------ Autonomous Starts ---------------------- //


        autonomousRobot.servoController.clamp();
//        autonomousRobot.drive_distance_rob(10000, 0, 0, 0.25, true, true, false);
        int currentY = autonomousRobot.getYEncoder();
        telemetry.addData("Y Delta",  currentY - startY);
        telemetry.update();
//        autonomousRobot.turnAndy(135, 5);
        autonomousRobot.servoController.dumperUp();
        autonomousRobot.drive_time(-1, -2, 135, 0.3, 1250, true, true, false);
        autonomousRobot.servoController.unClamp();
//        autonomousRobot.drive_distance_rob(2000, 0, 90, 0.3, true, true, false);
        telemetry.addData("Y", maalsRunner.getY());
        telemetry.addData("X", maalsRunner.getX());
        int targetY = maalsRunner.getX();
        telemetry.update();
        autonomousRobot.servoController.dumperDown();
        autonomousRobot.collectorThread.setPower(1);
        int currentX = autonomousRobot.getXEncoder();
//        autonomousRobot.drive_distance_rob(5600, 0, 90, 0.35, false, true, false);
        autonomousRobot.center_x_time(1, currentX, 80, 0.5, 300, true, false);
        autonomousRobot.center_x_time(1, currentX, 100, 0.5, 300, true, false);
        autonomousRobot.center_x_time(1, currentX, 80, 0.5, 300, true, false);
        autonomousRobot.center_x_time(1, currentX, 100, 0.5, 300, true, false);
        autonomousRobot.center_x_time(1, currentX, 80, 0.5, 300, true, false);
        autonomousRobot.center_x_time(1, currentX, 100, 0.5, 300, true, true);
//        autonomousRobot.drive_distance_rob(0, currentX - positionTracker.getY() , 90, 0.35, true, true, false);
//        autonomousRobot.drive_distance_rob(positionTracker.getX() - targetY, currentX - positionTracker.getY() , 90, 0.35, true, true, false);
    }
}