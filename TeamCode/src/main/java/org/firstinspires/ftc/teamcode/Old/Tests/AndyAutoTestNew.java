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
@Autonomous (name = "Andy Auto Test New")
public class AndyAutoTestNew extends LinearOpMode {

    TeamColor teamcolor = TeamColor.RED;
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

        int startY = autonomousRobot.getYEncoder();
//        autonomousRobot.drive_distance_rob(10000, 0, 0, 0.25, true, true, false);
        int currentY = autonomousRobot.getYEncoder();
        telemetry.addData("Y Delta",  currentY - startY);
        telemetry.update();
    }

}