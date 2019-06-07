package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.TriangleDetector;
import org.firstinspires.ftc.teamcode.Robot.VuforiaToOpenCV;

/**
 * Created by August on 12/23/2017.
 */
@Disabled
@Autonomous(name = "OpenCV Processing Test")
public class OpenCVTest extends LinearOpMode {
    VuforiaToOpenCV matGetter;
    TriangleDetector triangleDetector;
    AutonomousRobot autonomousRobot;

    TeamColor teamColor = TeamColor.RED;

    private void initVuforia() {
        VuforiaLocalizer vuforia;
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWRj/xj/////A AAAGQxUvYmWcEwclJxNRPrJOqYGevnetxTRA6P/gYZfb+gZsAeO0GosrfKmnO2O24hVPv8v1YQYA8vQ5qc1eVjwOjZPCykT4eRXxqxLZV/7BJmEraEs991INaYI9qXQjGkbeWbLQT/e7zJAvxsRVjQjRPukDLapC4dmdA5YbXvxy9pR2+LokoO6PiSLl9ktBte3BFGHQepiugBC7C1jXDfkClwTb/+R7OwaVuL1gp6rWun5Cn42RHysv4HsTkBMShaKdL4/whXVRmrYfkMMsAtihEAK+rLs8fWnmVB1Z/UJ67QIWqP04Va/u/mbTErjPDiRvCnYhGmIWTpIt+9slhip9vT9pRfLIe+gcfAIajoF6wUe";
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        matGetter = new VuforiaToOpenCV(vuforia);
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        initVuforia();
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autonomousRobot = new AutonomousRobot(this, teamColor);

        autonomousRobot.servoController.dumperDown();
        autonomousRobot.relicTrackables.activate();
        matGetter = new VuforiaToOpenCV(autonomousRobot.vuforia);
//        Mat image;
        CameraDevice.getInstance().setFlashTorchMode(true);
        new Thread(matGetter).start();
        triangleDetector = new TriangleDetector(this, teamColor);

        telemetry.addData("Status", " Ready");
        telemetry.update();

        waitForStart();
        triangleDetector.loadImage(matGetter);
//            matGetter.saveMatImage(triangleDetector.getInputMat(), "vuforia_image");
        triangleDetector.calculateContours(triangleDetector.getInputMat());
        triangleDetector.updateScreen();

//        autonomousRobot.center_y(0,triangleDetector.calculateCrabDistance(),0,0.3, true,true);

        while (opModeIsActive()) {
            triangleDetector.loadImage(matGetter);
//            matGetter.saveMatImage(triangleDetector.getInputMat(), "vuforia_image");
            triangleDetector.calculateContours(triangleDetector.getInputMat());
            triangleDetector.updateScreen();
        }
    }
}
