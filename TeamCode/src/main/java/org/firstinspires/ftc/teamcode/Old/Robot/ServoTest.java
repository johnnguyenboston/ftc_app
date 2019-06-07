package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 2/21/18.
 */

@Autonomous(name = "Servo Test")
public class ServoTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", " Initializing");
        telemetry.update();

        AutonomousRobot autonomousRobot = new AutonomousRobot(this, TeamColor.BLUE);
        autonomousRobot.servoController.dumperDown();

        telemetry.addData("Status", " Ready");
        telemetry.update();

        waitForStart();

        int stopTime = 700;

        telemetry.addData("Testing", " Jewel Mech");
        telemetry.update();
        // ---------- Jewel Mech ----------- //
        autonomousRobot.jewelPivot.setPosition(0);
        sleep(stopTime);
        autonomousRobot.jewelPivot.setPosition(1);
        sleep(stopTime);
        autonomousRobot.jewelArm.setPosition(0);
        sleep(stopTime);
        autonomousRobot.jewelArm.setPosition(1);
        sleep(stopTime);

        telemetry.addData("Testing", " Servo Stops");
        telemetry.update();
        // ---------- Cube Stops ----------- //
        autonomousRobot.leftServoStop.setPosition(0.5);
        autonomousRobot.rightServoStop.setPosition(0.5);
        sleep(stopTime);
        autonomousRobot.leftServoStop.setPosition(0);
        autonomousRobot.rightServoStop.setPosition(0);
        sleep(stopTime);

        telemetry.addData("Testing", " Clamps");
        telemetry.update();
        // ---------- Clamps ----------- //
        autonomousRobot.servoController.clamp();
        sleep(stopTime);
        autonomousRobot.servoController.unClamp();
        sleep(stopTime);

        telemetry.addData("Testing", " Dumpers");
        telemetry.update();
        // ---------- Dumpers ----------- //
        autonomousRobot.servoController.dumperUp();
        sleep(stopTime);
        autonomousRobot.servoController.dumperDown();
        sleep(stopTime);

        telemetry.addData("Testing", " Relic Mech");
        telemetry.update();
        // ----------- Relic ---------- //
        autonomousRobot.relicGripper.setPosition(1);
        sleep(stopTime);
        autonomousRobot.relicGripper.setPosition(0.4);
        sleep(stopTime);
        autonomousRobot.relicLifter.setPosition(0.9);
        sleep(stopTime);
        autonomousRobot.relicLifter.setPosition(0.13);
        sleep(stopTime);

        telemetry.addData("Testing", " Cryptobox Line Up");
        telemetry.update();
        // ---------- Extender ---------- //
//        autonomousRobot.servoController.setCryptoboxExtender(1000, 1);
        sleep(stopTime + 1000);
//        autonomousRobot.servoController.setCryptoboxExtender(1000, -1);
        sleep(stopTime + 1000);

    }
}
