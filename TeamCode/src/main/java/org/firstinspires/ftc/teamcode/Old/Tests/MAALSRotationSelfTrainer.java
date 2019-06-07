package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

import java.util.IllegalFormatCodePointException;

/**
 * Created by August on 3/9/2018.
 */
//@Disabled
@Autonomous(name = "MAALS Rotation Self Trainer")
public class MAALSRotationSelfTrainer extends LinearOpMode {

    AutonomousRobot autonomousRobot;
    double xRad = -3650;
    double yRad = 7570;
    double sidewaysSpeed = 0;
    double forwardsSpeed = 0;
    double sidewaysSpeedSum = 0;
    double forwardsSpeedSum = 0;
    double currentAngle;
    double oldAngle;
    double turnRads=0;
    double turnPower = 0.4;
    double oldEncoderHeading =0;
    int loopCount=0;


    private double deltaHeadingRadians() {
        double change = currentAngle - oldAngle;
        if (change > Math.PI) {
            return change - 2 * Math.PI;
        } else if (change < -Math.PI) {
            return change + 2 * Math.PI;
        } else {
            return change;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonomousRobot = new AutonomousRobot(this, TeamColor.RED);
        MAALSRunner maalsRunner = new MAALSRunner(this, autonomousRobot);

        new Thread(maalsRunner).start();

        telemetry.addData("ready to start","");
        telemetry.update();

        waitForStart();

        autonomousRobot.wheelBase.setPowers(-turnPower, turnPower, -turnPower, turnPower);

        currentAngle = Math.toRadians(autonomousRobot.imu1.getAngularOrientation().firstAngle);

        oldEncoderHeading = maalsRunner.totalHeadingRad();

        while (!isStopRequested()) {

            oldAngle = currentAngle;
            currentAngle = Math.toRadians(autonomousRobot.imu1.getAngularOrientation().firstAngle);

            turnRads+=deltaHeadingRadians();

            if (turnRads > 8) {
//                xRad += (sidewaysSpeedSum/loopCount) / turnRads;
                xRad += ((sidewaysSpeedSum/loopCount) / turnRads) * 100;
                yRad -= ((forwardsSpeedSum/loopCount) / turnRads) * 100;

                sidewaysSpeedSum=0;
                forwardsSpeedSum=0;
                turnRads=0;
                loopCount=0;
                oldEncoderHeading = maalsRunner.totalHeadingRad();

                maalsRunner.setRadii(xRad, yRad);
            }

            sidewaysSpeed = maalsRunner.getSidewaysSpeed();
            forwardsSpeed = maalsRunner.getForwardSpeed();

            sidewaysSpeedSum += sidewaysSpeed;
            forwardsSpeedSum += forwardsSpeed;
            loopCount++;


            telemetry.addData("Sideways 1", maalsRunner.getSideways1());
            telemetry.addData("Sideways 2", maalsRunner.getSideways2());
            telemetry.addData("Forwards", maalsRunner.getForward());
//            telemetry.addData("X", positionTracker.getX());
//            telemetry.addData("Y", positionTracker.getY());
            telemetry.addData("Angle from gyro", currentAngle);
////            telemetry.addData("Ang velocity from gyro", autonomousRobot.imu1.getAngularVelocity().zRotationRate);
            telemetry.addData("Delta Heading Radians", maalsRunner.deltaHeadingRadians());
            telemetry.addData("Radians", turnRads);
            telemetry.addData("Calculated Radians", maalsRunner.totalHeadingRad()-oldEncoderHeading);
//            telemetry.addData("Sideways 1 per radian", positionTracker.getSideways1() / turnRads);
//            telemetry.addData("Sideways 2 per radian", positionTracker.getSideways2() / turnRads);
//            telemetry.addData("Forwards per radian", positionTracker.getForward() / turnRads);
            telemetry.addData("Forwards speed", maalsRunner.getForwardSpeed());
            telemetry.addData("Sideways speed", maalsRunner.getSidewaysSpeed());
//            telemetry.addData("Adjusted forwards", positionTracker.getAdjustedForward());
            telemetry.addData("X radius", xRad);
//            telemetry.addData("Radius magnitude",radiusMag);
            telemetry.addData("Y radius", yRad);
            telemetry.update();
        }

        sleep(20000);
    }
}
