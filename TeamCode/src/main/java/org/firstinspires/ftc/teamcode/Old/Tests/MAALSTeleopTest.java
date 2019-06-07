package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;


/**
 * Created by August on 11/25/2017.
 */

//@Disabled
@Autonomous(name = "MAALS teleop test")
public class MAALSTeleopTest extends LinearOpMode {

    double revolutions = 0;

    AutonomousRobot autonomousRobot;
    double FRpower, BRpower, FLpower, BLpower;

    @Override
    public void runOpMode() throws InterruptedException {
        autonomousRobot = new AutonomousRobot(this, TeamColor.RED);

        autonomousRobot.servoController.dumperDown();

        MAALSRunner maalsRunner = new MAALSRunner(this, autonomousRobot);

        waitForStart();

        new Thread(maalsRunner).start();

        while (!isStopRequested()) {

            final double driveMod = 0.6;

            double YVal = -gamepad1.left_stick_y;
            double XVal = -gamepad1.left_stick_x;
            double RotVal = gamepad1.right_stick_x;

            // clip the right/left values so that the   values never exceed +/- 1
            double YPower = Range.clip(YVal, -1, 1);
            double XPower = Range.clip(XVal, -1, 1);
            double rotPower = Range.clip(RotVal, -1, 1);

            //combine values
            FRpower = YPower + XPower - rotPower;
            BRpower = YPower - XPower - rotPower;
            FLpower = YPower - XPower + rotPower;
            BLpower = YPower + XPower + rotPower;

            //clip power to acceptable levels
            FRpower = Range.clip(FRpower, -1, 1);
            FLpower = Range.clip(FLpower, -1, 1);
            BRpower = Range.clip(BRpower, -1, 1);
            BLpower = Range.clip(BLpower, -1, 1);

            if (gamepad1.a){
                maalsRunner.setLocation(0,0,0);
            }

            if (gamepad1.right_bumper){
                autonomousRobot.collect(1);
            } else {
                autonomousRobot.collect(0);
            }
            // set power
            //Square values and preserve sign

            autonomousRobot.wheelBase.setPowers(driveMod * Math.abs(FLpower) * (FLpower),driveMod * Math.abs(FRpower) * (FRpower),driveMod * Math.abs(BLpower) * (BLpower),driveMod * Math.abs(BRpower) * (BRpower));

//            autonomousRobot.FR.setPower(driveMod * -Math.abs(FRpower) * (FRpower));
//            autonomousRobot.FL.setPower(driveMod * Math.abs(FLpower) * (FLpower));
//            autonomousRobot.BR.setPower(driveMod * -Math.abs(BRpower) * (BRpower));
//            autonomousRobot.BL.setPower(driveMod * Math.abs(BLpower) * (BLpower));

//            telemetry.addData("Sideways 1", positionTracker.getSideways1());
//            telemetry.addData("Sideways 2", positionTracker.getSideways2());
//            telemetry.addData("Forwards", positionTracker.getForward());
            telemetry.addData("X", maalsRunner.getX());
            telemetry.addData("Y", maalsRunner.getY());
//            telemetry.addData("Angle from gyro", autonomousRobot.imu1.getAngularOrientation().firstAngle);
            telemetry.addData("calculated angle", maalsRunner.headingRad());
            telemetry.addData("Sideways 1", maalsRunner.getSideways1());
            telemetry.addData("Sideways 2", maalsRunner.getSideways2());
            telemetry.addData("Forwards", maalsRunner.getForward());//            telemetry.addData("Ang velocity from gyro", autonomousRobot.imu1.getAngularVelocity().zRotationRate);
//            telemetry.addData("Delta Heading Radians", positionTracker.getDeltaHeadingRadians());
//            telemetry.addData("Sideways speed", positionTracker.getSidewaysSpeed());
//            telemetry.addData("Forwards speed", positionTracker.getForwardSpeed());
//            telemetry.addData("Adjusted forwards", positionTracker.getAdjustedForward());
            telemetry.update();
        }
    }
}
