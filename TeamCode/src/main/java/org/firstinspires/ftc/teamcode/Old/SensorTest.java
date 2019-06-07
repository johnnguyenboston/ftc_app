package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 11/8/17.
 */

@TeleOp(name="SensorTest", group="Teleop")
//@Disabled
public class SensorTest extends LinearOpMode {
    OpMode opMode = this;
    Robot robot = new Robot(opMode, TeamColor.NOCOLOR);


    @Override
    public void runOpMode() {
        robot.initAuto(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Gyro", "");
            telemetry.addData("Yaw", robot.imu1.getAngularOrientation().firstAngle);
            telemetry.addData("Roll", robot.imu1.getAngularOrientation().secondAngle);
            telemetry.addData("Pitch", robot.imu1.getAngularOrientation().thirdAngle);
            telemetry.addData("Angular Velocity", robot.imu1.getAngularVelocity().zRotationRate);

            telemetry.addData("\nEncoders", "");
            telemetry.addData("fl", robot.FL.getCurrentPosition());
            telemetry.addData("fr", robot.FR.getCurrentPosition());
            telemetry.addData("bl", robot.BL.getCurrentPosition());
            telemetry.addData("br", robot.BR.getCurrentPosition());
            telemetry.addData("lift", robot.lift.getCurrentPosition());
            telemetry.addData("encoder (x)", robot.collectorL.getCurrentPosition());
            telemetry.addData("encoder (x) 2", robot.extender.getCurrentPosition());
            telemetry.addData("encoder (y)", robot.collectorR.getCurrentPosition());

            telemetry.addData("\nColor Sensors", "");
            telemetry.addData("Jewel blue", robot.jewelColor.blue());
            telemetry.addData("Jewel red", robot.jewelColor.red());
            telemetry.addData("colorFront alpha", robot.colorFront.alpha());
            telemetry.addData("colorBack alpha", robot.colorBack.alpha());
            telemetry.addData("colorFront blue", robot.colorFront.blue());
            telemetry.addData("colorBack blue", robot.colorBack.blue());
            telemetry.addData("colorFront red", robot.colorFront.red());
            telemetry.addData("colorBack red", robot.colorBack.red());
            telemetry.addData("colorFront green", robot.colorFront.green());
            telemetry.addData("colorBack green", robot.colorBack.green());
            telemetry.addData("colorFront argb", robot.colorFront.argb());
            telemetry.addData("colorBack argb", robot.colorBack.argb());

            telemetry.addData("\nDistance Sensors", "");
            telemetry.addData("Pile Detector", robot.pileDetector.getVoltage());
            telemetry.addData("Cube Orienter", robot.cubeOrienter.getVoltage());
            telemetry.addData("Ramp Postion", robot.rampDetector.getVoltage());
            telemetry.addData("Second Cube", robot.secondCube.getVoltage());

            telemetry.addData("\nTouch Sensors", "");
            telemetry.addData("Cryptobox Detector", robot.cryptoboxDetector.getState());
            telemetry.addData("Extender Out", robot.extenderOut.getState());
            telemetry.addData("Bottom Stop", robot.bottomStop.getState());
            telemetry.addData("Top Stop", robot.topStop.getState());
//            telemetry.addData("Magnet", robot.magnet.getState());
            telemetry.update();
        }
    }
}
