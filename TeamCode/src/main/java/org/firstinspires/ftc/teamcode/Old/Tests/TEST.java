package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by David on 3/8/18.
 */
@Disabled
@Autonomous(name = "Test")
public class TEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.colorSensor.get("color");
//        DcMotor FR = hardwareMap.dcMotor.get("fr");

//        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
//        FR.setTargetPosition(0);

//        FR.setPower(-1);

        while (!isStopRequested()) {
            telemetry.addData("color", color.alpha());
            telemetry.update();
            sleep(100);
        }
    }
}
