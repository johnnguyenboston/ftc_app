package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 12/8/17.
 */

@TeleOp(name="Teleop Dumb", group="Teleop")
//@Disabled
public class TeleopDumb extends LinearOpMode {

    OpMode opMode;

    Robot robot = new Robot(opMode, TeamColor.NOCOLOR);

    float YPower, XPower, rotPower;
    double driveMod = 1;
    int direction = 1, liftZero;
    double liftPower;
    boolean relicLifterIsUp = false, relicIsGripped = false;
    int toggleTime = 0, lastExtentionValue = 0, servoStopTime = 0;


    @Override
    public void runOpMode() {

        robot.initTele(hardwareMap);
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Drive control
            drive();
            // Collection control
            collect();
            // Clamping and dumping the cubes
            clampAndDump();
            // Lift control
            lift();
            // Setting the rear servo stop positions
            servoStops();
            // Controlling the jewel knocker to get balls out of the cryptobox
            jewelKnocker();
            // Gripping and lifting the relic
            relicGrippingAndLifting();
            // Extender control
            extender();
        }
    }

    public void drive() {
        float YVal = direction * -gamepad1.left_stick_y;
        float XVal = direction * -gamepad1.left_stick_x;
        float RotVal = gamepad1.right_stick_x / 1;

        // clip the right/left values so that the   values never exceed +/- 1
        YPower = Range.clip(YVal, -1, 1);
        XPower = Range.clip(XVal, -1, 1);
        rotPower = Range.clip(RotVal, -1, 1);

        //combine values
        double FRpower = YPower + XPower - rotPower;
        double BRpower = YPower - XPower - rotPower;
        double FLpower = YPower - XPower + rotPower;
        double BLpower = YPower + XPower + rotPower;

        //clip power to acceptable levels
        FRpower = Range.clip(FRpower, -1, 1);
        FLpower = Range.clip(FLpower, -1, 1);
        BRpower = Range.clip(BRpower, -1, 1);
        BLpower = Range.clip(BLpower, -1, 1);

        // set power
        robot.FR.setPower(driveMod * Math.abs(FRpower) * (FRpower));
        robot.FL.setPower(driveMod * -Math.abs(FLpower) * (FLpower));
        robot.BR.setPower(driveMod * Math.abs(BRpower) * (BRpower));
        robot.BL.setPower(driveMod * -Math.abs(BLpower) * (BLpower));

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            driveMod = 0.7;
        } else {
            driveMod = 1;
        }
    }

    public void collect() {
        if ((gamepad1.right_bumper || gamepad2.right_bumper) && (!robot.bottomStop.getState()) && robot.dump.getPosition() < 0.2) {
            robot.collectorR.setPower(1);
            robot.collectorL.setPower(1);
        } else if (gamepad1.left_bumper|| gamepad2.left_bumper) {
            robot.collectorR.setPower(-1);
            robot.collectorL.setPower(-1);
        } else {
            robot.collectorR.setPower(0);
            robot.collectorL.setPower(0);
        }
    }

    public void clampAndDump() {
        if (gamepad2.a) {
            robot.clampR.setPosition(0.65);
            robot.clampL.setPosition(0.65);
        } else {
            robot.clampR.setPosition(0.51);
            robot.clampL.setPosition(0.51);
        }

        if (gamepad2.right_trigger != 0) {
            robot.dump.setPosition(0.9);//0.7
            robot.dump1.setPosition(0.9);// 0.775
        } else {
            robot.dump.setPosition(0.17); // 0
            robot.dump1.setPosition(0.17); // 0.075
        }
    }

    public void lift() {
        if (gamepad2.dpad_up && robot.topStop.getState()) {
            robot.lift.setPower(1);
        } else if (gamepad2.dpad_down && robot.bottomStop.getState()) {
            if (robot.lift.getCurrentPosition() - liftZero < -1000)
                liftPower = -0.1;
            else
                liftPower = -1;
            robot.lift.setPower(liftPower);
        } else {
            robot.lift.setPower(0);
        }

        if (!robot.topStop.getState()) {
            liftZero = robot.lift.getCurrentPosition();
        }
    }

    public void servoStops() {
        if (!robot.bottomStop.getState() && !gamepad2.a && gamepad2.right_trigger == 0) {
            if (servoStopTime > 20) {
                robot.rightServoStop.setPosition(0.5);
                robot.leftServoStop.setPosition(0.5);
            }
        } else {
            robot.rightServoStop.setPosition(0);
            robot.leftServoStop.setPosition(0);
            servoStopTime = 0;
        }
        servoStopTime ++;
    }

    public void jewelKnocker() {
        if (gamepad1.b) {
            robot.jewelArm.setPosition(0);
            robot.jewelPivot.setPosition(0.5);
        } else {
            robot.jewelArm.setPosition(1);
            robot.jewelPivot.setPosition(1);
        }
    }

    public void relicGrippingAndLifting() {
        toggleTime ++;
        if (gamepad2.b && !gamepad2.start && toggleTime > 15){
            relicLifterIsUp = !relicLifterIsUp;
            toggleTime = 0;
        }

        if (relicLifterIsUp) {
            robot.relicLifter.setPosition(1);
//        } else if (!relicLifterIsUp && !relicIsGripped) {
//            if (robot.extender.getPower() == 0)
//                 robot.relicLifter.setPosition(0.05);
        } else {
            robot.relicLifter.setPosition(0.05);
        }

        if (gamepad2.x && toggleTime > 15){
            relicIsGripped = !relicIsGripped;
            toggleTime = 0;
        }
        if (relicIsGripped) {
            robot.relicGripper.setPosition(1);
        } else {
            robot.relicGripper.setPosition(0.4);
        }

        if (relicIsGripped && gamepad2.left_stick_y > 0) {
            relicLifterIsUp = true;
        }
    }

    public void extender() {
        if (!relicIsGripped) {
            robot.extender.setPower(0.6 * (gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y));
        } else {
            robot.extender.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y));
        }
        setLastExtentionValue();
    }

    public void setLastExtentionValue() {
        if (robot.extender.getPower() < 0) {
            lastExtentionValue = -1;
        } else if (robot.extender.getPower() > 0){
            lastExtentionValue = 1;
        }
    }



}
