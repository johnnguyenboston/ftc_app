package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

/**
 * Created by David on 1/10/18.
 */

@TeleOp(name="Teleop One Person", group="Teleop")
//@Disabled
public class TeleopOnePerson extends LinearOpMode {

    OpMode opMode;

    Robot robot = new Robot(opMode, TeamColor.NOCOLOR);

    float YPower, XPower, rotPower;
    double driveMod = 1;
    int direction = 1, liftZero;
    double liftPower;
    boolean relicLifterIsUp = false, relicIsGripped = false, override = false;
    boolean clamped = false, angled = false, liftUp = false;
    int toggleTime = 0, lastExtentionValue = 0, servoStopTime = 0;
    boolean pokerRetracted = false;
    long startTimePoker = System.currentTimeMillis();
    boolean extendedCollectorOut = false;
    long startTime = System.currentTimeMillis();


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
            // Overrides the distance sensors on the robot
            override();
            reset();
            retractPoker();

            extendCollector();
            toggleTime ++;
        }
    }

    public void drive() {
        int DRIVE_ON_RAMP_DISTANCE = -1500;
        int TOLERANCE = 500;

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
        //Square values and preserve sign
        robot.FR.setPower(driveMod * -Math.abs(FRpower) * (FRpower));
        robot.FL.setPower(driveMod * Math.abs(FLpower) * (FLpower));
        robot.BR.setPower(driveMod * -Math.abs(BRpower) * (BRpower));
        robot.BL.setPower(driveMod * Math.abs(BLpower) * (BLpower));

        if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
            driveMod = 0.7;
        } else {
            driveMod = 1;
        }
    }

    public void extendCollector() {
        if ((gamepad1.y || gamepad2.y) && toggleTime > 10) {
            toggleTime = 0;
            extendedCollectorOut = !extendedCollectorOut;
            startTime = System.currentTimeMillis();
        }
        if (extendedCollectorOut) {
            robot.collectionDeployR.setPosition(1);
            robot.collectionDeployL.setPosition(1);
        } else {
            robot.collectionDeployR.setPosition(0);
            robot.collectionDeployL.setPosition(0);
        }
    }

    public void collect() {
        boolean rampIsDown = robot.rampDetector.getVoltage() > 2;
        boolean liftIsDown = !robot.bottomStop.getState();
        if ((gamepad1.right_bumper || gamepad2.right_bumper)){
            double distance = robot.cubeOrienter.getVoltage();
            if ((liftIsDown && rampIsDown) || override) {
                if (distance > 0.45 && distance < 1.6) {
                    if (extendedCollectorOut) {
                        robot.collectorR.setPower(-1);
                        robot.collectorL.setPower(1);
                    } else {
                        robot.collectorR.setPower(0);
                        robot.collectorL.setPower(1);
                    }
                } else {
                    robot.collectorR.setPower(1);
                    robot.collectorL.setPower(1);
                }
            }
        } else if (gamepad1.left_bumper|| gamepad2.left_bumper) {
            robot.collectorR.setPower(-1);
            robot.collectorL.setPower(-1);
        } else if (gamepad2.dpad_right) {
            robot.collectorR.setPower(-1);
            robot.collectorL.setPower(1);
        } else if (gamepad2.dpad_left) {
            robot.collectorR.setPower(1);
            robot.collectorL.setPower(-1);
        } else {
            robot.collectorR.setPower(0);
            robot.collectorL.setPower(0);
        }
    }

    public void clampAndDump() {
        double dumpMax = 0.92;
        double dumpMin = 0.185;
        double dumpScale = dumpMax - dumpMin;

        if (gamepad1.right_stick_button && toggleTime > 15) {
            clamped = !clamped;
            toggleTime = 0;
        }

        if (gamepad1.left_stick_button && toggleTime > 15) {
            angled = !angled;
            toggleTime = 0;
        }

        if (clamped) {
            robot.clampR.setPosition(0.65);
            robot.clampL.setPosition(0.65);
        } else {
            robot.clampR.setPosition(0.51);
            robot.clampL.setPosition(0.51);
        }

        if (angled) {
            robot.dump.setPosition(dumpMax);//0.7
        } else {
            robot.dump.setPosition(dumpMin); // 0
        }
    }

    public void lift() {
        boolean liftIsDown = !robot.bottomStop.getState();
        if ((gamepad2.dpad_up || gamepad1.dpad_up) && robot.topStop.getState()) {
            robot.lift.setPower(1);
        } else if ((gamepad2.dpad_down || gamepad1.dpad_down) && !liftIsDown) {
            if (robot.lift.getCurrentPosition() - liftZero < -1000)
                liftPower = -0.3;
            else if (robot.rampDetector.getVoltage() > 2) {
                liftPower = -0.1;
            } else {
                liftPower = -1;
            }
            robot.lift.setPower(liftPower);
        } else {
            robot.lift.setPower(0);
        }
    }

    public void reset() {
        if (gamepad1.x) {
            liftUp = false;
            clamped = false;
            angled = false;
            toggleTime = 0;
        }
    }

    public void servoStops() {
        boolean liftIsDown = !robot.bottomStop.getState();
        if (liftIsDown && !clamped && !angled) {
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
        robot.jewelArm.setPosition(0.85);
        robot.jewelPivot.setPosition(0.5);
    }

    public void relicGrippingAndLifting() {
        boolean extenderIsOut = !robot.extenderOut.getState();

        if (gamepad2.b && !gamepad2.start && toggleTime > 15){
            relicLifterIsUp = !relicLifterIsUp;
            toggleTime = 0;
        }

        if (relicLifterIsUp) {
            robot.relicLifter.setPosition(1);
        } else if (extenderIsOut && relicIsGripped) {
            robot.relicLifter.setPosition(0);
        } else {
            robot.relicLifter.setPosition(0);
        }

        if (gamepad2.x && toggleTime > 15){
            if (!relicLifterIsUp) {
                relicIsGripped = !relicIsGripped;
            }
            toggleTime = 0;
        }
        if (relicIsGripped) {
            robot.relicGripper.setPosition(1);
        } else {
            robot.relicGripper.setPosition(0.4);
        }

        if (relicIsGripped && gamepad2.left_stick_y > 0 && robot.relicGripper.getPosition() > 0.7) {
            relicLifterIsUp = true;
        }
    }

    public void extender() {
        robot.extender.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y));
    }

    public void retractPoker() {
        int SERVO_MOVE_TIME = 2000;
        if (gamepad1.dpad_right && toggleTime > 15) {
            toggleTime = 0;
            pokerRetracted = !pokerRetracted;
            startTimePoker = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - startTimePoker > SERVO_MOVE_TIME) {
            robot.cubeAngler.setPower(0);
        } else {
            if (pokerRetracted)
                robot.cubeAngler.setPower(0.8);
            else
                robot.cubeAngler.setPower(-0.8);
        }
    }

    public void override() {
        if (gamepad1.back && toggleTime > 15) {
            override = !override;
            toggleTime = 0;
        }

        if (override) {
            telemetry.addData("Override:", "On");
            telemetry.update();
        } else {
            telemetry.addData("", "");
            telemetry.update();
        }

    }
}