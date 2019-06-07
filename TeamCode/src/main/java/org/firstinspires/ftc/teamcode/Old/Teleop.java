/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Teleop")
//@Disabled
public class Teleop extends LinearOpMode {

    OpMode opMode;

    Robot robot = new Robot(opMode, TeamColor.NOCOLOR);

    float YPower, XPower, rotPower;
    double driveMod = 1;
    int direction = 1, liftZero;
    double liftPower;
    boolean relicLifterIsUp = false, relicIsGripped = false, override = false, gettingOnRamp = false, extendedCollectorOut = true, rampOverride = false;
    int toggleTime = 0, servoStopTime = 0, startY = 0;
    long startTime, startTimePoker;
    boolean pokerRetracted = true;

    double dumpMin;
    double dumpMax;

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
            // Toggles for the column alignment device to extend in and out
            extendCollector();
            // Overrides the distance sensors on the robot
            override();

            retractPoker();
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

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            if (startY == 0) {
                startY = -robot.collectorR.getCurrentPosition();
            }
            gettingOnRamp = true;
        } else {
            gettingOnRamp = false;
            startY = 0;
        }

        if (!gettingOnRamp) {
            robot.FR.setPower(driveMod * -Math.abs(FRpower) * (FRpower));
            robot.FL.setPower(driveMod * Math.abs(FLpower) * (FLpower));
            robot.BR.setPower(driveMod * -Math.abs(BRpower) * (BRpower));
            robot.BL.setPower(driveMod * Math.abs(BLpower) * (BLpower));
        } else {
//            int currentY = -robot.collectorR.getCurrentPosition();
//            int distanceMoved = currentY - startY;
//            if (Math.abs(distanceMoved - DRIVE_ON_RAMP_DISTANCE) < TOLERANCE) {
//                robot.FR.setPower(0);
//                robot.FL.setPower(0);
//                robot.BR.setPower(0);
//                robot.BL.setPower(0);
//            } else if (distanceMoved - DRIVE_ON_RAMP_DISTANCE < 0) { // moving forwards
//                robot.FR.setPower(0.7);
//                robot.FL.setPower(-0.7);
//                robot.BR.setPower(0.7);
//                robot.BL.setPower(-0.7);
//            } else if (distanceMoved - DRIVE_ON_RAMP_DISTANCE > 0) { // moving backwards
//                robot.FR.setPower(0.7);
//                robot.FL.setPower(-0.7);
//                robot.BR.setPower(0.7);
//                robot.BL.setPower(-0.7);
//            }
        }

        if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
            driveMod = 0.7;
        } else {
            driveMod = 1;
        }
    }

    public void collect() {
        boolean rampIsDown = robot.rampDetector.getVoltage() > 2;
        if (rampOverride) {
            rampIsDown = true;
        }
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
        dumpMax = 0.92;
        dumpMin = 0.185;
        double dumpScale = dumpMax - dumpMin;


        if (gamepad2.a) {
            robot.clampR.setPosition(0.65);
            robot.clampL.setPosition(0.65);
        } else {
            robot.clampR.setPosition(0.517);
            robot.clampL.setPosition(0.517);
        }

        robot.dump.setPosition(gamepad2.right_trigger * dumpScale + dumpMin);
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

    public void servoStops() {
        boolean liftIsDown = !robot.bottomStop.getState();
        if (liftIsDown && !gamepad2.a && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
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
        double extenderSpeed = 1.0;
        robot.extender.setPower(extenderSpeed * (gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y));
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

    public void retractPoker() {
        int SERVO_MOVE_TIME = 2000;
        if (gamepad1.dpad_down && !gamepad1.start && toggleTime > 15) {
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
        if (gamepad1.x && toggleTime > 15) {
            override = !override;
            toggleTime = 0;
        }

        if (gamepad1.back && toggleTime > 15) {
            rampOverride = !rampOverride;
            toggleTime = 0;
        }

        telemetry.addData("rampOverride:", rampOverride);
        telemetry.addData("Override:", override);
        telemetry.update();
    }



}
