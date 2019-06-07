package org.firstinspires.ftc.teamcode.Robot;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by August on 10/22/2016.
 */

public class WheelBase {
    LinearOpMode opMode;
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    private double oldFLPower;
    private double oldFRPower;
    private double oldBLPower;
    private double oldBRPower;


    public WheelBase(LinearOpMode varopmode) {

        opMode = varopmode;

        FL = opMode.hardwareMap.dcMotor.get("fl");
        FR = opMode.hardwareMap.dcMotor.get("fr");
        BL = opMode.hardwareMap.dcMotor.get("bl");
        BR = opMode.hardwareMap.dcMotor.get("br");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public WheelBase(OpMode varopmode) {

        opMode = (LinearOpMode) varopmode;

        FL = opMode.hardwareMap.dcMotor.get("fl");
        FR = opMode.hardwareMap.dcMotor.get("fr");
        BL = opMode.hardwareMap.dcMotor.get("bl");
        BR = opMode.hardwareMap.dcMotor.get("br");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    boolean anyMotorBusy(){
        return  FL.isBusy()||FR.isBusy()||BL.isBusy()||BR.isBusy();
    }

    public void runUsingEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setStopMode(DcMotor.ZeroPowerBehavior mode) {
        FL.setZeroPowerBehavior(mode);
        FR.setZeroPowerBehavior(mode);
        BL.setZeroPowerBehavior(mode);
        BR.setZeroPowerBehavior(mode);
    }

    public void runToPosition() {
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runWithoutEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ResetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetDriveEncoders() {
        FL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void setLeftPower(double power) {
        power = Range.clip(power, -1, 1);
        FL.setPower(power);
        BL.setPower(power);
    }

    public void setRightPower(double power) {
        power = Range.clip(power, -1, 1); //because david is trash at building robots and it turns by itself we need to slow this side down, you know it would be so much more effective if you did it in hardware you ass - August
        BR.setPower(power);
        FR.setPower(power);
    }

    private void setAllPowerTo0() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    MotorPowerThread flmotor;
    MotorPowerThread frmotor;
    MotorPowerThread blmotor;
    MotorPowerThread brmotor;
    Thread flthread;
    Thread frthread;
    Thread blthread;
    Thread brthread;

    public void startMotorThreads(){
        flmotor = new MotorPowerThread(FL);
        frmotor = new MotorPowerThread(FR);
        blmotor = new MotorPowerThread(BL);
        brmotor = new MotorPowerThread(BR);

        flthread = new Thread(flmotor);
        frthread = new Thread(frmotor);
        blthread = new Thread(blmotor);
        brthread = new Thread(brmotor);

        flthread.start();
        frthread.start();
        blthread.start();
        brthread.start();
    }


    public void setPowers(double frontLeft, double frontRight, double backLeft, double backRight) throws InterruptedException {
        double speedTolerance = 0.02;
        if (frontLeft == 0 && frontRight == 0 && backLeft == 0 && backRight == 0) {
//            setAllPowerTo0();
        }
        if (!flthread.isInterrupted()&&!frthread.isInterrupted()&&!blthread.isInterrupted()&&!brthread.isInterrupted()){
            if (Math.abs(frontLeft - oldFLPower) > speedTolerance) {
                flmotor.setPower(-frontLeft);
                oldFLPower = frontLeft;
            }
            if (Math.abs(frontRight - oldFRPower) > speedTolerance) {
                frmotor.setPower(-frontRight);
                oldFRPower = frontRight;
            }
            if (Math.abs(backLeft - oldBLPower) > speedTolerance) {
                blmotor.setPower(-backLeft);
                oldBLPower = backLeft;
            }
            if (Math.abs(backRight - oldBRPower) > speedTolerance) {
                brmotor.setPower(-backRight);
                oldBRPower = backRight;
            }
        }
    }

    class MotorPowerThread implements Runnable{
        private DcMotor motor;
        private double power;

        public synchronized void setPower(double power){
            this.power = power;
        }

        public synchronized double getPower(){
            return power;
        }

        public void kill(){

        }

        public MotorPowerThread(DcMotor motor){
            this.motor=motor;
        }
        @Override
        public void run() {
            while (!opMode.isStopRequested()) {
//                Log.d("motors", "power setting");
                motor.setPower(power);
//                opMode.sleep(10);
                Thread.yield();
            }
//            Log.d("motors", "motor thread terminated");
        }
    }

    public void setAllMode (DcMotor.RunMode mode) {
        FR.setMode(mode);
        FL.setMode(mode);
        BR.setMode(mode);
        BL.setMode(mode);
    }

    public void setAllZeroPowerBehavior (DcMotor.ZeroPowerBehavior behavior) {
        FR.setZeroPowerBehavior(behavior);
        FL.setZeroPowerBehavior(behavior);
        BR.setZeroPowerBehavior(behavior);
        BL.setZeroPowerBehavior(behavior);
    }

    public void setAllPositions (int position) {
        FR.setTargetPosition(FR.getCurrentPosition() - position);
        BR.setTargetPosition(BR.getCurrentPosition() - position);
        FL.setTargetPosition(FL.getCurrentPosition() - position);
        BL.setTargetPosition(BL.getCurrentPosition() - position);
    }

    public void setFLPosition (int position) {
        FL.setTargetPosition(FL.getCurrentPosition() - position);
    }

    public void setFRPosition (int position) {
        FR.setTargetPosition(FR.getCurrentPosition() - position);
    }

    public void setBLPosition (int position) {
        BL.setTargetPosition(BL.getCurrentPosition() - position);
    }

    public void setBRPosition (int position) {
        BR.setTargetPosition(BR.getCurrentPosition() - position);
    }


    public int getPosistionFR(){
       return FR.getCurrentPosition();
    }

    public int getPosistionFL(){
        return FL.getCurrentPosition();
    }

    public int getPosistionBR(){
        return BR.getCurrentPosition();
    }

    public int getPosistionBL(){
        return BL.getCurrentPosition();
    }
}
