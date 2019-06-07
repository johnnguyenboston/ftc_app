package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by August on 4/17/2017.
 */

public class LiftThread implements Runnable {

    LinearOpMode opMode;

    private double power = 0;

    private DcMotor lift;
    private DigitalChannel topStop;
    private DigitalChannel bottomStop;
    private AnalogInput cubeOrienter;
    int startPosition;
    int position;
    boolean liftToPosition = false;
    boolean liftIsSetUp = false;
    boolean hasBeenSet = false;

    public LiftThread(LinearOpMode opMode, DcMotor lift, DigitalChannel topStop, DigitalChannel bottomStop, AnalogInput cubeOrienter){
        this.opMode = opMode;
        this.lift = lift;
        this.topStop = topStop;
        this.bottomStop = bottomStop;
        this.cubeOrienter = cubeOrienter;
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.startPosition = lift.getCurrentPosition();
    }

    public synchronized void liftUp() {
        power = 1;
        liftIsSetUp = true;
        liftToPosition = false;
    }

    public synchronized void liftToPostion(int position) {
        liftToPosition = true;
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.position = position;
        hasBeenSet = false;
    }


    public synchronized void liftDown() {
        power = -1;
        liftIsSetUp = false;
        liftToPosition = false;
    }

    public synchronized void restLift() {
        power = 0;
        liftIsSetUp = false;
        liftToPosition = false;
    }

    @Override
    public void run() {
        while (!opMode.isStopRequested()){
            if (!bottomStop.getState()) {
                startPosition = lift.getCurrentPosition();
            }
            if (liftToPosition) {
                if (!hasBeenSet) {
                    lift.setTargetPosition(startPosition + position);
                    hasBeenSet = true;
                }
                lift.setPower(0.1);
            } else {
                double distance = cubeOrienter.getVoltage();
                boolean cubesAreClear = distance < 0.4;
                if (cubesAreClear) {
                    if (power < 0 && bottomStop.getState()) {
                        lift.setPower(power);
                    } else if (power > 0 && topStop.getState()) {
                        lift.setPower(power);
                    } else {
                        if (liftIsSetUp) {
                            lift.setPower(0.1);
                        } else {
                            lift.setPower(0);
                        }
                    }
                }
            }
            opMode.sleep(100);
        }
    }
}
