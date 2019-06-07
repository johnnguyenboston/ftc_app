package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by August on 4/17/2017.
 */

public class CollectionThread implements Runnable {

    LinearOpMode opMode;

    private double power = 0;

    private boolean twoCubeStop = true;
    private boolean disperse = false;
    private boolean inNout = false;
    private boolean in=true;

    private DcMotor collectorR;
    private DcMotor collectorL;
    private AnalogInput cubeOrienter;
    private AnalogInput rampDetector;
    private AnalogInput secondCube;

    public CollectionThread(LinearOpMode opMode, DcMotor collectorR, DcMotor collectorL, AnalogInput cubeOrienter, AnalogInput rampDetector, AnalogInput secondCube){
        this.opMode = opMode;
        this.collectorR = collectorR;
        this.collectorL = collectorL;
        this.cubeOrienter = cubeOrienter;
        this.rampDetector = rampDetector;
        this.secondCube = secondCube;
        collectorR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public synchronized void setPower(double target){
        disperse = false;
        power=target;
    }
    public synchronized void inNoutSet() {
        inNout = true;
    }

    public synchronized void inNoutSetFalse() {
        inNout = false;
    }

    public synchronized void turnOffTwoCubeStop() {
        twoCubeStop = false;
    }

    public synchronized void turnOnTwoCubeStop() {
        twoCubeStop = true;
    }

    public synchronized void disperse() {
        disperse = true;
    }


    @Override
    public void run() {
        long oldTime = System.currentTimeMillis();
        Timer timer = new Timer(opMode);
        while (!opMode.isStopRequested()){
            boolean rampIsDown = rampDetector.getVoltage() > 2;
            double distance = cubeOrienter.getVoltage();
            boolean twoCubesInHopper = secondCube.getVoltage() > 0.75 && twoCubeStop;
            boolean stopCollection = twoCubesInHopper;
            boolean inCollector = distance > 0.45;

            if (!inCollector) {
                timer.startTimer();
            }
            boolean gettingRidOfCube = timer.getTimeElapsed() > 1000;

            if (rampIsDown && !stopCollection) {
                if (gettingRidOfCube) {
                    collectorR.setPower(-power);
                    collectorL.setPower(-power);
                } else if (distance > 0.45 && distance < 1.6 && power > 0) { //cube orientor
                    collectorR.setPower(-power);
                    collectorL.setPower(power);
                } else {
                    collectorR.setPower(power);
                    collectorL.setPower(power);
                }
            } else {
                collectorR.setPower(0);
                collectorL.setPower(0);
            }
            Thread.yield();
        }
    }
}
