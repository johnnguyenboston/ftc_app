package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by David on 4/7/18.
 */

public class Timer {
    LinearOpMode opMode;
    private long stopTime = 0;
    private long startTime = 0;
    private long time = 0;

    public Timer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void startTimer() {
        startTime = System.currentTimeMillis();
    }

    public void stopTimer() {
        stopTime = System.currentTimeMillis();
        time = stopTime - startTime;
    }

    public long getTime() {
        return time;
    }

    public long getTimeElapsed() {
        return System.currentTimeMillis() - startTime;
    }

    public long getStartTime() {
        return startTime;
    }

    public long getStopTime() {
        return stopTime;
    }

    public void displayTime() {
        opMode.telemetry.addData("time ", time);
        opMode.telemetry.update();
    }

}