package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;

/**
 * Created by David on 3/9/18.
 */
//MAALS=mark and august's location system
public class PositionTracker implements Runnable {
    LinearOpMode opMode;
    AutonomousRobot autonomousRobot;

    public PositionTracker(LinearOpMode opMode, AutonomousRobot autonomousRobot) {
        this.opMode = opMode;
        this.autonomousRobot = autonomousRobot;
        oldEncoderX1 = -autonomousRobot.collectorL.getCurrentPosition(); //x encoder 1
        oldEncoderX2 = -autonomousRobot.extender.getCurrentPosition(); //x encoder 2
        oldEncoderY = autonomousRobot.collectorR.getCurrentPosition(); //y encoder
        X = 0;
        Y = 0;
    }

    private double X = 0; //absolute x location
    private double Y = 0; //absolute y location
    private double oldEncoderX1; //x encoder 1
    private double oldEncoderX2; //x encoder 2
    private double oldEncoderY; //y encoder
    private double newEncoderX1;
    private double newEncoderX2;
    private double newEncoderY;
    private double deltaEncoderX1; //change in x1 encoder over a given cycle
    private double deltaEncoderX2; //change in x2 encoder over a given cycle
    private double deltaEncoderY; //change in y encoder over a given cycle
    private double totalEncoderDeltaX1 = 0;
    private double totalEncoderDeltaX2 = 0;
    private double totalEncoderDeltaY = 0;
    private int zeroDeltaCountX1 = 0;
    private int zeroDeltaCountX2 = 0;
    private double newGyroHeading;
    public int count;
    private boolean X1Disabled = false, X2Disabled = false, YDisabled = false, YJustChanged = false;



    public double getHeading() {
        return newGyroHeading;
    }

    public int getX() {
        return (int) X;
    }

    public int getY() {
        return (int) Y;
    }

    public double getSideways1() {
        return totalEncoderDeltaX1;
    }

    public double getSideways2() {
        return totalEncoderDeltaX2;
    }

    public double getSideways() {
        return (totalEncoderDeltaX1 + totalEncoderDeltaX2) / 2;
    }

    public double getForward() {
        return totalEncoderDeltaY;
    }

    public void setX(int X) {
        this.X = X;
    }

    public void setY(int Y) {
        this.Y = Y;
    }

    public int getEncoderX1() {
        return (int) newEncoderX1;
    }

    public int getEncoderX2() {
        return (int) newEncoderX2;
    }

    public int getEncoderY() {
        if (!YDisabled) {
            return (int) newEncoderY;
        } else {
            return (int) (7.39*newEncoderY);
        }
    }

    public void setEncoderX1AsBroken() {
        X1Disabled = true;
    }
    public void setEncoderX2AsBroken() {
        X2Disabled = true;
    }

    public void setEncoderYAsBroken() {
        YDisabled = true;
        YJustChanged = true;
    }

    public double getYDelta () {
        return deltaEncoderY;
    }

    public int getZeroDeltaCountX1 () {
        return zeroDeltaCountX1;
    }

    public int getZeroDeltaCountX2 () {
        return zeroDeltaCountX2;
    }

    public boolean isX1Working() {
        return !X1Disabled;
    }
    public boolean isX2Working() {
        return !X2Disabled;
    }
    public boolean isYWorking() {
        return !YDisabled;
    }

    public void setLocation(int X, int Y) {
        this.X = X;
        this.Y = Y;
    }

    @Override
    public void run() {
        Thread.currentThread().setPriority(6);
        Thread gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!opMode.isStopRequested()) {
                    newGyroHeading = autonomousRobot.imu1.getAngularOrientation().firstAngle;
                    Thread.yield();
                }
            }
        });
        gyroThread.setPriority((Thread.MAX_PRIORITY + Thread.NORM_PRIORITY) / 2);
        gyroThread.start();

        int X1DisabledCount = 0, X2DisabledCount = 0;
        while (!opMode.isStopRequested()) {
            newEncoderX1 = -autonomousRobot.collectorL.getCurrentPosition(); //encoders flipped to decrease degeneracy
            newEncoderX2 = -autonomousRobot.extender.getCurrentPosition();
            if (!YDisabled) {
                newEncoderY = autonomousRobot.collectorR.getCurrentPosition();
            } else {
                newEncoderY = -autonomousRobot.BR.getCurrentPosition();
                if (YJustChanged) {
                    oldEncoderY = newEncoderY;
                    YJustChanged = false;
                }
            }

            deltaEncoderX1 = newEncoderX1 - oldEncoderX1;
            deltaEncoderX2 = newEncoderX2 - oldEncoderX2;
            if (!YDisabled) {
                deltaEncoderY = newEncoderY - oldEncoderY;
            } else {
                deltaEncoderY = 7.39*(newEncoderY - oldEncoderY);
            }

            if (deltaEncoderX1 == 0) {
                zeroDeltaCountX1 ++;
            }
            if (deltaEncoderX2 == 0) {
                zeroDeltaCountX2 ++;
            }
//
            opMode.telemetry.addData("X2 Working", !X2Disabled);
            opMode.telemetry.addData("X1 Working", !X1Disabled);
            opMode.telemetry.addData("Y Working", !YDisabled);
//            opMode.telemetry.addData("deltaX1", deltaEncoderX1);
//            opMode.telemetry.addData("deltaX2", deltaEncoderX2);
//            opMode.telemetry.addData("deltaY", deltaEncoderY);
            opMode.telemetry.addData("X", X);
            opMode.telemetry.addData("Y", Y);
            opMode.telemetry.update();
//
//            if (!X1Disabled && !X2Disabled && deltaEncoderX1 != 0 && deltaEncoderX2 != 0) {
//                X += (deltaEncoderX1 + deltaEncoderX2) / 2;
//            } else {
//                if (X1Disabled || (deltaEncoderX1 == 0 && !X2Disabled))  {
//                    X+= deltaEncoderX2;
//                } else if (X2Disabled || (deltaEncoderX2 == 0 && !X1Disabled)){
//                    X+= deltaEncoderX1;
//                }
//            }

            if (!X1Disabled && !X2Disabled) {
                X += (deltaEncoderX1 + deltaEncoderX2) / 2;
            } else {
                if (X1Disabled)  {
                    X+= deltaEncoderX2;
                } else if (X2Disabled){
                    X+= deltaEncoderX1;
                }
            }
            Y += deltaEncoderY;

            totalEncoderDeltaX1 += deltaEncoderX1;
            totalEncoderDeltaX2 += deltaEncoderX2;
            totalEncoderDeltaY += deltaEncoderY;

            oldEncoderX1 = newEncoderX1;
            oldEncoderX2 = newEncoderX2;
            oldEncoderY = newEncoderY;
            count ++;
            Thread.yield();
        }
    }
}

