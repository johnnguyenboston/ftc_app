package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;

/**
 * Created by David on 3/9/18.
 */
//MAALS=mark and august's location system
public class MAALSRunner implements Runnable {
    LinearOpMode opMode;
    AutonomousRobot autonomousRobot;

    public MAALSRunner(LinearOpMode opMode, AutonomousRobot autonomousRobot) {
        this.opMode = opMode;
        this.autonomousRobot = autonomousRobot;
        oldEncoderX1 = -autonomousRobot.collectorL.getCurrentPosition(); //x encoder 1
        oldEncoderX2 = -autonomousRobot.extender.getCurrentPosition(); //x encoder 2
        oldEncoderY = -autonomousRobot.collectorR.getCurrentPosition(); //y encoder
    }

    public final boolean useTwoXEncoders = true;

    private double X = 0; //absolute x location
    private double Y = 0; //absolute y location
    private double twoEncoderRadiusX = -3900;
    private double radiusY = 1205; //distance between y encoder and center of robot - NOTE TO AUGUST: THIS IS EQUIVALENT TO ENCODER CLICKS PER RADIAN TURNED
    private double oldEncoderX1; //x encoder 1
    private double oldEncoderX2; //x encoder 2
    private double oldEncoderY; //y encoder
    private double newEncoderX1;
    private double newEncoderX2;
    private double newEncoderY;
    private double startEncoderX1;
    private double startEncoderX2;
    private double startEncoderY;
    private double deltaEncoderX1; //change in x1 encoder over a given cycle
    private double deltaEncoderX2; //change in x2 encoder over a given cycle
    private double deltaEncoderY; //change in y encoder over a given cycle
    private double forwardSpeed; //y velocity WITH RESPECT TO ROBOT POSITION
    private double sidewaysSpeed; //x velocity WITH RESPECT TO ROBOT POSITION
    private double sidewaysMovement1 = 0;
    private double sidewaysMovement2 = 0;
    private double forwardMovement = 0;
    private double adjustedForwardMovement = 0;
    private double time = 0;
    private double oldTime = 0;
    private double oldHeadingRad = 0;
    private double newGyroHeading;
    private double headingOffset;


    public double headingRad() {
        return (totalHeadingRad()-headingOffset) % (2 * Math.PI);
    }

    public double totalHeadingRad() {
        double encoderDifferenceX1 = newEncoderX1 - startEncoderX1;
        double encoderDifferenceX2 = newEncoderX2 - startEncoderX2;
        return ((encoderDifferenceX1 - encoderDifferenceX2) / (twoEncoderRadiusX));
    }

    private double headingRadians;

    public double deltaTime() {
        return time - oldTime;
    }

    public double deltaHeadingRadians() {
        double change = headingRad() - oldHeadingRad;
        if (change > Math.PI) {
            return change - 2 * Math.PI;
        } else if (change < -Math.PI) {
            return change + 2 * Math.PI;
        } else {
            return change;
        }
    }


    public void setRadii(double xTwoEncoder, double y) {
        twoEncoderRadiusX = xTwoEncoder;
        radiusY = y;
    }

    public double getSidewaysSpeed() {
        return sidewaysSpeed;
    }

    public double getForwardSpeed() {
        return forwardSpeed;
    }

    public int getX() {
        return (int) X;
    }

    public int getY() {
        return (int) Y;
    }

    public double getSideways1() {
        return sidewaysMovement1;
    }

    public double getSideways2() {
        return sidewaysMovement2;
    }

    public double getForward() {
        return forwardMovement;
    }

    public double getAdjustedForward() {
        return adjustedForwardMovement;
    }

    public void setX(int X) {
        this.X = X;
    }

    public void setY(int Y) {
        this.Y = Y;
    }

    public void setHeading(double heading){ //HEADING IS RADIANS
        headingOffset = totalHeadingRad()-heading;
    }

    public int getEncoderX1() {
        return (int) newEncoderX1;
    }

    public int getEncoderX2() {
        return (int) newEncoderX2;
    }

    public int getEncoderY() {
        return (int) newEncoderY;
    }

    public double getRawHeading() {
        return newGyroHeading;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(headingRad());
    }

    public void setLocation(int X, int Y) {
        this.X = X;
        this.Y = Y;
    }

    public void setLocation(int X, int Y, double heading) { //HEADING IS RADIANS
        this.X = X;
        this.Y = Y;
        setHeading(heading);
    }

    private double time1 = 10;
    private double time2 = 20;
    private double time3 = 0;

    public double timeToReadValues() {
        return time2;
    }

    public double timeToDoCalulations() {
        return time3 - time2;
    }

    @Override
    public void run() {
        Thread.currentThread().setPriority(6);
        startEncoderX1 = -autonomousRobot.collectorL.getCurrentPosition(); //encoders flipped to decrease degeneracy
        startEncoderX2 = -autonomousRobot.extender.getCurrentPosition();
        startEncoderY = -autonomousRobot.collectorR.getCurrentPosition();

        Thread gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!opMode.isStopRequested()) {
                    long startTime = System.currentTimeMillis();
                    newGyroHeading = autonomousRobot.imu1.getAngularOrientation().firstAngle;
//                    opMode.telemetry.addData("Gyro time ", System.currentTimeMillis() - startTime);
//                    opMode.telemetry.update();
                    Thread.yield(); // NEVER EVER REMOVE THIS
                }
            }
        });
        gyroThread.setPriority(7);
        gyroThread.start();

        while (!opMode.isStopRequested()) {
            oldHeadingRad = headingRad();
            oldTime = time;
            time = System.currentTimeMillis();

            newEncoderX1 = -autonomousRobot.collectorL.getCurrentPosition(); //encoders flipped to decrease degeneracy
            newEncoderX2 = -autonomousRobot.extender.getCurrentPosition();
            newEncoderY = autonomousRobot.collectorR.getCurrentPosition();

//            rotationRate = deltaHeadingRadians()/deltaTime();
            //            deltaHeadingRadians = calcDeltaHeadingRadians();
            time1 = time;
            time2 = System.currentTimeMillis() - time1;
            deltaEncoderX1 = newEncoderX1 - oldEncoderX1;

//            opMode.telemetry.addData("encoderTime", time2);
//            opMode.telemetry.update();

            deltaEncoderX2 = newEncoderX2 - oldEncoderX2;

//          --------------------------------------------------------------
//            deltaEncoderX2 = deltaEncoderX1; //REMOVE WHEN ENCODER IS FIXED!
//          --------------------------------------------------------------
            deltaEncoderY = newEncoderY - oldEncoderY;

            if (deltaEncoderX1 == 0 && deltaEncoderX2 == 0 && deltaEncoderY == 0) {
                continue;
            }


            //deltaTime = (System.currentTimeMillis() / 1000.0) - oldTime;

            //sidewaysSpeed = ((deltaEncoderX1 + deltaEncoderX2) / 2) - (rotationSpeed * oneEncoderRadiusX * deltaTime);
            //forwardSpeed = deltaEncoderY - (rotationSpeed * radiusY * deltaTime);
            sidewaysSpeed = ((deltaEncoderX1 + deltaEncoderX2) / 2) - (deltaHeadingRadians() * (twoEncoderRadiusX/(2*Math.PI)));


            forwardSpeed = deltaEncoderY - (deltaHeadingRadians() * (radiusY/(2*Math.PI)));

            X += sidewaysSpeed * Math.cos(headingRad()) - forwardSpeed * Math.sin(headingRad());
            Y += sidewaysSpeed * Math.sin(headingRad()) + forwardSpeed * Math.cos(headingRad());

            sidewaysMovement1 += deltaEncoderX1;
            sidewaysMovement2 += deltaEncoderX2;
            forwardMovement += deltaEncoderY;
            adjustedForwardMovement += forwardSpeed;


            oldEncoderX1 = newEncoderX1;
            oldEncoderX2 = newEncoderX2;
            oldEncoderY = newEncoderY;

//            opMode.telemetry.addData("X", X);
//            opMode.telemetry.addData("Y", Y);
//            opMode.telemetry.update();

            time3 = System.currentTimeMillis();
            Thread.yield(); // NEVER EVER REMOVE THIS
        }
    }
}