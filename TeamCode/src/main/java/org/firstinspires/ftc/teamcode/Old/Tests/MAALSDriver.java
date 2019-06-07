package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.AutonomousRobot;
import org.opencv.core.Mat;

/**
 * Created by August on 4/17/2017.
 */

public class MAALSDriver implements Runnable {

    LinearOpMode opMode;
    AutonomousRobot autonomousRobot;
    MAALSRunner maalsRunner;


//    private static double xkp = 0.000000;
//    private static double xki = 0.000000;
//    private static double xkd = 0.000000;
//
//    private static double ykp = 0.000400;
//    private static double yki = 0.000200;
//    private static double ykd = 0.001000;
//
//    private static double hkp = 0.100000;
//    private static double hki = 0.000000;
//    private static double hkd = 0.000000;

    private static double xkp = 0;
    private static double xki = 0;
    private static double xkd = 0;

    private static double ykp = 0.000400;
    private static double yki = 0;
    private static double ykd = 0;

    private static double hkp = 0.100000;
    private static double hki = 0;
    private static double hkd = 0;


    private double targetX;
    private double targetY;
    private double targetHeadingRadians;

    private double targetSpeedSideways;
    private double targetSpeedForwards;


    private double robotX;
    private double robotY;
    private double robotHeadingRadians;

    private double deltaX;
    private double deltaY;

    private long lastTime;
    private long time;

    public double deltaTime() {
        return (time - lastTime);
    }

    private double lastDeltaX;
    private double lastDeltaY;

    private double lastDeltaXMagnitude() {
        return Math.abs(lastDeltaX);
    }

    private double lastDeltaYMagnitude() {
        return Math.abs(lastDeltaY);
    }

    private double deltaHeadingRadians; //how much to turn
    private double lastDeltaHeadingRadians;
    private double deltaHeadingRadiansSum = 0;

    private double moveHeading; //heading to move RELATIVE TO ROBOT
    private double moveSpeed = 0.35; //constant to set how fast to go
    private double moveDist = Double.MAX_VALUE;

    private double moveDistSideways;
    private double lastMoveDistSideways;
    private double moveSideWaysRate(){
        return (moveDistSideways-lastMoveDistSideways)/deltaTime();
    }
    private double moveSidewaysError(){
        return targetSpeedSideways-moveSideWaysRate();
    }
    private double moveSidewaysErrorSum = 0;
    private double lastMoveSidewaysError = 0;

    private double moveDistForwards;
    private double lastMoveDistForwards;
    private double moveForwardsRate(){
        return (moveDistForwards - lastMoveDistForwards)/deltaTime();
    }
    private double moveForwardsError(){
        return targetSpeedForwards-moveForwardsRate();
    }
    private double moveForwardsErrorSum = 0; //starting speed
    private double lastMoveForwardsError = 0;

    double XPower = 0;
    double YPower = 0;
    double rotPower = 0;

    double FRPower = 0;
    double BRPower = 0;
    double FLPower = 0;
    double BLPower = 0;

    double highestWheelPower = 0;

    public double getXPower() {
        return XPower;
    }

    public double getYPower() {
        return YPower;
    }

    public double getMoveDist() {
        return moveDist;
    }

    public MAALSDriver(LinearOpMode opMode, AutonomousRobot autonomousRobot, MAALSRunner maalsRunner) {
        this.opMode = opMode;
        this.maalsRunner = maalsRunner;
        this.autonomousRobot = autonomousRobot;
    }

    public void setTargetX(double x) {
        targetX = x;
    }

    public void setTargetY(double y) {
        targetY = y;
    }

    public void setTargetHeading(double h) {
        targetHeadingRadians = h;
    }

    public void setTarget(double x, double y, double h) {
        targetX = x;
        targetY = y;
        targetHeadingRadians = h;
        moveDist = Double.MAX_VALUE;
    }

    public double getMoveDistSideways() { return moveDistSideways; }
    public double getMoveDistForwards() { return moveDistForwards; }


    @Override
    public void run() {
        maalsRunner.setLocation(0, 0);
        time = System.currentTimeMillis();
        autonomousRobot.wheelBase.setStopMode(DcMotor.ZeroPowerBehavior.FLOAT);
        //autonomousRobot.wheelBase.setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!opMode.isStopRequested()) {

            //Get Robot's location and heading from MAALSRunner
            robotX = maalsRunner.getX();
            robotY = maalsRunner.getY();
            robotHeadingRadians = maalsRunner.headingRad();
            lastTime=time;
            time = System.currentTimeMillis();
            if (deltaTime()==0){
                lastTime=System.currentTimeMillis();
                continue;
            }

            //Figure out how much the robot needs to move in each dimension and how many degrees to turn
            deltaX = targetX - robotX;
            deltaY = targetY - robotY;
            deltaHeadingRadians = targetHeadingRadians - robotHeadingRadians;
            moveDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            //Set the direction (relative to the robot) that it needs to move
            if (deltaY == 0) {
                moveHeading = ((Math.PI / 2) * -Math.signum(deltaX)) - robotHeadingRadians;
            } else if (deltaY > 0) {
                moveHeading = -Math.atan(deltaX / deltaY) - robotHeadingRadians;
            } else {
                moveHeading = Math.PI - Math.atan(deltaX / deltaY) - robotHeadingRadians;
            }

            //Set much it should move in each dimension relative to robot
            moveDistSideways = moveDist * Math.sin(moveHeading);
            moveDistForwards = moveDist * Math.cos(moveHeading);

            //https://www.desmos.com/calculator/6az70mk6fi
//            final double ax = 150; //Smoothing factor > is more gradual
//            final double bx = 300; //Shift over > is slows sooner
//            final double cx = 200; //Max encoder ticks per second target
//            targetSpeedSideways = Math.signum(moveDistSideways)*(((Math.atan((1.0/ax) * (Math.abs(moveDistSideways) - bx))*cx) / Math.PI) + (0.5*cx));
//
//            final double ay = 150; //Smoothing factor > is more gradual
//            final double by = 300; //Shift over > is slows sooner
//            final double cy = 200; //Max encoder ticks per second target
//            targetSpeedForwards = Math.signum(moveDistForwards)*(((Math.atan((1.0/ay) * (Math.abs(moveDistForwards) - by))*cy) / Math.PI) + (0.5*cy));

            //
            final double ay = 1600;
            final double by = 700;
            targetSpeedForwards = ay*((1.0/(1+Math.pow(2,-moveDistForwards*(1/by))))-0.5);

            final double ax = 2000;
            final double bx = 500;
            targetSpeedSideways = ay*((1.0/(1+Math.pow(2,-moveDistSideways*(1/by))))-0.5);


            XPower = 0;
            YPower = 0;
            rotPower = 0;

            if (Math.abs(moveDistForwards) > 100) {
                YPower = Math.signum(moveForwardsError())*0.0;
                YPower +=  moveForwardsError() * ykp;
                YPower += moveForwardsErrorSum * yki;
                YPower += ((moveForwardsError() - lastMoveForwardsError)/deltaTime()) * ykd; //mork do this
            }

            if (Math.abs(moveDistSideways) > 100) {
                XPower = Math.signum(moveSidewaysError())*0.0;
                XPower += moveSidewaysError() * xkp;
                XPower += moveSidewaysErrorSum * xki;
                XPower += ((moveSidewaysError() - lastMoveSidewaysError)/deltaTime()) * xkd;
            }

            if (Math.abs(deltaHeadingRadians) > (Math.PI / 180) * 5) {
                rotPower = deltaHeadingRadians * hkp;
                rotPower += deltaHeadingRadiansSum * hki;
                rotPower += ((deltaHeadingRadians - lastDeltaHeadingRadians)/deltaTime()) * hkd;
            }
//            if (Math.abs(robotX - targetX) > 500) {
//                XPower = 0.5 * Math.signum(robotX - targetX);
//            }

            FRPower = YPower + XPower - rotPower;
            BRPower = YPower - XPower - rotPower;
            FLPower = YPower - XPower + rotPower;
            BLPower = YPower + XPower + rotPower;


            //clip power to acceptable levels

            highestWheelPower = Math.max(Math.max(Math.abs(FRPower), Math.abs(BRPower)),
                    Math.max(Math.abs(FLPower), Math.abs(BLPower)));

            if (highestWheelPower > moveSpeed) {
                FRPower /= highestWheelPower;
                BRPower /= highestWheelPower;
                FLPower /= highestWheelPower;
                BLPower /= highestWheelPower;
            }
//            FRPower = Range.clip(FRPower, -1, 1);
//            FLPower = Range.clip(FLPower, -1, 1);
//            BRPower = Range.clip(BRPower, -1, 1);
//            BLPower = Range.clip(BLPower, -1, 1);

            try {
                autonomousRobot.wheelBase.setPowers(FLPower, FRPower, BLPower, BRPower);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }

            lastMoveDistSideways = moveDistSideways;
            lastMoveDistForwards = moveDistForwards;
            lastDeltaHeadingRadians = deltaHeadingRadians;
            moveSidewaysErrorSum += moveSidewaysError()*deltaTime();
            moveForwardsErrorSum += moveForwardsError()*deltaTime();
            deltaHeadingRadiansSum += deltaHeadingRadians;
            lastMoveSidewaysError = moveSidewaysError();
            lastMoveForwardsError = moveForwardsError();

            opMode.sleep(10);


            /**********************************
             *
             * Goost,
             *
             * I hope this message finds you in good health.
             * Please put 3 PIDS here. The first should try
             * to maintain an sideways speed of moveDistSideways, the
             * second should try to maintain a forward speed of
             * moveDistForwards, and the third should try to turn
             * the robot toward targetHeading. In other words,
             * they should, all together, take an input of
             * moveDistSideways, moveDistForwards, and targetHeading,
             * and output four motor powers.
             *
             * Godspeed,
             * Mork
             *
             **********************************/

        }
    }
}
