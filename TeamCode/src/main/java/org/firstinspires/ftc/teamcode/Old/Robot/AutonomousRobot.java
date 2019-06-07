package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tests.PositionTracker;


/**
 * Created by August on 10/22/2016.
 */

// new color values are
// difference = red value - blue value
//(Red value, Blue Value, difference)
// Blue Line (0, 10, -10)
// Red Line (12, 0, 12)
// Gray mat (2, 2, 0)
// Blue stone (1, 6, -5)
// Red stone (13, 0, 13)

public class AutonomousRobot extends Robot {
    public ServoController servoController;
    public CollectionThread collectorThread;
    public PositionTracker positionTracker;
//    public MAALSRunner maalsRunner;
    public Timer timer;
    LinearOpMode opMode;
    double lastYawError = 0;
    Thread collectionSetter;
    Thread positionSetter;
//    Thread maalsSetter;
    private TeamColor teamColor;
    public boolean startingPrimary;
    private int servoTime = 0;
    private double servoSpeed = 0;

    public AutonomousRobot(LinearOpMode varopmode, TeamColor tc) { //initialized and starts stuff
        super(varopmode, tc);
        teamColor = tc;
        opMode = varopmode;
        initAuto(varopmode.hardwareMap);
        wheelBase.runUsingEncoders();
        wheelBase.setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);

        collectorThread = new CollectionThread(varopmode, collectorR, collectorL, cubeOrienter, rampDetector, secondCube);
        collectionSetter = new Thread(collectorThread);
        collectionSetter.start();

        positionTracker = new PositionTracker(opMode, this);
        positionSetter = new Thread(positionTracker);
        positionSetter.setPriority((Thread.MAX_PRIORITY + Thread.NORM_PRIORITY) / 2);
        positionSetter.start();

//        maalsRunner = new MAALSRunner(opMode, this);
//        maalsSetter = new Thread(maalsRunner);
//        maalsSetter.setPriority((Thread.MAX_PRIORITY + Thread.NORM_PRIORITY) / 2);
//        maalsSetter.start();
//        maalsRunner.setLocation(0,0);


        timer = new Timer(varopmode);
        servoController = new ServoController(this);
    }

    public void collect(double power) {
        collectorThread.setPower(power);
    }

    public void knockOffJewel() {
        servoController.jewelArmDown();
        servoController.jewelPivotCenter();
        opMode.sleep(500);
        if (jewelColor.red() - jewelColor.blue() > 0) {
            if (teamColor == TeamColor.RED) {
                servoController.jewelPivotRight();
            } else {
                servoController.jewelPivotLeft();
            }
        } else {
            if (teamColor == TeamColor.RED) {
                servoController.jewelPivotLeft();
            } else {
                servoController.jewelPivotRight();
            }
        }
        opMode.sleep(300);
        servoController.jewelArmUp();
        servoController.jewelPivotCenter();
    }

    public void setStartPosition(boolean startingPrimary) {
        this.startingPrimary = startingPrimary;
    }

    // ------------------------ Encoder ------------------------ //

    public int getXEncoder() {
        if (positionTracker.isX1Working() && positionTracker.isX2Working()) {
            return (positionTracker.getEncoderX1() + positionTracker.getEncoderX2()) / 2;
        } else {
            if (positionTracker.isX1Working()) {
                return positionTracker.getEncoderX1();
            } else {
                return positionTracker.getEncoderX2();
            }
        }
//        return (maalsRunner.getEncoderX1() + maalsRunner.getEncoderX2()) / 2;
    }

    public int getXDiff() {
        if (positionTracker.isX1Working() && positionTracker.isX2Working()) {
            return (positionTracker.getEncoderX1() - positionTracker.getEncoderX2());
        } else {
            if (positionTracker.isX1Working()) {

                return 2 * positionTracker.getEncoderX1();
            } else {
                return -2 * positionTracker.getEncoderX2();
            }
        }
//        return (maalsRunner.getEncoderX1() - maalsRunner.getEncoderX2());
    }

    public int getYEncoder() {
        return positionTracker.getEncoderY();
//        return maalsRunner.getEncoderY();
    }

    public int getXPos() {
        return positionTracker.getX();
//        return maalsRunner.getX();
    }

    public int getYPos() {
        return positionTracker.getY();
//        return maalsRunner.getY();
    }

    public void displayPosition() {
        opMode.telemetry.addData("X", getXPos());
        opMode.telemetry.addData("Y", getYPos());
        opMode.telemetry.update();
    }

    public void collectDavid(int time, double speedCollect, double speedDrive, double heading) throws InterruptedException {
        opMode.sleep(500);
        double originalHeading = heading;
        long startTime = System.currentTimeMillis();
        collect(speedCollect);
        servoController.collectorsOut();
        servoController.servoStopsDown();
        int COLLECTION_POSITION = 6500;
        int FRONT_OF_PILE = 4000;
        int BACK_POSITION = 5900;
        int SHIFT_DISTANCE = 0; // was 600
        int driveInDistance = FRONT_OF_PILE - getYPos();
        int driveOutDistance;
        drive_distance_time(driveInDistance, 0, heading, 0.5, 1000, true, false, false);

        driveInDistance = COLLECTION_POSITION - getYPos();
        drive_distance_wiggle_time(driveInDistance, SHIFT_DISTANCE, heading, speedDrive, 2000, true, false);
        driveOutDistance = -500;
        drive_distance_time(driveOutDistance, SHIFT_DISTANCE, heading, 0.3, 1000, true, false, false);
        while (secondCube.getVoltage() < 1 && System.currentTimeMillis() - startTime < time && !opMode.isStopRequested()) {
            double collectionHeading = surveyForCubes(heading);
            int startY = getYPos();
            drive_distance_time(3000, 0, collectionHeading, 0.4, 1000, true, false, false);
            drive_distance_time(startY - getYPos(), 0, collectionHeading, 0.3, 2000, true, true, false);
            turn(heading, 5);
        }

        servoController.servoStopsUp();
    }

    public void collect_withCentering(int time, double speedCollect, double driveSpeed, double heading, int pilePosition, int firstCollectX) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        collect(speedCollect);
        servoController.collectorsOut();
        servoController.servoStopsDown();
        int COLLECTION_POSITION = 8000;
        int BACK_POSITION = 7500;
        int CUBE_WIDTH = 1600;
        int shiftPos = firstCollectX;
        while (secondCube.getVoltage() < 1 && System.currentTimeMillis() - startTime < time && !opMode.isStopRequested()) {
            int driveInDistance = COLLECTION_POSITION - getYPos();
            drive_to_pile(driveInDistance + 2000, shiftPos, heading, driveSpeed, 1000, false, true);
            drive_distance_time(1600, 0, heading, 0.2, 700, true, true, false);
            if (secondCube.getVoltage() > 1)
                break;
            drive_distance(BACK_POSITION - getYPos(), 0, heading, 0.4, true, true, false);
            if (teamColor == TeamColor.BLUE) {
                shiftPos -= CUBE_WIDTH;
            } else {
                shiftPos += CUBE_WIDTH;
            }
            opMode.sleep(100);
        }
        servoController.servoStopsUp();
    }

    public void collectSlide(int time, double speedCollect, double driveSpeed, double heading, boolean startCollectAtMax) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        collect(speedCollect);
        servoController.collectorsOut();
        servoController.servoStopsDown();
        int COLLECTION_POSITION = 10000;
        int X_MAX = 14500;
        int X_MIN = 8000;
        int direction;
        int firstCollectX;
        int xLimit;
        boolean sawACube;
        if (teamColor == TeamColor.BLUE) {
            X_MAX = -8000;
            X_MIN = -14500;
        }
        if (startCollectAtMax) {
            firstCollectX = X_MAX;
            direction = -100;
            xLimit = X_MIN;
        } else {
            firstCollectX = X_MIN;
            direction = 100;
            xLimit = X_MAX;
        }
        int driveInDistance = COLLECTION_POSITION - getYPos() + 2000;
        int BACK_POSITION = drive_to_pile(driveInDistance, firstCollectX, heading, driveSpeed, 2000, true, true) + 500;
        drive_distance_time(1600, 0, heading, 0.2, 700, true, true, false);
        drive_distance(BACK_POSITION - getYPos(), 0, heading, 0.3, true, true, false);
        while (secondCube.getVoltage() < 1 && System.currentTimeMillis() - startTime < time && !opMode.isStopRequested()) {
            if (getXPos() > X_MAX && direction > 0) {
                direction = -direction;
                xLimit = X_MIN;
                BACK_POSITION = drive_to_pile(driveInDistance, getXPos(), heading, driveSpeed, 700, true, true) + 500;
            } else if (getXPos() < X_MIN && direction < 0){
                direction = -direction;
                xLimit = X_MAX;
                BACK_POSITION = drive_to_pile(driveInDistance, getXPos(), heading, driveSpeed, 700, true, true) + 500;
            }
            sawACube = drive_surveyForCubes(BACK_POSITION, direction, heading, 0.35, (int)Math.min(time - (System.currentTimeMillis() - startTime), 6000), true, true, xLimit);
            if (sawACube) {
                drive_distance_time(1600, 0, heading, 0.4, 700, true, true, false);
                drive_distance(BACK_POSITION - getYPos(), 0, heading, 0.4, true, true, false);
            }
            opMode.sleep(100);
        }
        servoController.servoStopsUp();
    }


    // ------------------------ DRIVE FUNCTIONS --------------------- //
    public void depositOld(double targetHeading, int targetX) throws InterruptedException {
        int DEPOSIT_Y = -2000;
        int driveInDistance = DEPOSIT_Y - getYPos();
        collectorThread.turnOffTwoCubeStop();
        collect(1);
        servoController.clamp();
        servoController.dumperUp();
        opMode.sleep(800);
        servoController.unClamp();
        opMode.sleep(300);
//        cubeAngler.setPosition(0.6);
//      center_x_time(-1500, targetX, targetHeading, 0.3, 1200, true, true);
        drive_distance_time(driveInDistance, 0, targetHeading, 0.3, 1500, true, true, false);
//        cubeAngler.setPosition(0.2);
        servoController.dumperDown();
        drive_distance(1000, 0, targetHeading, 0.4, true, true, false);
        collectorThread.turnOnTwoCubeStop();
    }

    public void deposit(double targetHeading, int targetY) throws InterruptedException {
        if (dump.getPosition() < 0.7) {
            opMode.sleep(400);
        }
        Thread servoPush = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!opMode.isStopRequested()) {
                    if (timer.getTimeElapsed() > servoTime) {
                        cubeAngler.setPower(0);
                    } else {
                        cubeAngler.setPower(servoSpeed);
                    }
                    Thread.yield();
                }
            }
        });
        servoPush.start();
        int DEPOSIT_Y = targetY;
        int driveInDistance = DEPOSIT_Y - getYPos();
        servoController.dumperUp();
        servoController.unClamp();
        opMode.sleep(100);
        timer.startTimer();
        servoSpeed = 0.8;
        servoTime = 2300;
        opMode.sleep(500);
        servoController.dumperDown();
        drive_distance_time(driveInDistance, 0, targetHeading, 0.25, 1000, true, true, false);
        timer.startTimer();
        servoSpeed = -0.8;
        servoTime = 2300;
        drive_distance(1000, 0, targetHeading, 0.4, true, true, false);
        collectorThread.turnOnTwoCubeStop();
    }

    // Core drive functions

    public void drive_time (int y, int x, double targetHeading, double power, long time, boolean gain, boolean stop, boolean flipX) throws InterruptedException {
        if(flipX){
            x= -x;
        }
        boolean reached = false;
        long startTime = System.currentTimeMillis();
        while (!reached && !opMode.isStopRequested()) {
            baseDrive(x, y, targetHeading, power, gain);
            if (System.currentTimeMillis() - startTime > time)
                reached = true;
        }
        if (stop) {
            wheelBase.setPowers(0, 0, 0, 0);
        }
    }

    public int drive_color(int y, int x, double targetHeading, double speed, boolean stop) throws InterruptedException {
        ColorSensor stopColor;
        double targetVal = 15;
        if (teamColor == TeamColor.BLUE) {
            targetVal = 6;
        }
        if (y > 0) {
            stopColor = colorFront;
        } else {
            stopColor = colorBack;
        }
        int position = getYPos();
        boolean reached = false;
        int currentValStop;
        while (!reached && !opMode.isStopRequested()) {
            currentValStop = (stopColor.red() - stopColor.blue());
            baseDrive(x, y, targetHeading, speed, true);
            reached = Math.abs(currentValStop) > targetVal;
            if (reached) {
                position = getYPos();
            }
        }
        if(stop){
            wheelBase.setPowers(0,0,0,0);
        }
        return position;
    }

    public int drive_triange(int y, double targetSpeed) throws InterruptedException {
        int triangleCenter, firstPos, secondPos;
        int robotOffset;
        if (y < 0) {
            robotOffset = -1300;
        } else {
            robotOffset = 1300;
        }
        firstPos = drive_color(y, -Math.abs(y/2), 0, targetSpeed, false);
        opMode.telemetry.addData("Saw first line", firstPos);
        opMode.telemetry.update();
        drive_no_color(y, 0, 0, targetSpeed, false);
        secondPos = drive_color(y, 0, 0, targetSpeed, true);
        opMode.telemetry.addData("Saw second line", secondPos);
        opMode.telemetry.update();
        triangleCenter = (firstPos + secondPos)/2 + robotOffset;
//        drive_to_position(triangleCenter, 0, 0, 0.3, 3000);
        return triangleCenter;
    }

    public int drive_no_color (int y, int x, double targetHeading, double speed, boolean stop) throws InterruptedException {
        //drive till the color sensor sees the mat and no tape.
        double targetVal = 8;
        if (teamColor == TeamColor.BLUE) {
            targetVal = 6;
        }
        ColorSensor stopColor = colorBack;
        if (y < 0) {
            stopColor = colorFront;
        }
        int position = getYPos();
        boolean reached = false;
        int currentValStop;
        while (!reached && !opMode.isStopRequested()) {
            currentValStop = (stopColor.red() - stopColor.blue());
            baseDrive(x, y, targetHeading, speed, true);
            reached = Math.abs(currentValStop) < targetVal;
            if (reached) {
                position = getYPos();
            }
        }
        if(stop){
            wheelBase.setPowers(0,0,0,0);
        }
        return position;
    }

    public void center_x(int y, int targetX, double targetHeading, double power, int time, boolean gain, boolean stop) throws InterruptedException {
        boolean reached = false;
        int currentEncoderX;
        int startEncoderY;
        long startTime = System.currentTimeMillis();
        int stopTime = time;
        int encoderToXCompConversion = 2;
        startEncoderY = getYEncoder();

        while (!reached && !opMode.isStopRequested()) {
            currentEncoderX = getXPos();
            int xComp = (currentEncoderX - targetX)/encoderToXCompConversion;
            xComp = Range.clip(xComp, -500, 500);

            if (y > 0) {
                baseDrive(-xComp, 300, targetHeading, power, gain);
            } else {
                baseDrive(-xComp, -300, targetHeading, power, gain);
            }

            int yval = getYEncoder() - startEncoderY;

            if (Math.abs(y - yval) < 2500) { // slows down drive when it gets closer to desired value when driving straight
                power = 0.2;
            }

            if (System.currentTimeMillis() - startTime > stopTime) {
                reached = true;
            } else {
                reached = Math.abs(yval) > Math.abs(y);
            }
        }
        if (stop)
            wheelBase.setPowers(0,0,0,0);
    }

    public void center_y(int targetY, int x, double targetHeading, double power, int time, boolean gain, boolean stop) throws InterruptedException {
        boolean reached = false;
        int currentEncoderY;
        long startTime = System.currentTimeMillis();
        int encoderToYCompConversion = 2;
        int startEncoderX = getXEncoder();
        while (!reached && !opMode.isStopRequested()) {
            currentEncoderY = getYPos();
            int yComp = (targetY-currentEncoderY)/encoderToYCompConversion;
            yComp = Range.clip(yComp, -500, 500);

            if (x > 0) {
                baseDrive(2000, yComp, targetHeading, power, gain);
            } else {
                baseDrive(-2000, yComp, targetHeading, power, gain);
            }
            int xval = getXEncoder() - startEncoderX;

            if (System.currentTimeMillis() - startTime > time) {
                reached = true;
            } else {
                reached = Math.abs(xval) > Math.abs(x);
            }
        }
        if (stop)
            wheelBase.setPowers(0,0,0,0);
    }

    public void center_x_time(int y, int targetX, double targetHeading, double power, int time, boolean gain, boolean stop) throws InterruptedException {
        boolean reached = false;
        int currentEncoder;
        long startTime = System.currentTimeMillis();
        int encoderToXCompConversion = 2;
        int startY = getYEncoder();
        while (!reached && !opMode.isStopRequested()) {
            currentEncoder = getXPos();
            int yval = getYEncoder() - startY;
            int xComp = (currentEncoder - targetX)/encoderToXCompConversion;
            xComp = Range.clip(xComp, -500, 500);

            if (y > 0) {
                baseDrive(-xComp, 500, targetHeading, power, true);
            } else {
                baseDrive(-xComp, -500, targetHeading, power, true);
            }
            reached = Math.abs(yval) > Math.abs(y) || System.currentTimeMillis() - startTime > time;
        }
        if (stop)
            wheelBase.setPowers(0,0,0,0);
    }

    public void drive_to_position(int targetY, int targetX, double targetHeading, double speedCap, int time, int yTolerance, int xTolerance, boolean getReadyToDump) throws InterruptedException {
        boolean reached = false, withinTolerance;
        boolean withinSlowdownTolerance;
        int X_TOLERANCE = xTolerance; // good value is 300
        int Y_TOLERANCE = yTolerance; // good value is 700 for depositing, 400 for first Line up
        int SLOWDOWN_TOLERANCE = 2000;
        int TIME_TO_STOP = 500;
        double MINSPEED = 0.17;
        long startTime = System.currentTimeMillis();
        while (!reached && !opMode.isStopRequested()) {
            int differenceX = targetX - getXPos();
            int differenceY = targetY - getYPos();
            double vectorDifference = Math.sqrt(differenceX*differenceX + 0.5*differenceY*differenceY);
            double setSpeed =  Range.clip(vectorDifference/6000, MINSPEED, speedCap);
            int yComp = differenceY;
            int xComp = differenceX;
            baseDrive(xComp, yComp, targetHeading, setSpeed, true);
            withinSlowdownTolerance = Math.abs(differenceY) < SLOWDOWN_TOLERANCE;
            if (withinSlowdownTolerance) {
                if (getReadyToDump) {
                    collectorThread.turnOffTwoCubeStop();
                    collect(1);
                    servoController.clamp();
                    servoController.dumperUp();
                }
            }
            withinTolerance = (Math.abs(differenceY) < Y_TOLERANCE && Math.abs(differenceX) < X_TOLERANCE) || System.currentTimeMillis() - startTime > time;
            if (!withinTolerance) {
                timer.startTimer();
            } else {
                wheelBase.setPowers(0,0,0,0);
            }
            reached = timer.getTimeElapsed() > TIME_TO_STOP;
        }
        wheelBase.setPowers(0,0,0,0);
    }

    public void drive_to_position_old(int targetY, int targetX, double targetHeading, double targetSpeed, int time, int yTolerance) throws InterruptedException {
        boolean reached = false, withinTolerance;
        double setSpeed = targetSpeed;
        boolean withinSlowdownTolerance;
        int X_TOLERANCE = 300;
        int Y_TOLERANCE = yTolerance;
        int SLOWDOWN_TOLERANCE = 2000;
        int TIME_TO_STOP = 500;
        long startTime = System.currentTimeMillis();
        int encoderToXCompConversion = 2;
        int encoderToYCompConversion = 2;
        while (!reached && !opMode.isStopRequested()) {
            int differenceX = targetX - getXPos();
            int differenceY = targetY - getYPos();
            int yComp = Range.clip((differenceY)/encoderToYCompConversion, -500, 500);
            int xComp = Range.clip((differenceX)/encoderToXCompConversion, -500, 500);
            baseDrive(xComp, yComp, targetHeading, setSpeed, true);
            withinSlowdownTolerance = Math.abs(differenceY) < SLOWDOWN_TOLERANCE;

            if (withinSlowdownTolerance) {
                setSpeed = 0.2;
                collectorThread.turnOffTwoCubeStop();
                collect(1);
                servoController.clamp();
                servoController.dumperUp();
            } else if (Math.abs(differenceY) < SLOWDOWN_TOLERANCE + 2000) {
                setSpeed = 0.4;
            }else {
                setSpeed = targetSpeed;
            }
            withinTolerance = (Math.abs(differenceY) < Y_TOLERANCE && Math.abs(differenceX) < X_TOLERANCE) || System.currentTimeMillis() - startTime > time;
            if (!withinTolerance) {
                timer.startTimer();
            } else {
                wheelBase.setPowers(0,0,0,0);
            }
            reached = timer.getTimeElapsed() > TIME_TO_STOP;
        }
        wheelBase.setPowers(0,0,0,0);
    }

    public void drive_distance(int y, int x, double targetHeading, double targetSpeed, boolean gain, boolean stop, boolean flipX) throws InterruptedException {
        boolean reached = false;
        if (flipX) {
            x = -x;
        }
        int startY = getYEncoder();
        int startX = getXEncoder();
        int yval = 0;
        int xval = 0;
        while (!opMode.isStopRequested() && !reached) {
            yval = getYEncoder() - startY;
            xval = getXEncoder() - startX;
            baseDrive(x, y, targetHeading, targetSpeed, gain);
//            if (Math.abs(y - yval) < 1000 && Math.abs(y) > Math.abs(x))
//                targetSpeed = 0.1;
            if (Math.abs(y) > Math.abs(x)) {
                reached = Math.abs(yval) > Math.abs(y);
            } else {
                reached = Math.abs(xval) > Math.abs(x); //stop condition
            }
        }
        if (stop) {
            wheelBase.setPowers(0,0,0,0);
        }
    }

    public void drive_distance_time(int y, int x, double targetHeading, double targetSpeed, int time, boolean gain, boolean stop, boolean flipX) throws InterruptedException {
        long startTime=System.currentTimeMillis();
        boolean reached = false;
        if (flipX) {
            x = -x;
        }
        int startY = getYEncoder();
        int startX = getXEncoder();
        while (!opMode.isStopRequested() && !reached) {
            int yval = getYEncoder() - startY;
            int xval = getXEncoder() - startX;
            baseDrive(x, y, targetHeading, targetSpeed, gain);
            if (Math.abs(y) > Math.abs(x)) {
                reached = Math.abs(yval) > Math.abs(y);
            } else {
                reached = Math.abs(xval) > Math.abs(x); //stop condition
            }
            if (System.currentTimeMillis() - startTime > time)
                reached=true;
        }
        if (stop) {
            wheelBase.setPowers(0,0,0,0);
        }
    }

    public int drive_to_pile(int y, int targetX, double targetHeading, double power, int time, boolean gain, boolean stop) throws InterruptedException {
        boolean reached = false;
        int currentEncoder;
        long startTime = System.currentTimeMillis();
        int encoderToXCompConversion = 2;
        int startY = getYEncoder();
        int pilePosition = getYPos();
        while (!reached && !opMode.isStopRequested()) {
            currentEncoder = getXPos();
            double pileDetectorReading = pileDetector.getVoltage();
            int yval = getYEncoder() - startY;
            int xComp = (currentEncoder - targetX)/encoderToXCompConversion;
            xComp = Range.clip(xComp, -500, 500);

            if (y > 0) {
                baseDrive(-xComp, 500, targetHeading, power, true);
            } else {
                baseDrive(-xComp, -500, targetHeading, power, true);
            }
            reached = Math.abs(yval) > Math.abs(y) || System.currentTimeMillis() - startTime > time;
            pilePosition = getYPos();
            if (pileDetectorReading > 1.83) {
                reached = true;
            }
        }
        if (stop)
            wheelBase.setPowers(0,0,0,0);
        return pilePosition;
    }

    public void drive_distance_wiggle_time(int y, int x, double targetHeading, double targetSpeed, int time,  boolean stop, boolean flipX) throws InterruptedException {
        boolean reached = false;
        if (flipX) {
            x = -x;
        }
        int startY = getYEncoder();
        int startX = getXEncoder();
        int yval = 0;
        int xval = 0;
        int count = 0;
        double rightHeading = targetHeading + 20;
        double leftHeading = targetHeading - 20;
        double heading = rightHeading;
        long startTime = System.currentTimeMillis();
        while (!opMode.isStopRequested() && !reached) {
            if (count > 5000) {
                count = 0;
                if (heading == rightHeading) {
                    heading = leftHeading;
                } else {
                    heading = rightHeading;
                }
            }
            count ++;
            yval = getYEncoder() - startY;
            xval = getXEncoder() - startX;
            baseDriveWiggle(x, y, heading, targetSpeed);
            if (Math.abs(y) > Math.abs(x)) {
                reached = Math.abs(yval) > Math.abs(y);
            } else {
                reached = Math.abs(xval) > Math.abs(x); //stop condition
            }
            if (System.currentTimeMillis() - startTime > time)
                reached=true;
        }
        if (stop) {
            wheelBase.setPowers(0,0,0,0);
        }
    }

    public void drive_off_ramp(int targetY, int targetX, double targetSpeed) throws InterruptedException {
//        drive_to_position(targetY, targetX, 0, 0.5, 5000, 300, 500, true);
        double speedCap = 0.5;
        int time = 5000;
        int yTolerance = 300;
        int xTolerance = 500;
        boolean getReadyToDump = true;
        boolean reached = false, withinTolerance;
        boolean withinSlowdownTolerance;
        int X_TOLERANCE = xTolerance; // good value is 300
        int Y_TOLERANCE = yTolerance; // good value is 700 for depositing, 400 for first Line up
        int SLOWDOWN_TOLERANCE = 2000;
        int TIME_TO_STOP = 500;
        double MINSPEED = 0.17;
        long startTime = System.currentTimeMillis();
        boolean yDeltaIsSmall;
        int deltaSmallCount = 0;
        while (!reached && !opMode.isStopRequested()) {
            yDeltaIsSmall = Math.abs(positionTracker.getYDelta()) < 5;
            if (yDeltaIsSmall) {
                deltaSmallCount ++;
            } else {
                deltaSmallCount = 0;
            }

            if (deltaSmallCount > 5000) {
                if (positionTracker.isYWorking()) {
                    positionTracker.setEncoderYAsBroken();
                    if (targetY < 0) {
                        targetY = targetY - 800;
                    } else {
                        targetY = targetY + 800;
                    }
                }
            }
            int differenceX = targetX - getXPos();
            int differenceY = targetY - getYPos();
            double vectorDifference = Math.sqrt(differenceX*differenceX + 0.5*differenceY*differenceY);
            double setSpeed =  Range.clip(vectorDifference/6000, MINSPEED, speedCap);
            int yComp = differenceY;
            int xComp = differenceX;
            baseDrive(xComp, yComp, 0, setSpeed, true);
            withinSlowdownTolerance = Math.abs(differenceY) < SLOWDOWN_TOLERANCE;
            if (withinSlowdownTolerance) {
                if (getReadyToDump) {
                    collectorThread.turnOffTwoCubeStop();
                    collect(1);
                    servoController.clamp();
                    servoController.dumperUp();
                }
            }
            withinTolerance = (Math.abs(differenceY) < Y_TOLERANCE && Math.abs(differenceX) < X_TOLERANCE) || System.currentTimeMillis() - startTime > time;
            if (!withinTolerance) {
                timer.startTimer();
            } else {
                wheelBase.setPowers(0,0,0,0);
            }
            reached = timer.getTimeElapsed() > TIME_TO_STOP;
        }
        wheelBase.setPowers(0,0,0,0);



    }

    public boolean drive_surveyForCubes (int targetY, int x, double targetHeading, double power, int time, boolean gain, boolean stop, int xLimit) throws InterruptedException {
        boolean reached = false;
        int currentEncoderY;
        double startDistance = pileDetector.getVoltage();
        long startTime = System.currentTimeMillis();
        int encoderToYCompConversion = 2;
        int startEncoderX = getXEncoder();
        boolean sawACube = false;
        while (!reached && !opMode.isStopRequested()) {
            currentEncoderY = getYPos();
            int yComp = (targetY - currentEncoderY) / encoderToYCompConversion;
            yComp = Range.clip(yComp, -500, 500);

            if (x > 0) {
                baseDrive(2000, yComp, targetHeading, power, gain);
            } else {
                baseDrive(-2000, yComp, targetHeading, power, gain);
            }
            int xval = getXEncoder() - startEncoderX;

            boolean reachedXLimit;
            if (x<0) {
                reachedXLimit = getXPos() < xLimit;
            } else {
                reachedXLimit = getXPos() > xLimit;
            }

            if (System.currentTimeMillis() - startTime > time) {
                reached = true;
            } else {
                sawACube = pileDetector.getVoltage() - startDistance > 0.2 || pileDetector.getVoltage() > 1.3;
                reached = pileDetector.getVoltage() - startDistance > 0.2 || pileDetector.getVoltage() > 1.3 || reachedXLimit || secondCube.getVoltage() > 1;
            }
        }
        if (stop)
            wheelBase.setPowers(0, 0, 0, 0);
        return sawACube;
    }

        // Base drive functions
    public void baseDrive(int x, int y, double targetHeading, double targetSpeed, boolean gain) throws InterruptedException{
//        timer.stopTimer();
//        timer.displayTime();
//        timer.startTimer();
        if (opMode.isStopRequested()){
            return;
        }
        double KP;
        double KD;
        //Different PID values when going completely straight
        if (Math.abs(y) > Math.abs(x)) {
            KP = 0.02;//gyro
            KD = 0.02;//gyro
        } else {
            KP = 0.02;
            KD = 0.02;
        }
        if(gain == false){
            KP = 0;
            KD = 0;
        }
        double yawError = turnDifference(positionTracker.getHeading(), targetHeading);//not sure what angle to use
//        double yawError = turnDifference(maalsRunner.getRawHeading(), targetHeading);//not sure what angle to use
        double yawDeriv =  (yawError-lastYawError);
        //calculates how much power should be crabbing vs. going straight
        double percentMovementForward = y!=0 ? (y*1.0) / (Math.abs(y*1.0) + Math.abs(x*1.0)):0;
        double percentMovementSideways = x!=0 ? (x*1.0) / (Math.abs(y*1.0) + Math.abs(x*1.0)):0;
        yawError = Range.clip(yawError, -30, 30);
        lastYawError = yawError;
        double gyroCorrection = Range.clip(KD * yawDeriv + KP * yawError, -0.4, 0.4);
        wheelBase.setPowers(
                Range.clip((percentMovementForward * targetSpeed) + (percentMovementSideways * targetSpeed) + gyroCorrection, -1.0, 1.0), //FL
                Range.clip((percentMovementForward * targetSpeed) - (percentMovementSideways * targetSpeed) - gyroCorrection, -1.0, 1.0), //FR
                Range.clip((percentMovementForward * targetSpeed) - (percentMovementSideways * targetSpeed) + gyroCorrection, -1.0, 1.0), //BL
                Range.clip((percentMovementForward * targetSpeed) + (percentMovementSideways * targetSpeed) - gyroCorrection, -1.0, 1.0));//BR
//        opMode.sleep(50);
        Thread.yield();
    }

    public void baseDriveWiggle(int x, int y, double targetHeading, double targetSpeed) throws InterruptedException{
//        timer.stopTimer();
//        timer.displayTime();
//        timer.startTimer();
        if (opMode.isStopRequested()){
            return;
        }
        double KP;
        double KD;
        double GYRO_CLIP = 2;
        //Different PID values when going completely straight
        if (Math.abs(y) > Math.abs(x)) {
            KP = 0.02;//gyro
            KD = 0.02;//gyro
        } else {
            KP = 0.02;
            KD = 0.02;
        }
        double yawError = turnDifference(positionTracker.getHeading(), targetHeading);//not sure what angle to use
//        double yawError = turnDifference(maalsRunner.getRawHeading(), targetHeading);//not sure what angle to use

        double yawDeriv =  (yawError-lastYawError);
        //calculates how much power should be crabbing vs. going straight
        double percentMovementForward = y!=0 ? (y*1.0) / (Math.abs(y*1.0) + Math.abs(x*1.0)):0;
        double percentMovementSideways = x!=0 ? (x*1.0) / (Math.abs(y*1.0) + Math.abs(x*1.0)):0;
        yawError = Range.clip(yawError, -30, 30);
        lastYawError = yawError;

        double gyroCorrection = Range.clip(KD * yawDeriv + KP * yawError, -GYRO_CLIP, GYRO_CLIP);
        wheelBase.setPowers(
                Range.clip((percentMovementForward * targetSpeed) + (percentMovementSideways * targetSpeed) + gyroCorrection, -1.0, 1.0), //FL
                Range.clip((percentMovementForward * targetSpeed) - (percentMovementSideways * targetSpeed) - gyroCorrection, -1.0, 1.0), //FR
                Range.clip((percentMovementForward * targetSpeed) - (percentMovementSideways * targetSpeed) + gyroCorrection, -1.0, 1.0), //BL
                Range.clip((percentMovementForward * targetSpeed) + (percentMovementSideways * targetSpeed) - gyroCorrection, -1.0, 1.0));//BR
        Thread.yield();
    }

    // ------------------------ Turn ------------------------ //
    public void turn (double heading, double tolerance) throws InterruptedException {
        double ENCODER_CLICKS_PER_TURN = 25000;                                        // number of encoder clicks for a full rotation of the robot
        int COUNT_WITHIN_TOLERANCE = 10000;                                             // number of cycles the robot waits while its inside the tolerance before exiting
        double encoderTolerance = (int)((tolerance/360) * ENCODER_CLICKS_PER_TURN);    // converts degrees tolerance to encoder tolerance
        double TARGET_SPEED = 0.2;                                                  // initial targetSpeed
        double startDiff = getXDiff();
        double gyroHeading = positionTracker.getHeading();
        double targetDiff = (int)((-turnDifference(heading, gyroHeading)/360) * ENCODER_CLICKS_PER_TURN + startDiff); // converts turn difference into an encoder difference
        boolean reached = false;
        int count = 0;
        boolean encoderFailure = false;
        int startX1 = positionTracker.getEncoderX1();
        int startX2 = positionTracker.getEncoderX2();
        while (!reached && !opMode.isStopRequested()) {
            int currentXDiff = getXDiff();
            double turnDifference = targetDiff - currentXDiff;
            double turnDifferenceDegrees = -(turnDifference/ENCODER_CLICKS_PER_TURN) * 360;
            encoderFailure = Math.abs(turnDifferenceDegrees - turnDifference(heading, positionTracker.getHeading())) > 20;
//            opMode.telemetry.addData("turnDifferenceDegrees", turnDifferenceDegrees);
//            opMode.telemetry.addData("turnDifference", turnDifference);
//            opMode.telemetry.addData("turnDifferenceGyro", turnDifference(positionTracker.getHeading(), heading));
//            opMode.telemetry.update();
            if (encoderFailure) {
                break;
            }
            if (Math.abs(turnDifference) < 2000) {
                TARGET_SPEED = 0.1;
            } else {
                TARGET_SPEED = 0.5;
            }
            if (count == 0) {
                if (turnDifference < 0) {
                    wheelBase.setPowers(-TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED);
                } else {
                    wheelBase.setPowers(TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED);
                }
            }
            if (Math.abs(targetDiff - currentXDiff) < encoderTolerance) {
                if (count == 0) { // stops motors only the first time through the loop
                    wheelBase.setPowers(0,0,0,0);
                }
                count ++;
            } else {
                count = 0;
            }
            if (count > COUNT_WITHIN_TOLERANCE)
                reached = true;
            Thread.yield();
        }
        wheelBase.setPowers(0,0,0,0);
        if (encoderFailure) {
            opMode.telemetry.addData("in turn Gyro", "");
            opMode.telemetry.update();
            turnGyro(heading, tolerance);
        }
        if (Math.abs(startX1 - positionTracker.getEncoderX1()) < 100) {
            positionTracker.setEncoderX1AsBroken();
        } else if (Math.abs(startX2 - positionTracker.getEncoderX2()) < 100) {
            positionTracker.setEncoderX2AsBroken();
        }
    }

    public double surveyForCubes (double heading) throws InterruptedException {
        double TARGET_SPEED = 0.4;                                                  // initial targetSpeed
        boolean reached = false;
        double pileDistance;
        double highestPileDistance = 0;
        double rightHeading = heading - 30;
        double leftHeading = heading + 30;
        double targetHeading = rightHeading;
        double closeCubeHeading = heading;
        boolean foundCube = false;
        while (!reached && !opMode.isStopRequested()) {
            double currentHeading = positionTracker.getHeading();
//            double currentHeading = maalsRunner.getRawHeading();
            double turnDifference = turnDifference(targetHeading, currentHeading);
            pileDistance = pileDetector.getVoltage();
            if (pileDistance > highestPileDistance) {
                highestPileDistance = pileDistance;
                closeCubeHeading = currentHeading;
            }
            if (pileDistance > 1.25) {
                foundCube = true;
                break;
            }

            if (turnDifference > 0) {
                wheelBase.setPowers(-TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED);
            } else {
                wheelBase.setPowers(TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED);
            }

            if (Math.abs(turnDifference) < 5) {
                if (targetHeading == leftHeading) {
                    reached = true;
                } else {
                    targetHeading = leftHeading;
                }
            }
            Thread.yield();
        }
        wheelBase.setPowers(0,0,0,0);
        if (!foundCube)
            turn(closeCubeHeading, 5);
        return closeCubeHeading;
    }

    public void turnGyro (double heading, double tolerance) throws InterruptedException {
        int COUNT_WITHIN_TOLERANCE = 5;
        double TARGET_SPEED;
        boolean reached = false;
        int count = 0;
        while (!reached && !opMode.isStopRequested()) {
            double currentHeading = positionTracker.getHeading();
            double turnDifference = turnDifference(heading, currentHeading);
            if (Math.abs(turnDifference) < 30) {
                TARGET_SPEED = 0.07;
            } else if (Math.abs(turnDifference) < 45) {
                TARGET_SPEED = 0.25;
            } else {
                TARGET_SPEED = 0.5;
            }
            if (turnDifference > 0) {
                wheelBase.setPowers(-TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED);
            } else {
                wheelBase.setPowers(TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED);
            }

            if (Math.abs(turnDifference) < tolerance) {
                count ++;
            } else {
                count = 0;
            }

            if (count > COUNT_WITHIN_TOLERANCE)
                reached = true;
            opMode.sleep(50);
        }
        wheelBase.setPowers(0,0,0,0);
    }

    public void turnFast (double heading) throws InterruptedException {
        int COUNT_WITHIN_TOLERANCE = 5;
        double TOLERANCE = 10;
        double TARGET_SPEED = 0.3;                               // number of cycles the robot waits while its inside the tolerance before exiting
        boolean reached = false;
        int count = 0;
        while (!reached && !opMode.isStopRequested()) {
            double currentHeading = positionTracker.getHeading();
//            double currentHeading = maalsRunner.getRawHeading();
            double turnDifference = turnDifference(heading, currentHeading);
            if (turnDifference > 0) {
                wheelBase.setPowers(-TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED);
            } else {
                wheelBase.setPowers(TARGET_SPEED, -TARGET_SPEED, TARGET_SPEED, -TARGET_SPEED);
            }

            if (Math.abs(turnDifference) < TOLERANCE) {
                count ++;
            } else {
                count = 0;
            }

            if (count > COUNT_WITHIN_TOLERANCE)
                reached = true;
            opMode.sleep(50);
        }
        wheelBase.setPowers(0,0,0,0);
    }


    public double turnDifference(double currentHeading, double targetheading) { //compares difference between angles, keeping in mind 180 = -180
        double difference = (currentHeading - targetheading) % 360; //calculates the angle based on where the robot is now and how far it has to go

        if (difference >= 180) { //determines which way the robot will turn (left or right)
            difference -= 360.0;
        } else if (difference < -180) {
            difference += 360.0;
        }
        return difference;
    }
}