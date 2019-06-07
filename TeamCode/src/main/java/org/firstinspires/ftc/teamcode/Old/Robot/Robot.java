package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by August on 10/22/2016.
 */

public class Robot {
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    public DcMotor collectorR;
    public DcMotor collectorL;
    public DcMotor lift;
    public DcMotor extender;

    public Servo dump = null;
    public Servo dump1 = null;
    public Servo clampR = null;
    public Servo clampL = null;
    public Servo rightServoStop = null;
    public Servo leftServoStop = null;
    public Servo relicGripper = null;
    public Servo relicLifter = null;
    public Servo jewelArm = null;
    public Servo jewelPivot = null;
    public CRServo cubeAngler = null;
    public Servo collectionDeployR = null;
    public Servo collectionDeployL = null;

    public DigitalChannel bottomStop = null;
    public DigitalChannel topStop = null;
    public DigitalChannel cryptoboxDetector = null;
    public DigitalChannel extenderOut = null;
//    public DigitalChannel magnet = null;
    public ColorSensor jewelColor;
    public BNO055IMU imu1;
    public BNO055IMU imu2;
    public AnalogInput pileDetector = null;
    public AnalogInput cubeOrienter = null;
    public AnalogInput secondCube = null;
    public AnalogInput rampDetector = null;
    public ModernRoboticsI2cColorSensor colorFront = null;
    public ModernRoboticsI2cColorSensor colorBack = null;

    OpMode opMode;
    public WheelBase wheelBase;
    TeamColor teamColor;
    HardwareMap hwMap  = null;

    public VuforiaLocalizer vuforia;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    public Robot(OpMode varopmode, TeamColor tc) {
        opMode = varopmode;
        teamColor = tc;
    }

    public void initAuto(HardwareMap ahwMap){
        hwMap = ahwMap;
        //autoInit
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVuforia = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersVuforia.vuforiaLicenseKey = "AWRj/xj/////AAAAGQxUvYmWcEwclJxNRPrJOqYGevnetxTRA6P/gYZfb+gZsAeO0GosrfKmnO2O24hVPv8v1YQYA8vQ5qc1eVjwOjZPCykT4eRXxqxLZV/7BJmEraEs991INaYI9qXQjGkbeWbLQT/e7zJAvxsRVjQjRPukDLapC4dmdA5YbXvxy9pR2+LokoO6PiSLl9ktBte3BFGHQepiugBC7C1jXDfkClwTb/+R7OwaVuL1gp6rWun5Cn42RHysv4HsTkBMShaKdL4/whXVRmrYfkMMsAtihEAK+rLs8fWnmVB1Z/UJ67QIWqP04Va/u/mbTErjPDiRvCnYhGmIWTpIt+9slhip9vT9pRfLIe+gcfAIajoF6wUe";
        parametersVuforia.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //if normal vuforia doesn't work, comment out this
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVuforia);
        vuforia.setFrameQueueCapacity(1);
        //just these two lines

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);


        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu1 = hwMap.get(BNO055IMU.class, "imu");
        imu1.initialize(parameters);

        FR = hwMap.dcMotor.get("fr");
        FL = hwMap.dcMotor.get("fl");
        BR = hwMap.dcMotor.get("br");
        BL = hwMap.dcMotor.get("bl");
        collectorR = hwMap.dcMotor.get("collectorR");
        collectorL = hwMap.dcMotor.get("collectorL");
        lift = hwMap.dcMotor.get("lift");
        extender = hwMap.dcMotor.get("extender");

        dump = hwMap.servo.get("dump");
//        dump1 = hwMap.servo.get("dump1");
        clampR = hwMap.servo.get("clampR");
        clampL = hwMap.servo.get("clampL");
        rightServoStop = hwMap.servo.get("rStop");
        leftServoStop = hwMap.servo.get("lStop");
        relicGripper = hwMap.servo.get("relicGripper");
        relicLifter = hwMap.servo.get("relicLifter");
        jewelArm = hwMap.servo.get("jewelArm");
        jewelPivot = hwMap.servo.get("jewelPivot");
        cubeAngler = hwMap.crservo.get("cubeAngle");
        collectionDeployR = hwMap.servo.get("collectionDeployR");
        collectionDeployL = hwMap.servo.get("collectionDeployL");


        pileDetector = hwMap.analogInput.get("pileDetector");
        cubeOrienter = hwMap.analogInput.get("cubeOrienter");
        secondCube = hwMap.analogInput.get("cubeHopperSensor");
        rampDetector = hwMap.analogInput.get("rampDetector");


        bottomStop = hwMap.digitalChannel.get("tBottom");
        topStop = hwMap.digitalChannel.get("tTop");
        cryptoboxDetector = hwMap.digitalChannel.get("cryptoboxDetector");
        extenderOut = hwMap.digitalChannel.get("extenderOut");

        colorFront = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorRight");
        colorBack = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorLeft");

        jewelColor = hwMap.get(ColorSensor.class, "jewelColor");

        clampL.setDirection(Servo.Direction.REVERSE);
        relicGripper.setDirection(Servo.Direction.REVERSE);
        rightServoStop.setDirection(Servo.Direction.REVERSE);
        jewelArm.setDirection(Servo.Direction.REVERSE);
        collectionDeployL.setDirection(Servo.Direction.REVERSE);

        dump.setPosition(0.185);
        clampR.setPosition(0.51);
        clampL.setPosition(0.51);
        relicLifter.setPosition(0);
        relicGripper.setPosition(0.4);
        rightServoStop.setPosition(0);
        leftServoStop.setPosition(0);
        jewelArm.setPosition(0.85);
        jewelPivot.setPosition(0.5);
        cubeAngler.setPower(0);
        collectionDeployR.setPosition(0);
        collectionDeployL.setPosition(0);

        lift.setDirection(DcMotor.Direction.REVERSE);


        wheelBase = new WheelBase(opMode);
        wheelBase.startMotorThreads();
    }

    //separate init for Teleop
    public void initTele(HardwareMap ahwMap){

        hwMap = ahwMap;
        FR = hwMap.dcMotor.get("fr");
        FL = hwMap.dcMotor.get("fl");
        BR = hwMap.dcMotor.get("br");
        BL = hwMap.dcMotor.get("bl");
        collectorR = hwMap.dcMotor.get("collectorR");
        collectorL = hwMap.dcMotor.get("collectorL");
        lift = hwMap.dcMotor.get("lift");
        extender = hwMap.dcMotor.get("extender");

        dump = hwMap.servo.get("dump");
//        dump1 = hwMap.servo.get("dump1");
        clampR = hwMap.servo.get("clampR");
        clampL = hwMap.servo.get("clampL");
        rightServoStop = hwMap.servo.get("rStop");
        leftServoStop = hwMap.servo.get("lStop");
        relicGripper = hwMap.servo.get("relicGripper");
        relicLifter = hwMap.servo.get("relicLifter");
        jewelArm = hwMap.servo.get("jewelArm");
        jewelPivot = hwMap.servo.get("jewelPivot");
        cubeAngler = hwMap.crservo.get("cubeAngle");
        collectionDeployR = hwMap.servo.get("collectionDeployR");
        collectionDeployL = hwMap.servo.get("collectionDeployL");


        pileDetector = hwMap.analogInput.get("pileDetector");
        cubeOrienter = hwMap.analogInput.get("cubeOrienter");
        secondCube = hwMap.analogInput.get("cubeHopperSensor");
        rampDetector = hwMap.analogInput.get("rampDetector");


        bottomStop = hwMap.digitalChannel.get("tBottom");
        topStop = hwMap.digitalChannel.get("tTop");
        cryptoboxDetector = hwMap.digitalChannel.get("cryptoboxDetector");
        extenderOut = hwMap.digitalChannel.get("extenderOut");


        // ---------- Motor Directions and Break State ---------- //
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorR.setDirection(DcMotor.Direction.REVERSE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        // ---------- Set Servo Directions ---------- //
        clampL.setDirection(Servo.Direction.REVERSE);
        relicGripper.setDirection(Servo.Direction.REVERSE);
        rightServoStop.setDirection(Servo.Direction.REVERSE);
        jewelArm.setDirection(Servo.Direction.REVERSE);
        collectionDeployL.setDirection(Servo.Direction.REVERSE);

        // ---------- Set Servo Resting Positions ---------- //
        dump.setPosition(dump.getPosition());
        clampR.setPosition(clampR.getPosition());
        clampL.setPosition(clampL.getPosition());
        relicLifter.setPosition(relicLifter.getPosition());
        relicGripper.setPosition(relicGripper.getPosition());
        rightServoStop.setPosition(rightServoStop.getPosition());
        leftServoStop.setPosition(leftServoStop.getPosition());
        jewelArm.setPosition(jewelArm.getPosition());
        jewelPivot.setPosition(jewelPivot.getPosition());
        cubeAngler.setPower(0);
        collectionDeployL.setPosition(collectionDeployL.getPosition());
        collectionDeployR.setPosition(collectionDeployR.getPosition());

    }
}
