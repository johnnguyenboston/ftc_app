package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */

@Autonomous(name="Concept: Change PID", group = "Tune")
public class changePID extends LinearOpMode {

    // our DC motor.
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        FL = (DcMotorEx)hardwareMap.get(DcMotor.class, "FL");
        FR = (DcMotorEx)hardwareMap.get(DcMotor.class, "FR");
        BL = (DcMotorEx)hardwareMap.get(DcMotor.class, "BL");
        BR = (DcMotorEx)hardwareMap.get(DcMotor.class, "BR");


        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrigFL = FL.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigFR = FR.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigBL = BL.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigBR = BR.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);

        FL.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);


        // re-read coefficients and verify change.
        PIDCoefficients pidModifiedFL = FL.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidModifiedFR = FR.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidModifiedBL = BL.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidModifiedBR = BR.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (origFL)", "%.04f, %.04f, %.0f",
                    pidOrigFL.p, pidOrigFL.i, pidOrigFL.d);
            telemetry.addData("P,I,D (modifiedFL)", "%.04f, %.04f, %.04f",
                    pidModifiedFL.p, pidModifiedFL.i, pidModifiedFL.d);

            telemetry.addData("P,I,D (origFR)", "%.04f, %.04f, %.0f",
                    pidOrigFR.p, pidOrigFR.i, pidOrigFR.d);
            telemetry.addData("P,I,D (modifiedFR)", "%.04f, %.04f, %.04f",
                    pidModifiedFR.p, pidModifiedFR.i, pidModifiedFR.d);

            telemetry.addData("P,I,D (origBL)", "%.04f, %.04f, %.0f",
                    pidOrigBL.p, pidOrigBL.i, pidOrigBL.d);
            telemetry.addData("P,I,D (modifiedBL)", "%.04f, %.04f, %.04f",
                    pidModifiedBL.p, pidModifiedBL.i, pidModifiedBL.d);

            telemetry.addData("P,I,D (origBR)", "%.04f, %.04f, %.0f",
                    pidOrigBR.p, pidOrigBR.i, pidOrigBR.d);
            telemetry.addData("P,I,D (modifiedBR)", "%.04f, %.04f, %.04f",
                    pidModifiedBR.p, pidModifiedBR.i, pidModifiedBR.d);
            telemetry.update();
        }
    }
}