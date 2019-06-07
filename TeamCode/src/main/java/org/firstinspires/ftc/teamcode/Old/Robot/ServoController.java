package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by August on 4/17/2017.
 */

public class ServoController {
    Robot robot;
    public ServoController(Robot robot){
        this.robot = robot;
    }

    // -------------------- Operations -------------------- //
    public void clamp() {
        setClamps(0.7);
    }

    public void unClamp() {
        setClamps(0.51);
    }

    public void clampGentle() {
        setClamps(0.6);
    }

    public void dumperUp() {
        setDumpers(0.89);
    }

    public void dumperDown() {
        setDumpers(0.173);
    }

    public void collectorsIn () {
        setCollectorPos(0);
    }

    public void collectorsOut () {
        setCollectorPos(1);
    }

    public void rightCollectorOut () {
        robot.collectionDeployL.setPosition(0);
        robot.collectionDeployR.setPosition(1);
    }

    public void leftCollectorOut () {
        robot.collectionDeployL.setPosition(1);
        robot.collectionDeployR.setPosition(0);
    }

    public void servoStopsDown () {
        setServoStops(0.5);
    }

    public void servoStopsUp () {
        setServoStops(0);
    }

    public void jewelArmDown() {
        setJewelArm(0.27);
    }

    public void jewelArmUp() {
        setJewelArm(0.85);
    }

    public void jewelPivotRight() {
        setJewelPivot(0.7);
    }

    public void jewelPivotLeft() {
        setJewelPivot(0.3);
    }

    public void jewelPivotCenter() {
        setJewelPivot(0.5);
    }

    // -------------------- Control ----------------------- //
    private void setClamps(double target){
        robot.clampR.setPosition(target);
        robot.clampL.setPosition(target);
    }

    private void setDumpers(double target){
        robot.dump.setPosition(target);
    }

    private void setCollectorPos (double target) {
        robot.collectionDeployL.setPosition(target);
        robot.collectionDeployR.setPosition(target);
    }

    private void setServoStops(double target) {
        robot.rightServoStop.setPosition(target);
        robot.leftServoStop.setPosition(target);
    }

    private void setJewelPivot(double target) {
        robot.jewelPivot.setPosition(target);
    }

    private void setJewelArm(double target) {
        robot.jewelArm.setPosition(target);
    }
}
