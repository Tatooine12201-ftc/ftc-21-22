package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Intake
{
    RobotHardware robot;
    private final DcMotor intake;

    private static final double INTAKE_SPEED = -0.5;
    private static final double OUTTAKE_SPEED = 1;

    private static final double OPEN = 1;
    private static final double CLOSED = 0;


    /**
     * this function creates anew intake
     * @param robot the robot hardware
     */
    public Intake(RobotHardware robot) {
        this.robot = robot;
        this.intake = robot.intake;

    }

    public Intake(DcMotor intake ) {
        this.intake = intake;


    }

    /**
     * this function intakes
     */
    public void intake() {

        intake.setPower(INTAKE_SPEED);
    }
    public void intake(int sec){
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while ((spintime.time() < sec)) {
            intake ();
        }
        stop();
    }
    /**
     * this function outtakes
     */
    public void outtake() {

        intake.setPower(OUTTAKE_SPEED);
    }
    public void outtake (int sec){
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while ((spintime.time() < sec)) {
            intake ();
        }
        stop();
    }
    /**
     * this function turns off the intake
     */
    public void stop() {
        intake.setPower(0);
    }

    public void debug() {}
}