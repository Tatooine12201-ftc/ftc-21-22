package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Intake
{
    RobotHardware robot;
    private final DcMotor intake;
    private final Servo intakeServo;
    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = -1;

    private static final double OPEN = 1;
    private static final double CLOSED = 0;


    /**
     * this function creates anew intake
     * @param robot the robot hardware
     */
    public Intake(RobotHardware robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.intakeServo = robot.intakeServo;
    }

    public Intake(DcMotor intake) {
        this.intake = intake;
    }

    /**
     * this function intakes
     */
    public void intake() {
        intakeServo.setPosition(CLOSED);
        intake.setPower(INTAKE_SPEED);
    }

    /**
     * this function outtakes
     */
    public void outtake() {
        intakeServo.setPosition(OPEN);
        intake.setPower(OUTTAKE_SPEED);
    }

    /**
     * this function turns off the intake
     */
    public void stop() {
        intakeServo.setPosition(CLOSED);
        intake.setPower(0);
    }

}