package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Intake
{
    RobotHardware robot;
    private final DcMotor intake;

    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = -1;

    /**
     * this function creates anew intake
     * @param robot the robot hardware
     */
    public Intake(RobotHardware robot) {
        this.robot = robot;
        this.intake = robot.intake;
    }

    /**
     * this function intakes
     */
    public void intake() { intake.setPower(INTAKE_SPEED); }

    /**
     * this function outtakes
     */
    public void outtake() { intake.setPower(OUTTAKE_SPEED); }

    /**
     * this function turns off the intake
     */
    public void stop() { intake.setPower(0); }

}