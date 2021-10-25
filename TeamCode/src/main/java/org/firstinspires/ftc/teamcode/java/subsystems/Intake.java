package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Intake
{
    RobotHardware robot;
    private DcMotorEx intake;

    public static double intake_speed = 1;
    public static double outtake_speed = -1;

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
    public void intake() { intake.setPower(intake_speed); }

    /**
     * this function outtakes
     */
    public void outtake() { intake.setPower(outtake_speed); }

    /**
     * this function turns off the intake
     */
    public void stop() { intake.setPower(0); }

    public void debug() {}
}