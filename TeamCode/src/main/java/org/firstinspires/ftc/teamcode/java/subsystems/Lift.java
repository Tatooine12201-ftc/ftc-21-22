package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Lift
{
	RobotHardware robot;
	private final DcMotor lift;

	private static final double LIFTING_SPEED = 1;
	private static final double LOWERING_SPEED = -1;

	/**
	 * this function creates anew lift
	 * @param robot the robot hardware
	 */
	public Lift(RobotHardware robot) {
		this.robot = robot;
		this.lift = robot.elevator;
	}

	/**
	 * this function lifts
	 */
	public void lift() { lift.setPower(LIFTING_SPEED); }

	/**
	 * this function outtakes
	 */
	public void lower() { lift.setPower(LOWERING_SPEED); }

	/**
	 * this function turns off the lift
	 */
	public void stop() { lift.setPower(0); }

}