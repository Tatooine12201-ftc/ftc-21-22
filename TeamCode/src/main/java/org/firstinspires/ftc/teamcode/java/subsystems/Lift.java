package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Lift
{
	RobotHardware robot;
	private DcMotor lift;

	private static double LIFTING_SPEED = 1;
	private static double LOWERING_SPEED = -1;

	/**
	 * this function creates anew lift
	 * @param robot the robot hardware
	 */
	public Lift(RobotHardware robot) {
		this.robot = robot;
		this.lift = robot.elevator;
	}

	/**
	 * this function creates a alift from a motor
	 * @param lift the motor
	 */
	public Lift(DcMotor lift) {
		this.lift = lift;
	}

	/**
	 * this function creates a alift from a motor
	 * @param lift the motor
	 * @param liftingSpeed the lift speed
	 * @param loweringSpeed the lower lift speed
	 */
	public Lift(DcMotor lift, double liftingSpeed, double loweringSpeed) {
		new Lift(lift);
		LOWERING_SPEED = loweringSpeed;
		LIFTING_SPEED = liftingSpeed;
	}

	public void init()
	{
		while (lift.getCurrentPosition() > 0) {
			lift.setPower(LOWERING_SPEED);
		}
	}
	/**
	 * this function lifts
	 */
	public void lift() {

			lift.setPower(LIFTING_SPEED);

	}

	/**
	 * this function outtakes
	 */
	public void lower() {

			lift.setPower(LOWERING_SPEED);

	}

	public int getPose(){
		return lift.getCurrentPosition();
	};
	/**
	 * this function turns off the lift
	 */
	public void stop() {
		lift.setPower(0);
	}

}