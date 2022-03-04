package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Carousel {
	RobotHardware robot;
	private final DcMotor carousel;
	private static final double CAROUSEL_SPEED = 1;

	public boolean isReversed = false;

	/**
	 * this function is the constructor for the carousel subsystem
	 * @param robot the robot hardware
	 */
	public Carousel(RobotHardware robot) {
		this.robot = robot;
		this.carousel = robot.carousel;
	}

	public Carousel(DcMotor carousel) {
		this.carousel = carousel;
	}

	/**
	 * this function stops the carousel
	 */
	public void stop() {
		carousel.setPower(0);
	}

	/**
	 * this function starts the carousel
	 */
	public void spin (){
		carousel.setPower(CAROUSEL_SPEED);
	}

	public void spin(int sec){
		ElapsedTime spintime = new ElapsedTime();
		spintime.reset();
		while ((spintime.time() < sec)) {
			spin();
		}
		stop();
	}

	/**
	 * this function changes the direction of the carousel
	 */
	public void changeDirection() {
		if (isReversed == false){
			carousel.setDirection(DcMotor.Direction.REVERSE);
			isReversed = true;
		}
		else if (isReversed == true){
			carousel.setDirection(DcMotor.Direction.FORWARD);
			isReversed = false;
		}
	}
}

