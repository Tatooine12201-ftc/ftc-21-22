package org.firstinspires.ftc.teamcode.java.util;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardware {
	public HardwareMap hardwareMap;
	//DRIVE motors
	public DcMotor carousel = null;
	public DcMotorEx leftMotor = null;
	public DcMotorEx rightMotor = null;
	public DcMotorEx intake = null;
	public DcMotorEx elevator = null;

	public Servo armServo = null;
	public Servo cappingServo =null;

	public BNO055IMU imu = null;
	/**
	 * Sets up the HardwareMap
	 *
	 * @param hardwareMap is the hardware map
	 */
	public void init(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		// Parts in hardware map
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

		// config
		leftMotor = hardwareMap.get(DcMotorEx.class, "Left_Motor");
		rightMotor = hardwareMap.get(DcMotorEx.class, "Right_Motor");

		intake = hardwareMap.get(DcMotorEx.class,"Intake");
		elevator = hardwareMap.get(DcMotorEx.class, "Elevator");
		carousel = hardwareMap.get(DcMotor.class, "Carousel");
		//cappingLift = hardwareMap.get(DcMotor.class, "Capping Lift");

		armServo = hardwareMap.get(Servo.class,"armServo");
		cappingServo = hardwareMap.get(Servo.class, "cappingServo");
		elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//cappingLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//cappingLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		intake.setPower(0);
		leftMotor.setPower(0);
		rightMotor.setPower(0);
		carousel.setPower(0);
		elevator.setPower(0);
		//cappingLift.setPower(0);
		armServo.setPosition(1);
		cappingServo.setPosition(1);

		leftMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setDirection(DcMotor.Direction.REVERSE);
		carousel.setDirection(DcMotor.Direction.FORWARD);
		intake.setDirection(DcMotor.Direction.REVERSE);
		carousel.setDirection(DcMotor.Direction.FORWARD);
		elevator.setDirection(DcMotor.Direction.FORWARD);
		//armServo.setDirection(FORWARD);
		//cappingLift.setDirection(DcMotor.Direction.FORWARD);

		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		//cappingLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
}
