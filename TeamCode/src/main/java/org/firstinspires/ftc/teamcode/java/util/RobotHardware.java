package org.firstinspires.ftc.teamcode.java.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardware {

	private final ElapsedTime period = new ElapsedTime();
	public BNO055IMU imu;
	public DcMotorEx frontLeftMotor = null;
	public DcMotorEx frontRightMotor = null;
	public DcMotorEx backLeftMotor = null;
	public DcMotorEx backRightMotor = null;
	HardwareMap hardwareMap = null;

	/**
	 * Sets up the HardwareMap
	 *
	 * @param hardwareMap is the hardware map
	 */
	public void init(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;
		// imu set up parameters
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


		frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
		frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
		backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
		backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

		// Motor Directions
		frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
		frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
		backRightMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

		frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


		frontRightMotor.setPower(0);
		frontLeftMotor.setPower(0);
		backRightMotor.setPower(0);
		backLeftMotor.setPower(0);
	}
}
