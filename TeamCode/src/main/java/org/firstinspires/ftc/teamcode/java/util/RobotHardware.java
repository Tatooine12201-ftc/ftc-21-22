package org.firstinspires.ftc.teamcode.java.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardware {
	public HardwareMap hardwareMap;
	//DRIVE motors
	public DcMotor carusella = null;
	public DcMotorEx leftMotor = null;
	public DcMotorEx rightMotor = null;
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
		leftMotor = hardwareMap.get(DcMotorEx.class, "left_Motor");
		rightMotor = hardwareMap.get(DcMotorEx.class, "right_Motor");

		carusella = hardwareMap.get(DcMotor.class, "carusella");


		leftMotor.setPower(0);
		rightMotor.setPower(0);
		carusella.setPower(0);

		leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		carusella.setDirection(DcMotorSimple.Direction.FORWARD);

		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		carusella.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
}
