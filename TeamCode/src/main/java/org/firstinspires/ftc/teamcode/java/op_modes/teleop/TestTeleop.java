package org.firstinspires.ftc.teamcode.java.op_modes.teleop;


import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class TestTeleop extends LinearOpMode {
	private DcMotor flm;
	private DcMotor blm;
	private DcMotor frm;
	private DcMotor brm;

	private Servo servo;
	private RevTouchSensor button;  // Hardware Device Object
	private DistanceSensor distance;

	@Override
	public void runOpMode() throws InterruptedException {
		flm = hardwareMap.get(DcMotor.class, "FLM");
		blm = hardwareMap.get(DcMotor.class, "BLM");
		frm = hardwareMap.get(DcMotor.class, "FRM");
		brm = hardwareMap.get(DcMotor.class, "BRM");
		button = hardwareMap.get(RevTouchSensor.class, "TS");
		servo = hardwareMap.get(Servo.class, "SW");
		distance = hardwareMap.get(DistanceSensor.class, "DS");

		flm.setDirection(DcMotorSimple.Direction.FORWARD);
		blm.setDirection(DcMotorSimple.Direction.FORWARD);
		frm.setDirection(DcMotorSimple.Direction.REVERSE);
		brm.setDirection(DcMotorSimple.Direction.REVERSE);
		servo.setPosition(0);
		waitForStart();
		int direc =0;
		while (opModeIsActive()) {
			double dis = distance.getDistance(DistanceUnit.METER);
			if(button.isPressed()){
				direc =1;
			}
			else {
				direc = -1;
			}
			flm.setPower(direc*dis);
			blm.setPower(direc*dis);
			frm.setPower(direc*dis);
			brm.setPower(direc*dis);

			telemetry.addData("touch",button.isPressed());
			telemetry.addData("dis",dis);
			telemetry.update();

		}
	}
}
