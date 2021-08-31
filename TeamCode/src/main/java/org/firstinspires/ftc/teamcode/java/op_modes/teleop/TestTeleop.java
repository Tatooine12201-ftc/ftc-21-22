package org.firstinspires.ftc.teamcode.java.op_modes.teleop;


import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class TestTeleop extends LinearOpMode {
	private DcMotor flm;
	private DcMotor blm;
	private DcMotor frm;
	private DcMotor brm;

	private Servo servo;
	private RevTouchSensor button;  // Hardware Device Object


	@Override
	public void runOpMode() throws InterruptedException {
			flm = hardwareMap.get(DcMotor.class, "FLM");
			blm = hardwareMap.get(DcMotor.class, "BLM");
			frm = hardwareMap.get(DcMotor.class, "FRM");
			brm = hardwareMap.get(DcMotor.class, "BRM");
			button = hardwareMap.get(RevTouchSensor.class, "button");
			servo = hardwareMap.get(Servo.class, "servo");

			flm.setDirection(DcMotorSimple.Direction.FORWARD);
			blm.setDirection(DcMotorSimple.Direction.FORWARD);
			frm.setDirection(DcMotorSimple.Direction.REVERSE);
			brm.setDirection(DcMotorSimple.Direction.REVERSE);
			servo.setPosition(0);
			waitForStart();
			while (opModeIsActive()) {
				if(button.isPressed()){
					servo.setPosition(1);
				}
				else {
					servo.setPosition(0);
				}


				telemetry.addData("touch",button.isPressed());
				telemetry.update();

			}
	}
}
