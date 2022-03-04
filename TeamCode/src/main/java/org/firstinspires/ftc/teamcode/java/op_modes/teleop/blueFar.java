package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.subsystems.Intake;
import org.firstinspires.ftc.teamcode.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.java.util.AutoDrive;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@Autonomous(name = "blue far", group = "auto")
public class blueFar  extends LinearOpMode {
	/* Declare OpMode members. */
	RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware
	private ElapsedTime runtime = new ElapsedTime();
	/**
	 * Override this method and place your code here.
	 * <p>
	 * Please do not swallow the InterruptedException, as it is used in cases
	 * where the op mode needs to be terminated early.
	 *
	 *
	 */
	@Override
	public void runOpMode()  {
		robot.init(hardwareMap);

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Status", "Resetting Encoders");    //
		telemetry.update();

		robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		AutoDrive ad = new AutoDrive(robot.leftMotor, robot.rightMotor, robot.imu, telemetry);
		Lift lift = new Lift(robot.elevator);
		Intake intake = new Intake(robot.intake);
		waitForStart();
		//lift.init();
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,640,0);
	//	while (runtime.seconds() > 2);
		{

		//	lift.lift();
		}
	//	while (runtime.seconds() > 3)
		{
		//	intake.outtake();
		}
		//while (runtime.seconds() > 4)
		{
		//	intake.stop();
		//		lift.lower();

		}
	//	while (runtime.seconds() > 5.5)
		{

		//	lift.stop();

		}


		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,-100,0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED,85);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,1500,0);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 12*2*25.4,90);
		//ad.gyroTurn(AutoDrive.TURN_SPEED,0);
		//runtime.reset();
		//while (runtime.seconds() < 2)
		{
			//lift.lift();

			//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 150 +(12*2*25.4),0);
		//	runtime.reset();
			//while (runtime.seconds() < 2)
			{
				//intake.outtake();
			}
			///ad.gyroDrive(AutoDrive.DRIVE_SPEED, (150 +(12*2*25.4)) *-1,0);
			//while (runtime.seconds() < 2)
			{
			//	lift.lower();
			}
			//ad.gyroTurn(AutoDrive.TURN_SPEED,90);
		//	ad.gyroDrive(AutoDrive.DRIVE_SPEED, (150 +(12*2*25.4)) *2,90);
			//ad.gyroTurn(AutoDrive.TURN_SPEED,90);
		}
		ad.gyroDrive(AutoDrive.DRIVE_SPEED,1000,0);
	//	while (true) {
			//telemetry.addData("left", robot.leftMotor.getCurrentPosition());
			//telemetry.addData("right", robot.rightMotor.getCurrentPosition());
			//telemetry.update();
		}




	}