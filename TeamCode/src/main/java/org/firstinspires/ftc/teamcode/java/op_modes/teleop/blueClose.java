package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.subsystems.Intake;
import org.firstinspires.ftc.teamcode.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.java.util.AutoDrive;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@Autonomous(name = "park blue", group = "auto")
public class blueClose  extends LinearOpMode {
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
	public void runOpMode() {
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

		ad.gyroDrive(AutoDrive.DRIVE_SPEED, 700,0);


		//ad.gyroTurn(AutoDrive.TURN_SPEED, 90);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 50, 0);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 680, 0);;
		//ad.gyroTurn(AutoDrive.TURN_SPEED, 35.5);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,-400,0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED, -30);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,200,0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED,90 );
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,-750,0);
		ad.gyroTurn(AutoDrive.DRIVE_SPEED, -90);
		ad.gyroDrive(AutoDrive.DRIVE_SPEED, 550,0);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED,800,0);
	//	ad.gyroTurn(AutoDrive.TURN_SPEED, 1800);

		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 1100, 0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED, 90);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 600, 0);
		;
		//while (runtime.seconds() < 2) {
			//lift.lower();

		//}
		//runtime.reset();
		//while (runtime.seconds() < 2) {
		//	intake.outtake();

		//}
		//while (runtime.seconds() < 3){
			//lift.lift();
		//}
		//while (runtime.seconds() < 5){
			//lift.stop();
			//intake.stop();
		//}
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, -1030, 0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED, -45);
		//runtime.reset();
		//while (runtime.seconds() < 2) {
			//intake.intake();

		//ad.gyroTurn(AutoDrive.TURN_SPEED, 45);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 1030, 0);
		//}
		//runtime.reset();
		//while (runtime.seconds() < 2) {
			//lift.lower();
		//}
		//while (runtime.seconds() < 4){
			//intake.outtake();
		//}
		//while (runtime.seconds() < 5){
		//	intake.stop();
		//	lift.lift();

		//}
		//while (runtime.seconds() < 7){
		//	lift.stop();
		//}


		//runtime.reset();


		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, -1150, 0);
		//ad.gyroTurn(AutoDrive.TURN_SPEED, 90);
		//ad.gyroDrive(AutoDrive.DRIVE_SPEED, 300, 0);
	}
}