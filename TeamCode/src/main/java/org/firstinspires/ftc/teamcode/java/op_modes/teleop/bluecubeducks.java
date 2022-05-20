

package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

		import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
		import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
		import com.qualcomm.robotcore.hardware.DcMotor;
		import com.qualcomm.robotcore.util.ElapsedTime;

		import org.firstinspires.ftc.teamcode.java.subsystems.Capping;
		import org.firstinspires.ftc.teamcode.java.subsystems.Carousel;
		import org.firstinspires.ftc.teamcode.java.subsystems.Intake;
		import org.firstinspires.ftc.teamcode.java.subsystems.Lift;
		import org.firstinspires.ftc.teamcode.java.util.AutoDrive;
		import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@Autonomous(name = "blue cube ducks", group = "auto")
public class bluecubeducks extends LinearOpMode {
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

	//	 Send telemetry message to signify robot waiting;
		telemetry.addData("Status", "Resetting Encoders");    //
		telemetry.update();

		robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		AutoDrive ad = new AutoDrive(robot.leftMotor, robot.rightMotor, robot.imu, telemetry);
		DcMotor carouselMotor = robot.carousel;
		Lift lift = new Lift(robot.elevator);
		Intake intake = new Intake(robot.intake);
		Carousel carousel = new Carousel(carouselMotor);
		Capping capping=new Capping(robot.armServo, robot.cappingServo);
		capping.lift();
		waitForStart();


		ad.gyroDrive(AutoDrive.DRIVE_SPEED, 550, 0);
		ad.gyroTurn(AutoDrive.TURN_SPEED, 40);
		ad.gyroDrive(AutoDrive.DRIVE_SPEED, -675, 0);
		//-695

		carousel.changeDirection();
		carousel.spin(3);

		carousel.stop();

		ad.gyroTurn(AutoDrive.TURN_SPEED, -0.3);

		ad.gyroDrive(AutoDrive.DRIVE_SPEED, 730, 0);

		//lift.lift(5);

		ad.gyroTurn(AutoDrive.DRIVE_SPEED, 65);

		//lift.lift(3);

		ad.gyroDrive(AutoDrive.DRIVE_SPEED,360,90);

		lift.lift(2);

		ad.gyroDrive(AutoDrive.DRIVE_SPEED,490,90);

		intake.outtake(2);
		intake.stop();

		ad.gyroDrive(AutoDrive.TURN_SPEED, -900,90);

		lift.lower(2);

		ad.gyroHold(AutoDrive.DRIVE_SPEED,-0.5,1 );
		ad.gyroDrive(AutoDrive.DRIVE_SPEED,-180,-1);
		ad.gyroTurn(AutoDrive.TURN_SPEED, -1);





	}
}
