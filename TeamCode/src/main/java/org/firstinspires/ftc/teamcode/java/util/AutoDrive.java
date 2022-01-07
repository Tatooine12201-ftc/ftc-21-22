package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AutoDrive {
	private final DcMotor leftMotor;
	private final DcMotor rightMotor;
	private final BNO055IMU imu;
	private final Telemetry telemetry;


	private static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
	private static final double     DRIVE_GEAR_REDUCTION    = 18.9;     // This is < 1.0 if geared UP
	private static final double     WHEEL_DIAMETER_MM  = 4.0 * 25.4 ;     // For figuring circumference
	private static final double     WHEEL_CIRCUMFERENCE         = (  WHEEL_DIAMETER_MM * Math.PI) ;

	// These constants define the desired driving/control characteristics
	// The can/should be tweaked to suite the specific this drive train.
	public static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
	public static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

	private static final double     HEADING_THRESHOLD       = 0.5;      // As tight as we can make it with an integer gyro
	private static final double     P_TURN_COEFF            = 0.015;     // Larger is more responsive, but also less stable
	private static final double     P_DRIVE_COEFF           = 0.000009;     // Larger is more responsive, but also less stable

	public AutoDrive(DcMotor leftMotor, DcMotor rightMotor, BNO055IMU imu, Telemetry telemetry)
	{
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.imu = imu;
		this.telemetry = telemetry;
	}
	public void gyroDrive ( double speed,
	                        double distance,
	                        double angle) {

		int     newLeftTarget;
		int     newRightTarget;
		int     moveCounts;
		double  max;
		double  error;
		double  steer;
		double  leftSpeed;
		double  rightSpeed;

		// Ensure that the opmode is still active


			// Determine new target position, and pass to motor controller
			moveCounts = (int)((distance *0.76 / WHEEL_CIRCUMFERENCE)*COUNTS_PER_MOTOR_REV *(DRIVE_GEAR_REDUCTION));
			newLeftTarget = this.leftMotor.getCurrentPosition() + moveCounts;
			newRightTarget = this.rightMotor.getCurrentPosition() + moveCounts;

			// Set Target and Turn On RUN_TO_POSITION
			this.leftMotor.setTargetPosition(newLeftTarget);
			this.rightMotor.setTargetPosition(newRightTarget);

			this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// start motion.
			speed = Range.clip(Math.abs(speed), 0.0, 1.0);
			this.leftMotor.setPower(speed);
			this.rightMotor.setPower(speed);

			// keep looping while we are still active, and BOTH motors are running.
			while (
					(this.leftMotor.isBusy() && this.rightMotor.isBusy())) {

				// adjust relative speed based on heading error.
				error = getError(angle);
				steer = getSteer(error, P_DRIVE_COEFF);

				// if driving in reverse, the motor correction also needs to be reversed
				if (distance < 0)
					steer *= -1.0;

				leftSpeed = speed - steer;
				rightSpeed = speed + steer;

				// Normalize speeds if either one exceeds +/- 1.0;
				max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
				if (max > 1.0)
				{
					leftSpeed /= max;
					rightSpeed /= max;
				}

				this.leftMotor.setPower(leftSpeed);
				this.rightMotor.setPower(rightSpeed);

				// Display drive status for the driver.
				telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
				telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
				telemetry.addData("Actual",  "%7d:%7d",      this.leftMotor.getCurrentPosition(),
						this.rightMotor.getCurrentPosition());
				telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
				telemetry.update();
			}

			// Stop all motion;
			this.leftMotor.setPower(0);
			this.rightMotor.setPower(0);

			// Turn off RUN_TO_POSITION
			this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	/**
	 *  Method to spin on central axis to point in a new direction.
	 *  Move will stop if either of these conditions occur:
	 *  1) Move gets to the heading (angle)
	 *  2) Driver stops the opmode running.
	 *
	 * @param speed Desired speed of turn.
	 * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from current heading.
	 */
	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (!onHeading(speed, angle, P_TURN_COEFF)) {
			// Update telemetry & Allow time for other processes to run.
			telemetry.update();
		}
	}

	/**
	 *  Method to obtain & hold a heading for a finite amount of time
	 *  Move will stop once the requested time has elapsed
	 *
	 * @param speed      Desired speed of turn.
	 * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from current heading.
	 * @param holdTime   Length of time (in seconds) to hold the specified heading.
	 */
	public void gyroHold( double speed, double angle, double holdTime) {

		ElapsedTime holdTimer = new ElapsedTime();

		// keep looping while we have time remaining.
		holdTimer.reset();
		while ((holdTimer.time() < holdTime)) {
			// Update telemetry & Allow time for other processes to run.
			onHeading(speed, angle, P_TURN_COEFF);
			telemetry.update();
		}

		// Stop all motion;
		this.leftMotor.setPower(0);
		this.rightMotor.setPower(0);
	}

	/**
	 * Perform one cycle of closed loop heading control.
	 *
	 * @param speed     Desired speed of turn.
	 * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
	 *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                  If a relative angle is required, add/subtract from current heading.
	 * @param PCoeff    Proportional Gain coefficient
	 * @return
	 */
	private boolean onHeading(double speed, double angle, double PCoeff) {
		double   error ;
		double   steer ;
		boolean  onTarget = false ;
		double leftSpeed;
		double rightSpeed;

		// determine turn power based on +/- error
		error = getError(angle);

		if (Math.abs(error) <= HEADING_THRESHOLD) {
			steer = 0.0;
			leftSpeed  = 0.0;
			rightSpeed = 0.0;
			onTarget = true;
		}
		else {
			steer = getSteer(error, PCoeff);
			rightSpeed  = speed * steer;
			leftSpeed   = -rightSpeed;
		}

		// Send desired speeds to motors.
		this.leftMotor.setPower(leftSpeed);
		this.rightMotor.setPower(rightSpeed);

		// Display it for the driver.
		telemetry.addData("Target", "%5.2f", angle);
		telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
		telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

		return onTarget;
	}

	/**
	 * getError determines the error between the target angle and the this's current heading
	 * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
	 * @return  error angle: Degrees in the range +/- 180. Centered on the this's frame of reference
	 *          +ve error means the this should turn LEFT (CCW) to reduce error.
	 */
	private double getError(double targetAngle) {

		double thisError;

		// calculate error in -179 to +180 range  (
		thisError = targetAngle - getHeading();
		while (thisError > 180)  thisError -= 360;
		while (thisError <= -180) thisError += 360;
		return thisError;
	}

	/**
	 * returns desired steering force.  +/- 1 range.  +ve = steer left
	 * @param error   Error angle in this relative degrees
	 * @param PCoeff  Proportional Gain Coefficient
	 * @return
	 */
	private double getSteer(double error, double PCoeff) {
		return Range.clip(error * PCoeff, -1, 1);
	}

	private double getHeading() {
		return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
	}


}



