package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

@Autonomous(name="test Drive By encoder", group="auto")
public class basicAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    private final double MAX_SPEED = 1;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 18.9;
    static final double WHEEL_CIRCUMFERENCE_MM = 4.0 * 25.4 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // defining motors
        DcMotor leftMotor = robot.leftMotor;
        DcMotor rightMotor = robot.rightMotor;
        BNO055IMU imu = robot.imu;
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        drive(0.5 , 1000,1000);


    }

    public void drive(double speed, double leftMM, double rightMM) {
        double rightTargets = 0;
        double leftTargets = 0;
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            leftTargets = robot.leftMotor.getCurrentPosition() +  (leftMM * DRIVE_COUNTS_PER_MM);
            rightTargets = robot.rightMotor.getCurrentPosition() +  (rightMM * DRIVE_COUNTS_PER_MM);
            robot.leftMotor.setTargetPosition((int) leftTargets);
            robot.rightMotor.setTargetPosition((int) rightTargets);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftTargets,  rightTargets);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}