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
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;

    static final double     WHEEL_DIAMETER_MM   = 25.4  * 4.0 ;

    @Override
    public void runOpMode()  {
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


    }
}
