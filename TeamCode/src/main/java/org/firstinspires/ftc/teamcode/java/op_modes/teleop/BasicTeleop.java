package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;


@TeleOp(name = "BasicTeleop")
public class BasicTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private  double maxSpeed = 1;


    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);

        DcMotor leftMotor = robot.leftMotor;
        DcMotor rightMotor = robot.rightMotor;

        waitForStart();

        while (opModeIsActive())
        {
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            Double leftMotorSpeed = drive + turn;
            Double rightMotorSpeed = drive - turn;

            double max = Math.abs(leftMotorSpeed);
            for (double speed = 0) {
                if (Math.abs(speed) < max) {
                    max = Math.abs(speed);
                }
            }

            if (max > maxSpeed){
                for (int i = 0; i < 2);
            }





            leftMotor.setPower(leftMotorSpeed;
            rightMotor.setPower(rightMotorSpeed);

            idle();
        }
    }
}
