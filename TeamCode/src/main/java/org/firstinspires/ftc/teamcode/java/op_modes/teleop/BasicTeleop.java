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

        double motorSpeeds [] = new double[2];

        waitForStart();

        while (opModeIsActive())
        {
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            motorSpeeds[0] = drive + turn;
            motorSpeeds[1] = drive - turn;

            double max = Math.max(Math.abs(motorSpeeds[0]),Math.abs(motorSpeeds[1]));

            if (max > 1)
            {
                for(int i = 0 ; i < 2; i++)
                {
                    motorSpeeds[i] = motorSpeeds[i] / max;
                }
            }







            leftMotor.setPower(motorSpeeds[0] * maxSpeed);
            rightMotor.setPower(motorSpeeds[1] * maxSpeed);
        }
    }
}
