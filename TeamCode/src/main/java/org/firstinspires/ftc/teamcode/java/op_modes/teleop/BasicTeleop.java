package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.java.subsystems.Capping;
import org.firstinspires.ftc.teamcode.java.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.java.subsystems.Intake;
import org.firstinspires.ftc.teamcode.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

import java.util.List;
import java.util.Set;


@TeleOp(name = "BasicTeleop")


public class BasicTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    private final double MAX_SPEED = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        // defining motors
        DcMotor leftMotor = robot.leftMotor;
        DcMotor rightMotor = robot.rightMotor;
        DcMotor elevaterMotor = robot.elevator;
        DcMotor carouselMotor = robot.carousel;
        DcMotor intakeMotor = robot.intake;
        Servo intakeServo = robot.intakeServo;
        Servo cappingServo = robot.cappingServo;

        Lift lift = new Lift(elevaterMotor);
        Carousel carousel = new Carousel(carouselMotor);
        Intake intake = new Intake(intakeMotor, intakeServo);
        Capping capping = new Capping(carouselMotor, cappingServo);

        boolean isSecondLift = false;



        // creating an array for the motor speeds
        double[] motorSpeeds = new double[2];

        waitForStart();

        while (opModeIsActive()) {

            // correlating gamepad sticks to driving states
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

                if (gamepad2.b) {
                    capping.changePosition();
                }
                if (gamepad2.y) {
                    carousel.spin();
                } else if (gamepad2.back) {
                    carousel.changeDirection();
                }
                if (gamepad2.dpad_down && !isSecondLift) {
                    lift.lower();

                }
                else if (gamepad2.dpad_down && isSecondLift){
                    capping.lower();
                }
                if (gamepad2.dpad_up && !isSecondLift) {
                    lift.lift();

                }
                else if (gamepad2.dpad_up && isSecondLift){
                    capping.lift();
                }
                if (gamepad2.start && isSecondLift){
                    isSecondLift = false;
                }
                else if ( gamepad2.start && isSecondLift == false){
                isSecondLift = true;
                }
                if (gamepad2.right_bumper) {
                    intake.intake();
                } else if (gamepad2.left_bumper) {
                    intake.outtake();
                }


                // creating array for motor speeds that takes a value from the different driving states
                motorSpeeds[0] = drive + turn;
                motorSpeeds[1] = drive - turn;


                // defining max speed for motors
                double max = Math.max(Math.abs(motorSpeeds[0]), Math.abs(motorSpeeds[1]));


                // creating a stable range for the motor max speed
                if (max > 1) {
                    for (int i = 0; i < 2; i++) {
                        motorSpeeds[i] = motorSpeeds[i] / max;
                    }
                }


                // setting power to the motors based the values of the speeds from the
                // gamepad and max speed
                leftMotor.setPower(motorSpeeds[0] * MAX_SPEED);
                rightMotor.setPower(motorSpeeds[1] * MAX_SPEED);
                capping.stop();
                lift.stop();
                intake.stop();
                carousel.stop();
            }
        }
    }


