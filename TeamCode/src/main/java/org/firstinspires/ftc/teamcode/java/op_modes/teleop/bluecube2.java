package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.subsystems.Intake;
import org.firstinspires.ftc.teamcode.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.java.util.AutoDrive;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class bluecube2 {
    @Autonomous(name = "bluecube3", group = "auto")
    public class bluecube3  extends LinearOpMode {
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

            DcMotor carouselMotor = robot.carousel;
            AutoDrive ad = new AutoDrive(robot.leftMotor, robot.rightMotor, robot.imu, telemetry);
            Intake intake = new Intake(robot.intake);
            Lift lift = new Lift(robot.elevator);
            ad.gyroDrive(AutoDrive.DRIVE_SPEED,560 , 0);
            lift.lift(4);
            intake.outtake(2);
            intake.stop();
            lift.lower(2);
            lift.stop();
            ad.gyroTurn(AutoDrive.TURN_SPEED,90);
            ad.gyroDrive(AutoDrive.DRIVE_SPEED, 1600 , 0);
        }
    }




    }
