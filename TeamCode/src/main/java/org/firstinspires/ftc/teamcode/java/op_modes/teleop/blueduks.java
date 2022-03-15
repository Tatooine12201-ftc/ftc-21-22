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

@Autonomous(name = "blueDucks", group = "auto")
public class blueduks  extends LinearOpMode {
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
        DcMotor intakeMotor = robot.intake;
        AutoDrive ad = new AutoDrive(robot.leftMotor, robot.rightMotor, robot.imu, telemetry);
        Carousel carousel = new Carousel(carouselMotor);
        Intake intake = new Intake(intakeMotor);
        Capping capping=new Capping(robot.armServo, robot.cappingServo);
        Lift lift = new Lift(robot.elevator);
        capping.lift();
        waitForStart();
        //Lift lift = new Lift(robot.elevator);
        ad.gyroDrive(AutoDrive.DRIVE_SPEED, 550, 0);
        ad.gyroTurn(AutoDrive.TURN_SPEED, 38);
        ad.gyroDrive(AutoDrive.DRIVE_SPEED, -697, 0);

        carousel.changeDirection();
        carousel.spin(5);

        carousel.stop();

        //ad.gyroTurn(AutoDrive.TURN_SPEED, -38);
        //ad.gyroDrive(AutoDrive.DRIVE_SPEED, 50, 0);
        ad.gyroTurn(AutoDrive.TURN_SPEED, -1);
        ad.gyroDrive(AutoDrive.DRIVE_SPEED, 570, 0);
        intake.outtake(3);
        intake.stop();
       // lift.lift(1);
       // intake.outtake(3);
       // lift.lower(2);
       // lift.stop();
    }

}


