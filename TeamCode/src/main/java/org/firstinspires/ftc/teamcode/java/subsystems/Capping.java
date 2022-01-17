//package org.firstinspires.ftc.teamcode.java.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
//
//import java.util.Calendar;
//import java.util.List;
//
//public class Capping
//{
//    RobotHardware robot;
//
//    private final Servo cappingServo;
//    private final Servo arm;
//
//    public Capping(RobotHardware robot) {
//        this.robot = robot;
//        cappingServo = robot.cappingServo;
//        this.arm    = (robot.cappingLift);
//    }
//
//    public Capping(DcMotor arm, Servo cappingServo) {
//        this.arm = (arm);
//        this.cappingServo = cappingServo;
//    }
//
//    private static final double LIFTING_SPEED = 0.1;
//    private static final double LOWERING_SPEED = -0.1;
//
//    private static final double OPEN_ARM = 1;
//    private static final double CLOSED_ARM = 0;
//
//    private boolean isOpen = false;
//
//    /**
//     * this function is opening the servo
//     */
//    public void open() {
//        cappingServo.setPosition(OPEN_ARM);
//        isOpen = true;
//    }
//
//    /**
//     * this function is closing the servo
//     */
//    public void close() {
//        cappingServo.setPosition(CLOSED_ARM);
//        isOpen = false;
//    }
//
//    /**
//     * this function is opening or closing the servo if open it will close
//     */
//    public void changePosition(){
//        if (isOpen){
//            close();
//        }
//        else
//        {
//            open();
//        }
//    }
//
//    /**
//     * rises the capping lift
//     */
//    public void lift(double power) {
//        close();
//        if(arm.getCurrentPosition() < 500){
//            arm.setPower(LIFTING_SPEED *power) ;
//        }
//    }
//
//    /**
//     * lower the capping lift
//     */
//    public void  lower(double power){
//        if(arm.getCurrentPosition() > 100){
//            arm.setPower(LOWERING_SPEED * power);
//        }
//    }
//
//    /**
//     * stop the capping lift
//     */
//    public void stop() {
//        close();
//        arm.setPower(0);
//    }
//
//    }