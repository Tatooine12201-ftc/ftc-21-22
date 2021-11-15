package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

import java.util.Calendar;
import java.util.List;

public class Capping
{
    RobotHardware robot;

    private final Servo cappingServo;
    private final Lift lift;

    public Capping(RobotHardware robot) {
        this.robot = robot;
        cappingServo = robot.cappingServo;
        this.lift = new Lift(robot.cappingLift);
    }

    public Capping(DcMotor lift, Servo cappingServo) {
        this.lift = new Lift(lift);
        this.cappingServo = cappingServo;
    }

    private static final double LIFTING_SPEED = 1;
    private static final double LOWERING_SPEED = -1;

    private static final double OPEN_ARM = 1;
    private static final double CLOSED_ARM = 0;

    private boolean isOpen = false;

    /**
     * this function is opening the servo
     */
    public void open() {
        cappingServo.setPosition(OPEN_ARM);
        isOpen = true;
    }

    /**
     * this function is closing the servo
     */
    public void close() {
        cappingServo.setPosition(CLOSED_ARM);
        isOpen = false;
    }

    /**
     * this function is opening or closing the servo if open it will close
     */
    public void changePosition(){
        if (isOpen){
            close();
        }
        else
        {
            open();
        }
    }

    /**
     * rises the capping lift
     */
    public void lift() {
        close();
        lift.lift();
    }

    /**
     * lower the capping lift
     */
    public void  lower(){
        open();
        lift.lower();
    }

    /**
     * stop the capping lift
     */
    public void stop(){
        close();
        lift.stop();
    }

}