package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Capping
{
    RobotHardware robot;
    private final DcMotor cappingLift;
    private final Servo cappingServo;


    private static final double LIFTING_SPEED = 1;
    private static final double LOWERING_SPEED = -1;

    private static final double OPEN_ARM = 1;
    private static final double CLOSED_ARM = 0;

    private boolean isOpen = false;


    /**
     * this function creates anew lift
     * @param robot the robot hardware
     */
    public CappingLift(RobotHardware robot) {
        this.robot = robot;
        this.lift = robot.elevator;
        this.cappingServo = robot.cappingServo;
    }

    /**
     * This function sets the servo to open
     */

    public void open() {
        cappingServo.setPosition(OPEN_ARM);
    }

    /**
     * This function sets the servo to close
     */

    public void close() {
        cappingServo.setPosition(CLOSED_ARM);
    }

    /**
     * This function changes the position of the arm
     */

    public void changeGrabbingDirection() {
        if (isOpen){
            cappingServo = cappingServo.close();
            isOpen = false;
        }else {
            cappingServo = cappingServo.open();
            isOpen = true;
        }
    }

    /**
     * this function lifts
     */
    public void liftcapping() {
        cappingLift.setPower(LIFTING_SPEED);

    }

    /**
     * this function outtakes
     */
    public void lowercapping() {
        cappingLift.setPower(LOWERING_SPEED);
    }

    /**
     * this function turns off the lift
     */
    public void stop() {
        cappingServo.setPosition(CLOSED_ARM);
        cappingLift.setPower(0);
    }

}