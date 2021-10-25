package org.firstinspires.ftc.teamcode.java.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

public class Intake extends RobotHardware
{

    private DcMotorEx intake;

    public static double intake_speed = 1;
    public static double outtake_speed = -1;


    public void intake(double speed)
    {

        intake.setPower(intake_speed);
    }

    public void outtake()
    {
        intake.setPower(outtake_speed);
    }

    public void stop(){
        intake(0.0);
    }

    public void debug(){

    }
}