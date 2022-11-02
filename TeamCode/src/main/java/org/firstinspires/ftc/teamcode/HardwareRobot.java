package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot {
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    public DcMotor encR = null;
    public DcMotor encS = null;
    public DcMotor encL = null;

    HardwareMap hdwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void HardwareMap(){

    }

    public void init(HardwareMap ahwMap){
        hdwMap = ahwMap;
        leftFront = hdwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hdwMap.get(DcMotorEx.class, "rightFront");
        leftBack = hdwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hdwMap.get(DcMotorEx.class, "rightBack");

        encR = hdwMap.get(DcMotorEx.class, "encr");
        encL = hdwMap.get(DcMotorEx.class, "encl");
        encS = hdwMap.get(DcMotorEx.class, "encs");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
