package org.firstinspires.ftc.teamcode.autotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.Odometry;

import java.util.Arrays;

@Autonomous(name="Autonomous: test")
public class AutonomousTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    Odometry odometry = new Odometry();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     countsPerMotorRev   = 8192 ;    // eg: TETRIX Motor Encoder
    static final double     wheelDiameterCm     = 6.0 ;     // For figuring circumference
    static final double     countsPerCm         = (countsPerMotorRev / (wheelDiameterCm * Math.PI));

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        resetEncoders();

        double[][] targetPoints= {{0, 0, 0},
                                  {100, 0, 0},
                                  {100, 50, 0},
                                  {100, 50, 90},
                                  {50, 0, 0} }; //array of points we set for the robot as its path
        int targetCounter = 0;

        double[] currentPos= {0,0,0};

        double rPrevPos;
        double rCurrPos = 0;

        double lPrevPos;
        double lCurrPos = 0;

        double sPrevPos;
        double sCurrPos = 0;

        while(opModeIsActive()){
            rPrevPos = rCurrPos;
            rCurrPos = robot.encR.getCurrentPosition();
            double dR = (rCurrPos - rPrevPos)/countsPerCm;

            lPrevPos = lCurrPos;
            lCurrPos = robot.encL.getCurrentPosition();
            double dL = (lCurrPos - lPrevPos)/countsPerCm;

            sPrevPos = sCurrPos;
            sCurrPos = robot.encS.getCurrentPosition();
            double dS = (sCurrPos - sPrevPos)/countsPerCm;

            currentPos = odometry.nowPos(currentPos, dR, dL, dS);

            if(odometry.distanceCheck(currentPos[0], currentPos[1], currentPos[2], targetPoints[targetCounter][0], targetPoints[targetCounter][1])){
                targetCounter++;
            }

            double[] power = odometry.calc(targetPoints[targetCounter][0], targetPoints[targetCounter][1], targetPoints[targetCounter][2], currentPos[0],currentPos[1],currentPos[2]);

            robot.leftFront.setPower(power[0]);
            robot.leftBack.setPower(power[1]);
            robot.rightFront.setPower(power[2]);
            robot.rightBack.setPower(power[3]);

            telemetry.addData("power", Arrays.toString(power));
            telemetry.addData("target", Arrays.toString(targetPoints[targetCounter]));
            telemetry.update();
        }
    }

    private void resetEncoders(){
        robot.encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
