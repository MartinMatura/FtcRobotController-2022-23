package org.firstinspires.ftc.teamcode.autotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.Odometry;

import java.util.Arrays;

@Autonomous(name="Autonomous: Spline test")
public class AutonomousTestTwo extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    Odometry odometry = new Odometry();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     countsPerMotorRev   = 8192 ;    // eg: TETRIX Motor Encoder
    static final double     wheelDiameterCm     = 6.0 ;     // For figuring circumference
    static final double     countsPerCm         = (countsPerMotorRev / (wheelDiameterCm * Math.PI));

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        double[][] targetPoints = {{68, 0, 0},
                                   {68, 130, 0},
                                   {30, 130, 0},
                                   {68, 130, 0},
                                   {68, 0, 0},
                                   {0, 0, 0},
                                   }; //array of points we set for the robot as its path

        //double[][] splinePoints = odometry.splinePath(targetPoints);//addition to previous calc test, uses spline points for path, otherwise same
        int splinePointCounter = 0;

        waitForStart();
        resetEncoders();

        double targetX = targetPoints[splinePointCounter][0];
        double targetY = targetPoints[splinePointCounter][1];
        double targetAngle = targetPoints[splinePointCounter][2];
        double powerSmoothing = 0;//smoothes out power between moves

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
            if(odometry.distanceCheckWithAngle(currentPos[0],currentPos[1],currentPos[2],targetX,targetY,targetAngle)){
                if(splinePointCounter+1<targetPoints.length){
                    splinePointCounter++;
                    targetX = targetPoints[splinePointCounter][0];
                    targetY = targetPoints[splinePointCounter][1];
                    targetAngle = targetPoints[splinePointCounter][2];
                    powerSmoothing = -0.2;//reset power smoothing, set to negative to create small delay between moves
                }
            }

            powerSmoothing = powerSmoothing+0.025; //update power smoothing
            double[] power = odometry.calcM(targetX,targetY, targetAngle, currentPos[0],currentPos[1],currentPos[2], Range.clip(powerSmoothing,0,1));

            robot.leftFront.setPower(power[0]); //-x
            robot.leftBack.setPower(power[1]); //x
            robot.rightFront.setPower(power[2]); //x
            robot.rightBack.setPower(power[3]); //-x

            telemetry.addData("power", Arrays.toString(power));
            telemetry.addData("currentPos", Arrays.toString(currentPos));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("dX", targetX-currentPos[0]);
            telemetry.addData("dY", targetY-currentPos[1]);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("pointCounter", splinePointCounter);
            telemetry.update();
        }
    }

    private void resetEncoders(){
        robot.encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
