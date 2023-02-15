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

        //TARGET POINTS - determine path robot takes - values in each command in this order:
        // X (-left +right)
        // Y (-back +front)
        // ROT (rad. -left +right)
        // liftPos (-1100 - 0)
        // spinnerPos (0 - 1)
        // grabber (0 or 1)
        double[][] targetPoints = {
                {-130, 0, 0},
                {-130, 0, -1.571},
                {-130, 0, 0},
                {0, 0, 0},
        }; //array of points we set for the robot as its path
        /*
        double[][] targetPoints = {{0, 10, 0},
                                   {65, 10, 0},
                                   {65, 130, -0.1},
                                   {25, 130, 0},
                                   {65, 130, 0},
                                   {65, 0, 0},
                                   {0, 0, 0},
                                   }; //array of points we set for the robot as its path
        */
        int pointCounter = 0; //counts current step in targetPoints Array

        waitForStart();
        resetEncoders();

        //set starting target
        double targetX = targetPoints[pointCounter][0];
        double targetY = targetPoints[pointCounter][1];
        double targetAngle = targetPoints[pointCounter][2];
        double powerSmoothing = 0;//smoothes out power between moves

        //ODOMETRY SETUP
        double[] currentPos= {0,0,0};

        double rPrevPos;
        double rCurrPos = 0;

        double lPrevPos;
        double lCurrPos = 0;

        double sPrevPos;
        double sCurrPos = 0;

        //MAIN LOOP
        while(opModeIsActive()){

            //update delta encoder values
            rPrevPos = rCurrPos;
            rCurrPos = robot.encR.getCurrentPosition();
            double dR = (rCurrPos - rPrevPos)/countsPerCm;

            lPrevPos = lCurrPos;
            lCurrPos = robot.encL.getCurrentPosition();
            double dL = (lCurrPos - lPrevPos)/countsPerCm;

            sPrevPos = sCurrPos;
            sCurrPos = robot.encS.getCurrentPosition();
            double dS = (sCurrPos - sPrevPos)/countsPerCm;

            currentPos = odometry.nowPos(currentPos, dR, dL, dS); //get new position

            //check if close enough to target destination
            if(odometry.distanceCheckWithAngle(currentPos[0],currentPos[1],currentPos[2],targetX,targetY,targetAngle)){
                if(pointCounter+1<targetPoints.length){
                    pointCounter++;
                    targetX = targetPoints[pointCounter][0];
                    targetY = targetPoints[pointCounter][1];
                    targetAngle = targetPoints[pointCounter][2];
                    powerSmoothing = -0.3;//reset power smoothing, set to negative to create small delay between moves
                }
            }

            powerSmoothing = powerSmoothing+0.05; //update power smoothing
            double[] power = odometry.calcM(targetX,targetY, targetAngle, currentPos[0],currentPos[1],currentPos[2], Range.clip(powerSmoothing,0,1));

            //set new power
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
            telemetry.addData("pointCounter", pointCounter);
            telemetry.update();
        }
    }

    private void resetEncoders(){
        robot.encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
