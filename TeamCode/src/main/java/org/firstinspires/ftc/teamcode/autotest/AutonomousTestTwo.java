package org.firstinspires.ftc.teamcode.autotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        // spinnerPos (0 - 1) when lift down(0.75=pickup,1=up)
        // grabber (0.3 - closed, 0.1 open)
        double pi = 3.1416;
        double[][] targetPoints = {
                {0,    0, 0,     0,    1,  0.3}, //grab cone tight
                {-97, 0, 0,    -820,  0.5, 0.3}, //go to pole, lift lift
                {-97, -8.5, 0,    -820,  0.5, 0.3}, //move slightly back
                {-97, -8.5, 0,    -820,  0.45, 0.1}, //drop cone
                {-97, 0, 0,    -820,  0.6, 0.3}, //move forward again
                {-127, 0, 0,    -820, 0.6, 0.1}, //move to intersection
                {-127, 0, 0.14,    0,  1, 0.1}, //drop lift

                /* old method
                {0,    0, 0,     0,    1,  0.3}, //grab cone tight
                {-125, 0, 0,    -775,  0.5, 0.3}, //go to pole
                {-125, 0, pi/4, -775,  0.5, 0.3}, //rotate
                {-125, 0, pi/4, -775,  0.5, 0.1}, //release cone
                {-125, 0, pi/2, -775,  0.5, 0.1}, //rotate back
                {-127, 0, pi/2,  0,    1,  0.1}, //drop lift, slightly move to center of intersection
                {-127, 0, pi,    0,    1,  0.1}, //rotate fully to cone stack
                {-130, -52, pi,    0,    0.75,  0.1}, //move towards cone stack
                {-130, -52, pi,    0,    0.75,  0.3}, //grab cone from stack
                {-127, 0, pi,     -1100,    1,  0.3}, //move to intersection
                {-127, 0, 7/4*pi, -1100,  0.3,  0.3}, //rotate on intersection
                {-127, 0, 7/4*pi, -1100,  0.3,  0.1}, //drop second cone
                {-127, 0, pi,     -1100,    1,  0.1}, //rotate back
                {-127, -52, pi,    0,    0.75,  0.1}, //drop lift, move to cone stack
                */
                //{-100, 0, 0, 0,  1,    0.1},
                //{-100, 0, 0, -100,  1,    0.1},
        }; //array of points we set for the robot as its path

        int pointCounter = 0; //counts current step in targetPoints Array

        waitForStart();
        resetEncoders();

        //set starting targets
        double targetX = targetPoints[pointCounter][0];
        double targetY = targetPoints[pointCounter][1];
        double targetAngle = targetPoints[pointCounter][2];
        setLiftPos(targetPoints[pointCounter][3]);
        double targetSpinner = targetPoints[pointCounter][4];
        double targetGrabber = targetPoints[pointCounter][5];

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


            //UPDATE POWER - X, Y, ROTATION
            powerSmoothing = powerSmoothing+0.05; //update power smoothing
            double[] power = odometry.calcM(targetX,targetY, targetAngle, currentPos[0],currentPos[1],currentPos[2], Range.clip(powerSmoothing,0,1));
            robot.leftFront.setPower(power[0]); //-x
            robot.leftBack.setPower(power[1]); //x
            robot.rightFront.setPower(power[2]); //x
            robot.rightBack.setPower(power[3]); //-x

            //UPDATE LIFT
            runLift(0.5,0.1, false);

            //UPDATE SPINNER
            robot.spinner.setPosition(targetSpinner);

            //UPDATE GRABBER - closed or open
            robot.grabber.setPosition(targetGrabber);

            //check if close enough to target destination
            if(odometry.distanceCheckWithAngle(currentPos[0],currentPos[1],currentPos[2],targetX,targetY,targetAngle)){
                if(Math.abs(robot.grabber.getPosition() - targetGrabber) < 0.01){//check if grabber in correct position
                    if(Math.abs(robot.lift.getCurrentPosition() - robot.lift.getTargetPosition())<10){ //check if lift in correct position
                        if(pointCounter+1<targetPoints.length){
                            pointCounter++; //increment point counter, new targets
                            powerSmoothing = -0.3;//reset power smoothing, set to negative to create small delay between moves

                            targetX = targetPoints[pointCounter][0];
                            targetY = targetPoints[pointCounter][1];
                            targetAngle = targetPoints[pointCounter][2];
                            targetSpinner = targetPoints[pointCounter][4];
                            targetGrabber = targetPoints[pointCounter][5];
                            setLiftPos(targetPoints[pointCounter][3]);
                        }
                    }
                }
            }

            telemetry.addData("power", Arrays.toString(power));
            telemetry.addData("currentPos", Arrays.toString(currentPos));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("dX", targetX-currentPos[0]);
            telemetry.addData("dY", targetY-currentPos[1]);
            telemetry.addData("dRot", targetAngle-currentPos[2]);
            telemetry.addData("grabberPos", robot.grabber.getPosition());
            telemetry.addData("pointCounter", pointCounter);
            telemetry.update();
        }
    }

    //LIFT FUNCTIONS
    private void runLift(double highPower, double lowPower, boolean liftManualOp){
        double liftPos;
        liftPos = robot.lift.getCurrentPosition();
        //direction of rotation to lift target
        double liftRotDir = (Math.abs(liftPos - robot.lift.getTargetPosition()) / (liftPos - robot.lift.getTargetPosition()));
        //set power for lift in assisted mode
        if(robot.lift.getCurrentPosition() != robot.lift.getTargetPosition() &! liftManualOp) {
            if(liftRotDir == -1) {
                robot.lift.setPower(liftRotDir * lowPower);
            }
            else if ((robot.lift.getCurrentPosition() < -850) && (liftRotDir == 1)){
                robot.lift.setPower(liftRotDir * lowPower);
            }
            else {
                robot.lift.setPower(liftRotDir * highPower);
            }
        }
    }
    private void setLiftPos(double liftPos){
        robot.lift.setTargetPosition((int) (liftPos));
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetEncoders(){
        robot.encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
