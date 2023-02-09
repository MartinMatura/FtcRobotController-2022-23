package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.Odometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Odometry Drive Mode", group="Linear Opmode")
public class OdoDriveMode extends LinearOpMode{
    HardwareRobot robot = new HardwareRobot();
    Odometry odometry = new Odometry();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        resetRight();
        resetLeft();
        resetSide();
        resetLift();

        robot.lift.setTargetPosition(0);

        List<double[]> path = new ArrayList<>(); //to log path created by dpad inputs
        int lastInput = 0;

        double[] currPos;
        double[] power;

        double targetRot = 0;

        boolean fieldRelative = true;

        while(opModeIsActive()) {

            double x = 0;
            double y = 0;
            currPos = getPos();

            if(gamepad2.x){
                robot.grabber.setPosition(1);
            }

            if(gamepad2.y){
                robot.grabber.setPosition(0.4);
            }

            if(gamepad2.right_bumper){
                robot.grabber.setPosition(0.5);
            }

            if(gamepad2.a){
                robot.spinner.setPosition(1);
            }

            if(gamepad2.b){
                robot.spinner.setPosition(0);
            }

            if(gamepad2.dpad_up){
                robot.lift.setTargetPosition(-1100);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_down){
                robot.lift.setTargetPosition(-350);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_left){
                robot.lift.setTargetPosition(-300);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_right){
                robot.lift.setTargetPosition(-700);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if(gamepad2.left_bumper){
                robot.lift.setTargetPosition(-200);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.right_bumper){
                fieldRelative = !fieldRelative;
            }

            double liftPos = robot.lift.getCurrentPosition();
            if(robot.lift.getCurrentPosition() != robot.lift.getTargetPosition()) {
                robot.lift.setPower((Math.abs(liftPos - robot.lift.getTargetPosition())/(liftPos - robot.lift.getTargetPosition())) * 0.5);
            }

            //update target angle
            targetRot = getTargetAngle();

            if(true){    //gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                if(fieldRelative){
                    x = gamepad1.left_stick_x;
                    y = gamepad1.left_stick_y;
                    power = calcPowerFR(x, y, currPos);
                } else{
                    x = gamepad1.left_stick_y;
                    y = -gamepad1.left_stick_x;
                    power = calcPowerRR(x, y);
                }
                double rotDif = targetRot - currPos[2];
                driveTA(power[0], power[1], rotDif);
                //drive(power[0], power[1]);
                currPos = getPos();
            }

            if(gamepad1.dpad_up){
                path = dpadUp(path, currPos, lastInput);
                lastInput = 1;
            }
            if(gamepad1.dpad_down){
                path = dpadDown(path, currPos, lastInput);
                lastInput = 2;
            }
            if(gamepad1.dpad_right){
                path = dpadRight(path, currPos, lastInput);
                lastInput = 3;
            }
            if(gamepad1.dpad_left){
                path = dpadLeft(path, currPos, lastInput);
                lastInput = 4;
            }

            if(!path.isEmpty()){ //drive to target points based on dpad input
                if(path.get(0)[1] > currPos[1] + 30){
                    double startY = currPos[1];
                    double endY = path.get(0)[1];
                    while(currPos[1]+20 < path.get(0)[1]){
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("position", Arrays.toString(currPos));
                        telemetry.addData("start",startY);
                        telemetry.addData("end",endY);
                        telemetry.update();
                        y = -1;
                        power = calcPowerFR(x, y, currPos);
                        drive(power[0], power[1]);
                        currPos = getPos();

                        if(gamepad1.dpad_up){//listen for other dpad inputs while running the while loop
                            path = dpadUp(path, currPos, lastInput);
                            lastInput = 1;
                        }
                        if(gamepad1.dpad_down){
                            path = dpadDown(path, currPos, lastInput);
                            lastInput = 2;
                        }
                        if(gamepad1.dpad_right){
                            path = dpadRight(path, currPos, lastInput);
                            lastInput = 3;
                        }
                        if(gamepad1.dpad_left){
                            path = dpadLeft(path, currPos, lastInput);
                            lastInput = 4;
                        }
                    }
                    path.remove(0);
                } else if(path.get(0)[1] < currPos[1] - 30){
                    double startY = currPos[1];
                    double endY = path.get(0)[1];
                    while(currPos[1]-20 > path.get(0)[1]){
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("position", Arrays.toString(currPos));
                        telemetry.addData("start",startY);
                        telemetry.addData("end",endY);
                        telemetry.update();
                        y = 1;
                        power = calcPowerFR(x, y, currPos);
                        drive(power[0], power[1]);
                        currPos = getPos();

                        if(gamepad1.dpad_up){
                            path = dpadUp(path, currPos, lastInput);
                            lastInput = 1;
                        }
                        if(gamepad1.dpad_down){
                            path = dpadDown(path, currPos, lastInput);
                            lastInput = 2;
                        }
                        if(gamepad1.dpad_right){
                            path = dpadRight(path, currPos, lastInput);
                            lastInput = 3;
                        }
                        if(gamepad1.dpad_left){
                            path = dpadLeft(path, currPos, lastInput);
                            lastInput = 4;
                        }
                    }
                    path.remove(0);
                } else if(path.get(0)[0] > currPos[0] + 30){
                    double startX = currPos[0];
                    double endX = path.get(0)[0];
                    while(currPos[0]+20 < path.get(0)[0]){
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("position", Arrays.toString(currPos));
                        telemetry.addData("start",startX);
                        telemetry.addData("end",endX);
                        telemetry.update();
                        x = 1;
                        power = calcPowerFR(x, y, currPos);
                        drive(power[0], power[1]);
                        currPos = getPos();

                        if(gamepad1.dpad_up){
                            path = dpadUp(path, currPos, lastInput);
                            lastInput = 1;
                        }
                        if(gamepad1.dpad_down){
                            path = dpadDown(path, currPos, lastInput);
                            lastInput = 2;
                        }
                        if(gamepad1.dpad_right){
                            path = dpadRight(path, currPos, lastInput);
                            lastInput = 3;
                        }
                        if(gamepad1.dpad_left){
                            path = dpadLeft(path, currPos, lastInput);
                            lastInput = 4;
                        }
                    }
                    path.remove(0);
                } else if(path.get(0)[0] < currPos[0] - 30){
                    double startX = currPos[0];
                    double endX = path.get(0)[0];
                    while(currPos[0]-20 > path.get(0)[0]){
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("position", Arrays.toString(currPos));
                        telemetry.addData("start",startX);
                        telemetry.addData("end",endX);
                        telemetry.update();
                        x = -1;
                        power = calcPowerFR(x, y, currPos);
                        drive(power[0], power[1]);
                        currPos = getPos();

                        if(gamepad1.dpad_up){
                            path = dpadUp(path, currPos, lastInput);
                            lastInput = 1;
                        }
                        if(gamepad1.dpad_down){
                            path = dpadDown(path, currPos, lastInput);
                            lastInput = 2;
                        }
                        if(gamepad1.dpad_right){
                            path = dpadRight(path, currPos, lastInput);
                            lastInput = 3;
                        }
                        if(gamepad1.dpad_left){
                            path = dpadLeft(path, currPos, lastInput);
                            lastInput = 4;
                        }
                    }
                    path.remove(0);
                }
            }

            if (gamepad1.a){
                robot.leftFront.setPower(0);
                robot.rightBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                break;
            }
        }
    }

    private void resetRight(){ //reset encoders to 0 before start
        robot.encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetLeft(){
        robot.encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetSide(){
        robot.encS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetLift() {
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double[] calcPowerFR(double x, double y, double[] currPos){ //field relative

        double speedCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);

        double rotX = x * Math.cos(currPos[2]) - y * Math.sin(-currPos[2]);
        double rotY = y * Math.cos(currPos[2]) + x * Math.sin(-currPos[2]);

        telemetry.addData("rot",currPos[2]);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("position", Arrays.toString(currPos));
        telemetry.update();

        double lFrBpower = Range.clip((rotY - rotX) * speedCoe, -1.0, 1.0);
        double lBrFpower = Range.clip((rotY + rotX) * speedCoe, -1.0, 1.0);

        double[] power = {lFrBpower, lBrFpower};

        return power;
    }

    private double[] calcPowerRR(double x, double y){ //robot relative
        double speedCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();

        double lFrBpower = Range.clip((y - x) * speedCoe, -1.0, 1.0);
        double lBrFpower = Range.clip((y + x) * speedCoe, -1.0, 1.0);

        return new double[]{lFrBpower, lBrFpower};
    }

    private void drive(double lFrB, double lBrF){//give power to motors
        double turnCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);
        double turn = gamepad1.right_stick_x * turnCoe;

        robot.leftFront.setPower(Range.clip(lFrB - turn, -1.0, 1.0));
        robot.rightBack.setPower(Range.clip(lFrB + turn, -1.0, 1.0));

        robot.rightFront.setPower(Range.clip(lBrF + turn, -1.0, 1.0));
        robot.leftBack.setPower(Range.clip(lBrF - turn, -1.0, 1.0));
    }

    private void driveTA(double lFrB, double lBrF, double rotDif){ //drive function with targetAngle Adjustment

        //use difference between target angle and odometry read angle to adjust turn
        double turn = Range.clip(rotDif, -1.0, 1.0);

        robot.leftFront.setPower(Range.clip(lFrB - turn, -1.0, 1.0));
        robot.rightBack.setPower(Range.clip(lFrB + turn, -1.0, 1.0));

        robot.rightFront.setPower(Range.clip(lBrF + turn, -1.0, 1.0));
        robot.leftBack.setPower(Range.clip(lBrF - turn, -1.0, 1.0));
    }

    private double getTargetAngle(){ //update target angle with right stick input
        double turnCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);
        return gamepad1.right_stick_x * turnCoe;
    }

    private double[] getPos(){ //update position using odometry
        double     countsPerMotorRev   = 8192 ;    // eg: TETRIX Motor Encoder
        double     wheelDiameterCm     = 6.0 ;     // For figuring circumference
        double     countsPerCm         = (countsPerMotorRev / (wheelDiameterCm * Math.PI));

        double[] currPos = {0,0,0};

        double rPrevPos;
        double rCurrPos = 0;

        double lPrevPos;
        double lCurrPos = 0;

        double sPrevPos;
        double sCurrPos = 0;

        rPrevPos = rCurrPos;
        rCurrPos = robot.encR.getCurrentPosition();
        double dR = (rCurrPos - rPrevPos)/countsPerCm;

        lPrevPos = lCurrPos;
        lCurrPos = robot.encL.getCurrentPosition();
        double dL = (lCurrPos - lPrevPos)/countsPerCm;

        sPrevPos = sCurrPos;
        sCurrPos = robot.encS.getCurrentPosition();
        double dS = (sCurrPos - sPrevPos)/countsPerCm;

        currPos = odometry.nowPos(currPos, dR, dL, dS);

        return currPos;
    }

    private List<double[]> dpadUp(List<double[]> path, double[] currPos, int lastInput){ //add new target point to path based on dpad input
        double startY = currPos[1];
        double endY = startY - startY % 61 + 61;
        if(path.size() == 0){
            path.add(new double[] {currPos[0],endY});
        } else if(lastInput==1){
            path.set(path.size()-1,new double[] {path.get(path.size()-1)[0],path.get(path.size()-1)[1]+61});
        } else{
            path.add(new double[] {path.get(path.size()-1)[0],path.get(path.size()-1)[1]+61});
        }
        return path;
    }

    private List<double[]> dpadDown(List<double[]> path, double[] currPos, int lastInput){
        double startY = currPos[1];
        double endY = startY - startY%61;
        if(path.size() == 0){
            path.add(new double[] {currPos[0],endY});
        } else if(lastInput==2){
            path.set(path.size()-1,new double[] {path.get(path.size()-1)[0],path.get(path.size()-1)[1]-61});
        } else{
            path.add(new double[] {path.get(path.size()-1)[0],path.get(path.size()-1)[1]-61});
        }
        return path;
    }

    private List<double[]> dpadRight(List<double[]> path, double[] currPos, int lastInput){
        double startX = currPos[0];
        double endX = startX - startX%61 + 61;
        if(path.size() == 0){
            path.add(new double[] {endX, currPos[1]});
        } else if(lastInput==3){
            path.set(path.size()-1,new double[] {path.get(path.size()-1)[0]+61,path.get(path.size()-1)[1]});
        } else{
            path.add(new double[] {path.get(path.size()-1)[0]+61,path.get(path.size()-1)[1]});
        }
        return path;
    }

    private List<double[]> dpadLeft(List<double[]> path, double[] currPos, int lastInput){
        double startX = currPos[0];
        double endX = startX - startX%61;
        if(path.size() == 0){
            path.add(new double[] {endX, currPos[1]});
        } else if(lastInput==4){
            path.set(path.size()-1,new double[] {path.get(path.size()-1)[0]-61,path.get(path.size()-1)[1]});
        } else{
            path.add(new double[] {path.get(path.size()-1)[0]-61,path.get(path.size()-1)[1]});
        }
        return path;
    }
}



