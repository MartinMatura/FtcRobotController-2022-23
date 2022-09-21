package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@TeleOp(name="Basic Drive Mode", group="Linear Opmode")
public class BasicDriveMode extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            double speedCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1.0);

            //needs to be changed to reflect new sideways drive -> left is forward, forward is right, right is back, back is left
            double iPower = Range.clip((y - x) * speedCoe, -1.0, 1.0);
            double kPower = Range.clip((y + x) * speedCoe, -1.0, 1.0)*-1;

            //i
            robot.leftFront.setPower(iPower);
            robot.rightBack.setPower(iPower);
            //k
            robot.rightFront.setPower(kPower);
            robot.leftBack.setPower(kPower);
        }
    }
}
