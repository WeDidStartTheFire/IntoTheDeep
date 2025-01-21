package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Logic", group = "Test")
public class TeleOp_MainTestLogic extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        while (active()) {
            drivetrainLogic(false);
            wristMotorLogic();
            wristServoLogic();
            intakeServoLogic();
            horizontalLiftLogic();
            verticalLiftLogic();
            basketServoLogic();
            specimenServoLogic();

            if (gamepad2.y) {
                wristMotorTicksStopped = 5;
                wristMotorStopPos = 0;
                wristPos = 1;
                moveWristServo(newWristPos = 0.5);
                liftRunToPos = true;
                liftGoal = 0;
            }

            if (gamepad1.dpad_up && !isDpu && !wasDpu) isDpu = wasDpu = true;
            else if (gamepad1.dpad_up && isDpu && wasDpu) isDpu = false;
            else if (!gamepad1.dpad_up && wasDpu) wasDpu = false;
            else if (!gamepad1.dpad_down && isDpu) isDpu = wasDpu = false;

            if (gamepad1.dpad_down && !isDpd && !wasDpd) isDpd = wasDpd = true;
            else if (gamepad1.dpad_down && isDpd && wasDpd) isDpd = false;
            else if (!gamepad1.dpad_down && wasDpd) wasDpd = false;
            else if (!gamepad1.dpad_down && isDpd) isDpd = wasDpd = false;

            updateAll();
        }
    }
}
