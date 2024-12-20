package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main", group = "Into The Deep")
public class TeleOp_Main extends Base {

    double axial;
    double lateral;
    double yaw;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double max;

    double slowdownMultiplier;
    boolean touchSensorPressed = false;
    boolean touchSensorWasPressed = false;

    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    static final double WRIST_MOTOR_POWER = 0.1;
    static final int[] WRIST_MOTOR_BOUNDARIES = {0, 140};
    static final int[] LIFT_BOUNDARIES = {0, 1200};
    static final int[] V_LIFT_BOUNDARIES = {0, 10000}; // Temporary, need to change
    double intakeServoGoal = 0;
    double wristServoGoal = 0;
    double nextWristServoGoal = 0.5;
    double newNextWristServoGoal;

    int vertA, vertB, vertAvg;
    double power = 0;

    boolean wasIntakeServoButtonPressed = false;
    boolean wasWristServoButtonPressed = false;
    int wristMotorTicksStopped = 0;
    int wristMotorPosition = 0;
    int error;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        while (opModeIsActive() && !isStopRequested()) {
            slowdownMultiplier = 0.7;
            if (gamepad1.left_bumper) {
                slowdownMultiplier = 0.3;
            }
            if (gamepad1.right_bumper) {
                slowdownMultiplier = 1;
            }
            // Slows down movement for better handling the more the right trigger is held down
            // slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;

            axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER));
            lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER));
            yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED));

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            if (lf != null) {
                lf.setVelocity(leftFrontPower * 5000 * slowdownMultiplier);
                rf.setVelocity(rightFrontPower * 5000 * slowdownMultiplier);
                lb.setVelocity(leftBackPower * 5000 * slowdownMultiplier);
                rb.setVelocity(rightBackPower * 5000 * slowdownMultiplier);
            } else {
                print("WARNING:", "At least one drivetrain motor disconnected");
            }

            // Logic for the wrist motor
            if (wristMotor != null) {
                print("Wrist Motor position", wristMotor.getCurrentPosition());
                if (gamepad2.dpad_down
                        && wristMotor.getCurrentPosition() < WRIST_MOTOR_BOUNDARIES[1]) {
                    wristMotor.setPower(WRIST_MOTOR_POWER);
                    wristMotorTicksStopped = 0;
                    addLastActionTelemetry("Wrist Motor now moving");
                } else if (gamepad2.dpad_up
                        && (wristMotor.getCurrentPosition() > WRIST_MOTOR_BOUNDARIES[0]
                                || gamepad2.right_bumper)) {
                    wristMotor.setPower(-WRIST_MOTOR_POWER);
                    wristMotorTicksStopped = 0;
                    addLastActionTelemetry("Wrist Motor now moving");
                    if (gamepad2.right_bumper) {
                        WRIST_MOTOR_BOUNDARIES[1] +=
                                wristMotor.getCurrentPosition() - WRIST_MOTOR_BOUNDARIES[0];
                        WRIST_MOTOR_BOUNDARIES[0] = wristMotor.getCurrentPosition();
                        addLastActionTelemetry("Wrist Motor boundaries overridden");
                    }
                } else {
                    error = wristMotorPosition - wristMotor.getCurrentPosition();
                    if (wristMotorTicksStopped < 5) {
                        wristMotor.setPower(0);
                        wristMotorPosition = wristMotor.getCurrentPosition();
                    } else if (Math.abs(error) > 3) {
                        wristMotor.setPower(WRIST_MOTOR_POWER * error / 10.0);
                    } else {
                        wristMotor.setPower(0);
                    }
                    wristMotorTicksStopped++;
                }
            }

            // Logic for the wrist servo
            if (wristServo != null) {
                if (gamepad2.a && !wasWristServoButtonPressed) {
                    if (nextWristServoGoal == 0 || nextWristServoGoal == 1) {
                        newNextWristServoGoal = 0.5;
                    } else if (nextWristServoGoal == 0.5) {
                        newNextWristServoGoal = 1 - wristServoGoal;
                    }
                    wristServoGoal = nextWristServoGoal;
                    nextWristServoGoal = newNextWristServoGoal;
                    wristServo.setPosition(wristServoGoal);
                }
                wasWristServoButtonPressed = gamepad2.a;
                print("Wrist Servo Goal", wristServoGoal);
            }

            // Logic for the intake servo
            if (intakeServo != null) {
                if (gamepad2.b && !wasIntakeServoButtonPressed) {
                    if (intakeServoGoal == 0) {
                        intakeServoGoal = 1;
                        intakeServo.setPosition(intakeServoGoal);
                    } else {
                        intakeServoGoal = 0;
                        intakeServo.setPosition(intakeServoGoal);
                    }
                }
                wasIntakeServoButtonPressed = gamepad2.b;
                print("Intake Servo Goal", intakeServoGoal);
            }

            // Logic to extend or retract the horizontal lift
            if (liftMotor != null) {
                addLastActionTelemetry("Current lift position: " + liftMotor.getCurrentPosition());
                if (!gamepad1.dpad_right && !gamepad1.dpad_left
                        || gamepad1.dpad_left && gamepad1.dpad_right) {
                    liftMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        touchSensorPressed = touchSensor.isPressed();
                    } else {
                        touchSensorPressed = false;
                        addLastActionTelemetry("Touch sensor not connected");
                    }
                    if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                        if (liftMotor.getCurrentPosition() < LIFT_BOUNDARIES[1]) {
                            liftMotor.setPower(1 * slowdownMultiplier);
                            addLastActionTelemetry("Lift Motor now moving");
                        } else {
                            liftMotor.setPower(0);
                            addLastActionTelemetry("Lift Motor no longer moving");
                        }
                    } else if (gamepad1.dpad_left && !gamepad1.dpad_right && !touchSensorPressed) {
                        if (liftMotor.getCurrentPosition() > LIFT_BOUNDARIES[0]) {
                            liftMotor.setPower(-1 * slowdownMultiplier);
                            addLastActionTelemetry("Lift Motor now moving");
                        } else {
                            liftMotor.setPower(0);
                            addLastActionTelemetry("Lift Motor no longer moving");
                        }
                    }
                }
            }

            // Logic to raise or lower the vertical lift
            if (verticalMotorA != null) {
                vertA = verticalMotorA.getCurrentPosition();
                vertB = verticalMotorB.getCurrentPosition();
                vertAvg = (vertA + vertB) / 2;
//                addLastActionTelemetry("Current lift position: " + liftMotor.getCurrentPosition());
                if (!gamepad1.dpad_up && !gamepad1.dpad_down
                        || gamepad1.dpad_down && gamepad1.dpad_up) {
                    power = 0;
                } else {
                    if (touchSensor != null) {
                        touchSensorPressed = touchSensor.isPressed();
                    } else {
                        touchSensorPressed = false;
                        addLastActionTelemetry("Touch sensor not connected");
                    }
                    if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                        if (vertAvg < V_LIFT_BOUNDARIES[1]) {
                            power = 1;
                            addLastActionTelemetry("Vertical Motors now moving");
                        } else {
                            power = 0;
                            addLastActionTelemetry("Vertical Motors no longer moving");
                        }
                    } else if (gamepad1.dpad_down && !gamepad1.dpad_up && !touchSensorPressed) {
                        if (vertAvg > V_LIFT_BOUNDARIES[0]) {
                            power = -slowdownMultiplier;
                            addLastActionTelemetry("Vertical Motors now moving");
                        } else {
                            power = 0;
                            addLastActionTelemetry("Vertical Motors no longer moving");
                        }
                    }
                }
                verticalMotorA.setPower(power);
                verticalMotorB.setPower(power);
                print("Vertical Motor Power", power);
            }

            // Logic to stop lift when it hits touch sensor
            if (touchSensor != null && liftMotor != null) {
                if (!touchSensorWasPressed) {
                    if (touchSensor.isPressed()) {
                        liftMotor.setPower(0);
                        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        touchSensorWasPressed = true;
                    } else {
                        touchSensorWasPressed = false;
                    }
                } else if (!touchSensor.isPressed()) {
                    touchSensorWasPressed = false;
                }
            }

            print("Speed Multiplier", slowdownMultiplier);
            updateAll();
        }
    }
}
