package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test No Driving", group = "Into The Deep")
public class TeleOp_Debug_No_Driving extends Base {

    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    boolean wasDownA = false;
    boolean wasDownB = false;
    public Servo servoA, servoB, servoC, servoD;
    public DcMotorEx motorA, motorB;
    public final double MOTOR_SPEED = 0.25;
    public final boolean controlHub = true;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        if (controlHub) {
            try {
                servoA = hardwareMap.get(Servo.class, "servoA");
            } catch (Exception e) {
                except("servoA disconnected");
            }
            try {
                servoB = hardwareMap.get(Servo.class, "servoB");
            } catch (Exception e) {
                except("servoB disconnected");
            }
            try {
                servoC = hardwareMap.get(Servo.class, "servoC");
            } catch (Exception e) {
                except("servoC disconnected");
            }
            try {
                servoD = hardwareMap.get(Servo.class, "servoD");
            } catch (Exception e) {
                except("servoD disconnected");
            }
            motorA = lf;
            motorB = lb;
        } else {
            servoA = droneServo;
            servoB = specimenServo;
            servoC = wristServo;
            servoD = intakeServo;
            motorA = wristMotor;
            motorB = liftMotor;
        }

        while (active()) {
            if (servoA != null) {
                if (gamepad1.a && !wasDownA) {
                    if (servoA.getPosition() > 0.95) {
                        servoA.setPosition(0);
                        addLastActionTelemetry("Set servoA to 0");
                    } else {
                        servoA.setPosition(1);
                        addLastActionTelemetry("Set servoA to 1");
                    }
                }
                wasDownA = gamepad1.a;
            } else {
                addLastActionTelemetry("servoA disconnected");
            }

            if (servoB != null) {
                if (gamepad1.b && !wasDownB) {
                    if (servoB.getPosition() > 0.95) {
                        servoB.setPosition(0);
                        addLastActionTelemetry("Set servoB to 0");
                    } else {
                        servoB.setPosition(1);
                        addLastActionTelemetry("Set servoB to 1");
                    }
                }
                wasDownB = gamepad1.b;
            } else {
                addLastActionTelemetry("servoB diconnected");
            }

            if (servoC != null) {
                if (gamepad1.x) {
                    servoC.setPosition(1);
                    addLastActionTelemetry("Set servoC to 1");
                } else if (gamepad1.y) {
                    servoC.setPosition(0);
                    addLastActionTelemetry("Set servoC to 0");
                } else {
                    servoC.setPosition(0.5);
                    addLastActionTelemetry("Set servoC to 0.5");
                }
            } else {
                addLastActionTelemetry("servoC disconnected");
            }

            if (servoD != null) {
                if (gamepad1.left_bumper) {
                    servoD.setPosition(0);
                    addLastActionTelemetry("Set servoD to 0");
                } else if (gamepad1.right_bumper) {
                    servoD.setPosition(1);
                    addLastActionTelemetry("Set servoD to 1");
                } else {
                    servoD.setPosition(0.5);
                    addLastActionTelemetry("Set servoD to 0.5");
                }
            } else {
                addLastActionTelemetry("servoD disconnected");
            }

            if (motorA != null) {
                if (gamepad1.dpad_up) {
                    motorA.setPower(1 * MOTOR_SPEED);
                } else if (gamepad1.dpad_down) {
                    motorA.setPower(-1 * MOTOR_SPEED);
                } else {
                    motorA.setPower(0);
                }
            } else {
                addLastActionTelemetry("motorA disconnected");
            }

            if (motorB != null) {
                if (gamepad1.dpad_right) {
                    motorB.setPower(1 * MOTOR_SPEED);
                } else if (gamepad1.dpad_left) {
                    motorB.setPower(-1 * MOTOR_SPEED);
                } else {
                    motorB.setPower(0);
                }
            } else {
                addLastActionTelemetry("motorB disconnected");
            }

            updateAll();
        }
    }
}
