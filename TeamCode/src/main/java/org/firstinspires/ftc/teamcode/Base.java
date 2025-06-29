package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localizers.RedundantLocalizer;
import org.firstinspires.ftc.teamcode.other.MecanumDrive;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import static java.util.Locale.US;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

// Connect to robot: rc

/** Base class that contains common methods and other configuration. */
public abstract class Base extends LinearOpMode {
    private static final double LIFT_VEL = 1500;
    private final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, liftMotor, wristMotor, verticalMotorA, verticalMotorB;
    public Servo wristServoX, wristServoY, basketServo, specimenServo, intakeServo;
    public TouchSensor verticalTouchSensor, horizontalTouchSensor;
    public IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    */
    public String hubName;
    public static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;

    public static final double TAU = 2 * PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    static final double STRAFE_FRONT_MODIFIER = 1.3;
    static final double B = 1.1375;
    static final double M = 0.889;
    static final double TURN_SPEED = 0.5;
    static final int[] LIFT_BOUNDARIES = {0, 1625};
    static final int[] V_LIFT_BOUNDS = {0, 1950};
    static final int[] V_LIFT_GOALS = {0, 280, 500, 1350, 1950};
    static final double[] WRIST_S_GOALS = {.1, .3, .5, .9};
    int wristIndex = 0;

    public boolean useOdometry = true, useCam = true;
    static final double DEFAULT_VELOCITY = 2000;
    double velocity = DEFAULT_VELOCITY;
    public VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public MecanumDrive drive;
    public Pose2d currentPose = new Pose2d(0, 0, 0);

    public volatile boolean loop = false;
    public volatile boolean running = true;
    public volatile boolean hold = false;
    public volatile boolean holdWrist = false;

    public boolean auto = false;

    /** Dimension front to back on robot in inches */
    public static final double ROBOT_LENGTH = 18;
    /** Dimension left to right on robot in inches */
    public static final double ROBOT_WIDTH = 18;

    double goalAngle = 0;


    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


    static final double WRIST_MOTOR_POWER = 0.1;
    static final int[] WRIST_M_BOUNDS = {0, 80};
    int vertGoal;
    boolean vertRunToPos, vertStopped;
    double verticalLiftTimer = 0;
    double wristServoTimer = 0;

    boolean wasIntakeServoButtonPressed, wasWristServoButtonPressed;
    boolean wasSpecimenServoButtonPressed, wasBasketServoButtonPressed;
    int wristMotorTicksStopped = 0, wristMotorStopPos = 0, ticksPowered = 0;

    static double baseSpeedMultiplier = 0.75;
    static double baseTurnSpeed = 2.5;

    static double[] speeds = {0.2, 0.6, 1};
    boolean wasDpu, isDpu, isDpd, wasDpd;

    int liftGoal = 0;
    boolean liftRunToPos, handoff;

    Pose2d NET_ZONE_POSITION = new Pose2d(72 - 12 - (ROBOT_LENGTH / 2 / sqrt(2)) + 1, 72 - 12 - (ROBOT_LENGTH / 2 / sqrt(2)) + 1, toRadians(45));
    Pose2d OBSERVATION_ZONE_POSITION = new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, toRadians(0));
    boolean following = false;

    double[] WRIST_S_ANGLES = {toRadians(-180), toRadians(180)};

    /** Directions. Options: LEFT, RIGHT, FORWARD, BACKWARD */
    public enum Dir {
        LEFT, RIGHT, FORWARD, BACKWARD
    }

    /** Initializes all hardware devices on the robot. */
    public void setup() {
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMS)) throw new RuntimeException("IMU initialization failed");
        imu.resetYaw();

        // The following try catch statements "check" if a motor is connected. If it isn't, it sets
        // that motor's value to null. Later, we check if that value is null. If it is, we don't
        // run the motor.
        // Drive train
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront"); // Port 1
            lb = hardwareMap.get(DcMotorEx.class, "leftBack"); // Port 3
            rf = hardwareMap.get(DcMotorEx.class, "rightFront"); // Port 0
            rb = hardwareMap.get(DcMotorEx.class, "rightBack"); // Port 4
        } catch (IllegalArgumentException e) {
            except("At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        // Motors
        try {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("liftMotor (previously used as carWashMotor) not connected");
        }
        try {
            verticalMotorA = hardwareMap.get(DcMotorEx.class, "verticalMotorA"); // Expansion Hub 2
            verticalMotorB = hardwareMap.get(DcMotorEx.class, "verticalMotorB"); // Expansion Hub 3
        } catch (IllegalArgumentException e) {
            verticalMotorA = verticalMotorB = null;
            except(">= 1 verticalMotor connected; All vertical lift motors disabled");
        }
        try {
            wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("wristMotor not connected");
        }

        // Servos
        try {
            wristServoX = hardwareMap.get(Servo.class, "wristServoX"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("wristServoX not connected");
        }
        try {
            wristServoY = hardwareMap.get(Servo.class, "wristServoY"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            except("wristServoY not connected");
        }
        try {
            intakeServo = hardwareMap.get(Servo.class, "intakeServo"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("intakeServo not connected");
        }
        try {
            specimenServo = hardwareMap.get(Servo.class, "specimenServo"); // Control Hub 2
        } catch (IllegalArgumentException e) {
            except("specimenServo not connected");
        }
        try {
            basketServo = hardwareMap.get(Servo.class, "basketServo"); // Control Hub 3
        } catch (IllegalArgumentException e) {
            except("basketServo not connected");
        }

        // Touch Sensors
        try {
            verticalTouchSensor = hardwareMap.get(TouchSensor.class, "verticalTouchSensor"); // Control Hub 0/1
        } catch (IllegalArgumentException e) {
            except("verticalTouchSensor not connected");
        }
        try {
            horizontalTouchSensor = hardwareMap.get(TouchSensor.class, "horizontalTouchSensor"); // Control Hub 2/3
        } catch (IllegalArgumentException e) {
            except("horizontalTouchSensor not connected");
        }

        if (useCam) {
            try {
                WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors(cam);
            } catch (IllegalArgumentException e) {
                except("Webcam not connected");
            }
        }
        try {
            drive = new MecanumDrive(hardwareMap, currentPose);
            drive.breakFollowing();
        } catch (IllegalArgumentException e) {
            except("SparkFun Sensor not connected");
            useOdometry = false;
        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (auto) {
                lf.setZeroPowerBehavior(BRAKE);
                lb.setZeroPowerBehavior(BRAKE);
                rf.setZeroPowerBehavior(BRAKE);
                rb.setZeroPowerBehavior(BRAKE);
            } else {
                lf.setZeroPowerBehavior(FLOAT);
                lb.setZeroPowerBehavior(FLOAT);
                rf.setZeroPowerBehavior(FLOAT);
                rb.setZeroPowerBehavior(FLOAT);
            }

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (liftMotor != null) {
            liftMotor.setMode(STOP_AND_RESET_ENCODER);
            liftMotor.setMode(RUN_WITHOUT_ENCODER);
            liftMotor.setZeroPowerBehavior(BRAKE);
        }

        if (verticalMotorA != null) {
            verticalMotorB.setDirection(REVERSE);

            verticalMotorA.setZeroPowerBehavior(BRAKE);
            verticalMotorB.setZeroPowerBehavior(BRAKE);

            verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(STOP_AND_RESET_ENCODER);

            verticalMotorA.setTargetPosition(verticalMotorA.getCurrentPosition());
            verticalMotorB.setTargetPosition(verticalMotorB.getCurrentPosition());

            verticalMotorA.setMode(RUN_USING_ENCODER);
            verticalMotorB.setMode(RUN_USING_ENCODER);
        }

        if (wristMotor != null) {
            wristMotor.setDirection(REVERSE);
            wristMotor.setZeroPowerBehavior(BRAKE);
            wristMotor.setMode(STOP_AND_RESET_ENCODER);
            wristMotor.setMode(RUN_USING_ENCODER);
        }

        print("Status", "Initialized");
        print("Hub Name", hubName);
        update();

        while (!isStarted() && !isStopRequested()) {
            useOdometry = ((useOdometry || gamepad1.b) && !gamepad1.a);
            print("useOdometry", useOdometry);
            print("Status", "Initialized");
            print("Hub Name", hubName);
            print(":::", ":::");
            print(":::", ":::");
            updateAll();
        }
        runtime.reset();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param startPose Starting position of the robot
     */
    public void setup(Pose2d startPose) {
        currentPose = startPose;
        setup();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param useOdom Whether to use odometry.
     * @deprecated
     */
    @Deprecated
    public void setup(boolean useOdom) {
        useOdometry = useOdom;
        setup();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param useCamera Whether to use the camera.
     * @param useOdom   Whether to use odometry.
     * @deprecated
     */
    @Deprecated
    public void setup(boolean useCamera, boolean useOdom) {
        useOdometry = useOdom;
        useCam = useCamera;
        setup();
    }

    /**
     * Drives using encoder velocity. An inches value of zero will cause the robot to drive until
     * manually stopped.
     *
     * @param inches    Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.*
     */
    private void velocityDrive(double inches, Dir direction) {
        if (isStopRequested() || !opModeIsActive() || lf == null) return;

        int lfTarget = 0;
        int rfTarget = 0;
        int dir = direction == FORWARD ? 1 : direction == BACKWARD ? -1 : 0;

        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorModes(RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        if (inches != 0) {
            runtime.reset();
            setMotorVelocities(velocity * signum(inches) * dir);
            inches = signum(inches) * (abs(inches) + B) / M;
        } else setMotorVelocities(velocity * dir);

        double duration = abs(inches * COUNTS_PER_INCH / velocity);

        while (active() && (runtime.seconds() < duration) && inches != 0) {
            // Display it for the driver.
            print("Angle", imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
            print("Running to", " " + lfTarget + ":" + rfTarget);
            print("Currently at", lf.getCurrentPosition() + ":" + rf.getCurrentPosition());
            if (!loop) update();
        }
        if (inches != 0) stopRobot();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void IMUTurn(double degrees, Dir direction) {
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
        sleep(100);
        degrees *= -1;
        degrees -= imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        imu.resetYaw();
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double angle, turnModifier, turnPower, initialGoalAngle;
        double correctedGoalAngle = initialGoalAngle = startAngle + degrees;
        double difference = 999;
        if (abs(initialGoalAngle) > 180)
            correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
        while (active() && (difference > tolerance) && degrees != 0) {
            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            difference = simplifyAngle(min(abs(initialGoalAngle - angle), abs(correctedGoalAngle - angle)));
            turnModifier = min(1, (difference + 3) / 30);
            turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier * direct;
            setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);

            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            difference = min(abs(initialGoalAngle - angle), abs(correctedGoalAngle - angle));

            print("Corrected Goal", correctedGoalAngle);
            print("Initial Goal", initialGoalAngle);
            print("Start", startAngle);
            print("Angle", angle);
            print("Distance from goal", difference);
            if (!loop) update();
        }
        stopRobot();
        imu.resetYaw();
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction Direction to turn if degrees is zero.
     */
    public void newIMUTurn(double degrees, Dir direction) {
        double direct = direction == LEFT ? -1 : direction == RIGHT ? 1 : 0;
        goalAngle = simplifyAngle(goalAngle - degrees);
        degrees = simplifyAngle(-degrees - imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle);
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        double angle, turnModifier, turnPower;
        double error = 999;
        while (active() && error > tolerance) {
            angle = imu.getRobotOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
            error = abs(angleDifference(angle, goalAngle));
            turnModifier = min(1, (error + 3) / 30);
            turnPower = TURN_SPEED * turnModifier * direct * signum(degrees);
            setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);

            print("Start Angle", startAngle);
            print("Current Angle", angle);
            print("Goal Angle", goalAngle);
            print("Error", error);
            if (!loop) update();
        }
        stopRobot();
    }

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees
     *
     * @param angle Angle in degrees to be simplified
     * @return Angle now simplified between -180 and 180 degrees
     */
    public double simplifyAngle(double angle) {
        return simplifyAngle(angle, DEGREES);
    }

    /**
     * Simplifies an angle in degrees to be between -180 and 180 degrees or -pi and pi radians
     *
     * @param angle Angle
     * @param u     Whether the angle is in radians or degrees
     */
    public double simplifyAngle(double angle, AngleUnit u) {
        if (u == DEGREES) return ((angle + 180) % 360 + 360) % 360 - 180;
        return ((angle + PI) % TAU + TAU) % TAU - PI;
    }

    /**
     * Returns the difference between two angles in degrees
     *
     * @param a First angle in degrees
     * @param b Second angle, to subtract, in degrees
     * @return The difference between the two angles in degrees
     */
    public double angleDifference(double a, double b) {
        return angleDifference(a, b, DEGREES);
    }

    /**
     * Returns the difference between two angles in degrees or radians
     *
     * @param a First angle
     * @param b Second angle to subtract
     * @param u Whether the angles are in radians or degrees
     * @return The difference between the two angles in the specified unit
     */
    public double angleDifference(double a, double b, AngleUnit u) {
        return simplifyAngle(a - b, u);
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left.
     *
     * @param degrees   The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void turn(double degrees, Dir direction) {
        if (useOdometry) {
            int dir = direction == LEFT ? -1 : 1;
            drive.turn(Math.toRadians(degrees * dir));
            currentPose = new Pose2d(currentPose.position.x, currentPose.position.y, currentPose.heading.real + Math.toRadians(degrees * dir));
        } else IMUTurn(degrees, direction);
    }

    /**
     * Turns the robot a specified number of degrees. Positive values turn right, negative values
     * turn left. A degrees value of zero will cause the robot to turn until manually stopped.
     *
     * @param degrees The amount of degrees to turn.
     */
    public void turn(double degrees) {
        turn(degrees, RIGHT);
    }

    public void lineTo(Pose2d position, boolean reverse, boolean double_reverse) {
        if (!useOdometry) return;
        Pose2d delta_pos = position.minusExp(currentPose);  // position - currentPose?
        turn(simplifyAngle(toDegrees(toRadians(reverse ^ double_reverse ? 180 : 0) + atan2(delta_pos.position.y, delta_pos.position.x) + currentPose.heading.real)));
        drive(hypot(delta_pos.position.x, delta_pos.position.y), reverse ? BACKWARD : FORWARD);
        turn(simplifyAngle(toDegrees(position.heading.real - currentPose.heading.real)));
    }

    public void lineTo(Pose2d position, boolean reverse) {
        lineTo(position, reverse, false);
    }

    public void lineTo(Pose2d position) {
        lineTo(position, false, false);
    }

    /**
     * Sets the mode of all drive train motors to the same mode.
     *
     * @param mode The mode to set the motors to.
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        if (lb == null) return;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    /**
     * Sets the power of all drive train motors individually.
     *
     * @param lbPower Left back motor power.
     * @param rbPower Right back motor power.
     * @param lfPower Left front motor power.
     * @param rfPower Right front motor power.
     */
    public void setMotorPowers(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb == null) return;
        lb.setPower(lbPower);
        rb.setPower(rbPower);
        lf.setPower(lfPower);
        rf.setPower(rfPower);
    }

    /**
     * Sets the velocity of all drive train motors to the same value.
     *
     * @param velocity Velocity of the motors.
     */
    public void setMotorVelocities(double velocity) {
        if (lb == null) return;
        lf.setVelocity(velocity);
        lb.setVelocity(velocity);
        rf.setVelocity(velocity);
        rb.setVelocity(velocity);
    }

    public void setMotorVelocities(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb == null) return;
        lf.setVelocity(lfPower);
        lb.setVelocity(lbPower);
        rf.setVelocity(rfPower);
        rb.setVelocity(rbPower);
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void velocityStrafe(double inches, Dir direction) {
        if (!active() || lf == null) return;
        setMotorModes(STOP_AND_RESET_ENCODER);

        setMotorModes(RUN_USING_ENCODER);

        double dir = direction == RIGHT ? -1 : direction == LEFT ? 1 : 0;

        runtime.reset();
        lb.setVelocity(velocity * dir);
        rb.setVelocity(-velocity * dir);
        lf.setVelocity(-velocity * STRAFE_FRONT_MODIFIER * dir);
        rf.setVelocity(velocity * STRAFE_FRONT_MODIFIER * dir);

        if (inches != 0) inches = (abs(inches) + 1.0125) / 0.7155; // Linear regression

        double duration = abs(inches * COUNTS_PER_INCH / velocity);

        runtime.reset();
        while (active() && (runtime.seconds() < duration) && inches != 0) {
            print("Strafing until", duration + " seconds");
            print("Currently at", runtime.seconds() + " seconds");
            if (!loop) update();
        }
        if (inches != 0) stopRobot();
        print("Strafing", "Complete");
        if (!loop) update();
    }

    /**
     * Strafes left or right for a specified number of inches. An inches value of zero will cause
     * the robot to strafe until manually stopped.
     *
     * @param inches    Amount of inches to strafe.
     * @param direction Direction to strafe in.*
     */
    public void strafe(double inches, Dir direction) {
        if (useOdometry) {
            Trajectory strafeTraj;
            if (direction == LEFT)
                strafeTraj = drive.trajectoryBuilder(currentPose).strafeLeft(inches).build();
            else strafeTraj = drive.trajectoryBuilder(currentPose).strafeRight(inches).build();

            drive.followTrajectory(strafeTraj);
            currentPose = strafeTraj.end();
            return; // Early return
        }
        velocityStrafe(inches, direction);
    }

    /**
     * Strafes right for a specified number of inches. An inches value of zero will cause the robot
     * to strafe until manually stopped.
     *
     * @param inches Amount of inches to strafe.
     */
    public void strafe(double inches) {
        strafe(inches, RIGHT);
    }

    /**
     * Changes the velocity.
     *
     * @param vel New velocity value.
     */
    public void setVelocity(double vel) {
        velocity = vel;
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches    Amount of inches to drive.
     * @param direction Direction to drive in.*
     */
    public void drive(double inches, Dir direction) {
        if (useOdometry && active()) {
            Trajectory driveTrajectory;
            if (direction == FORWARD)
                driveTrajectory = drive.trajectoryBuilder(currentPose).forward(inches).build();
            else driveTrajectory = drive.trajectoryBuilder(currentPose).back(inches).build();

            drive.followTrajectory(driveTrajectory);
            currentPose = driveTrajectory.end();
            return; // Early return
        }

        int checks = 1;
        if (inches == 0) {
            velocityDrive(0, direction);
            return;
        }

        for (int i = 0; i < checks; i++) velocityDrive(inches / checks, direction);
        stopRobot();
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches Amount of inches to drive.
     */
    public void drive(double inches) {
        drive(inches, FORWARD);
    }

    /**
     * Drives the specified number of inches. Negative values will drive backwards.
     *
     * @param inches    Amount of inches to drive.
     * @param direction Direction to drive in.*
     */
    public void drive(Dir direction, double inches) {
        drive(inches, direction);
    }

    /** Stops all drive train motors on the robot. */
    public void stopRobot() {
        if (lb == null) return;
        setMotorPowers(0, 0, 0, 0);
        lb.setVelocity(0);
        rb.setVelocity(0);
        lf.setVelocity(0);
        rf.setVelocity(0);

        // Set target position to avoid an error
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        setMotorModes(RUN_TO_POSITION);

        // Turn off RUN_TO_POSITION
        setMotorModes(RUN_USING_ENCODER);
    }

    /**
     * Moves the wrist servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveWristServoX(double position) {
        if (wristServoX != null) wristServoX.setPosition(position);
    }

    /**
     * Moves the wrist servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveWristServoY(double position) {
        if (wristServoY != null) wristServoY.setPosition(position);
    }

    /** Retracts the wrist Y servo */
    public void retractWristServoY() {
        moveWristServoY(0);
    }

    /** Hovers the wrist Y servo */
    public void hoverWristServoY() {
        moveWristServoY(0.9);
    }

    /** Extends the wrist Y servo */
    public void extendWristServoY() {
        moveWristServoY(1);
    }

    /**
     * Moves the intake servo to the specified position.
     *
     * @param position The position to move the intake servo to.
     */
    public void moveIntake(double position) {
        if (intakeServo != null) intakeServo.setPosition(position);
    }

    /** Opens the intake servo. */
    public void openIntake() {
        moveIntake(1);
    }

    /** Closes the intake servo. */
    public void closeIntake() {
        moveIntake(0);
    }

    /**
     * Gets the position of the intake servo.
     *
     * @return The position of the intake servo when connected, else -1;
     */
    public double getIntakePosition() {
        return intakeServo != null ? intakeServo.getPosition() : -1;
    }

    /**
     * Moves the specimen servo to the specified position.
     *
     * @param position Position between 0 and 1 to move the servo to
     */
    public void moveSpecimenServo(double position) {
        if (specimenServo != null) specimenServo.setPosition(position);
    }

    /** Opens the specimen servo. */
    public void openSpecimenServo() {
        moveSpecimenServo(1);
    }

    /** Closes the specimen servo. */
    public void closeSpecimenServo() {
        moveSpecimenServo(0);
    }

    /**
     * Gets the position of the specimen servo.
     *
     * @return The position of the specimen servo when connected, else -1;
     */
    public double getSpecimenPosition() {
        return specimenServo != null ? specimenServo.getPosition() : -1;
    }

    /**
     * Moves the basket servo to the specified position.
     *
     * @param position The position to move the basket servo to.
     */
    public void moveBasketServo(double position) {
        if (basketServo != null) basketServo.setPosition(position);
    }

    /** Extends the basket servo outside of the robot */
    public void extendBasketServo() {
        moveBasketServo(0);
    }

    /** Returns the basket servo to the robot */
    public void returnBasketServo() {
        moveBasketServo(1);
    }

    /**
     * Gets the position of the basket servo.
     *
     * @return The position of the basket servo when connected, else -1;
     */
    public double getBasketPosition() {
        return basketServo != null ? basketServo.getPosition() : -1;
    }

    /**
     * Moves the wrist to the specified encoder value.
     *
     * @param encoderValue The encoder value to move the wrist to.
     */
    public void moveWrist(int encoderValue) {
        if (wristMotor == null) return;
        int pos = getWristPos();
        int error = pos - encoderValue;
        wristMotor.setMode(RUN_WITHOUT_ENCODER);
        double t0 = getRuntime();
        while ((abs(error) > 3 && !(getRuntime() - t0 >= 1 && pos == 0)) && active()) {
            if (error > 0) wristMotor.setPower(max(-0.7, -error / 10.0));
            else wristMotor.setPower(min(0.7, -error / 10.0));
            pos = getWristPos();
            error = pos - encoderValue;
        }
        wristMotor.setPower(0);
    }

    public void wristOutOfWay() {
        moveWrist(30);
    }

    /** Extends the wrist. */
    public void extendWrist() {
        moveWrist(50);
    }

    /** Holds the wrist */
    public void holdWrist(int holdPos) {
        if (wristMotor == null) return;
        holdWrist = true;
        while (holdWrist && active()) {
            int wristMotorPos = wristMotor.getCurrentPosition();
            int error = holdPos - wristMotorPos;
            wristMotor.setPower((abs(error) > 3 ? WRIST_MOTOR_POWER * error / 10.0 : 0) + (wristMotorPos > 30 ? 0.02 : 0));
        }
    }

    public void holdWristOutOfWay() {
        holdWrist(30);
    }

    /** Retracts the wrist. */
    public void retractWrist() {
        moveWrist(0);
    }

    /**
     * Returns the current position of the wrist.
     *
     * @return The current position of the wrist motor.
     */
    public int getWristPos() {
        return wristMotor != null ? wristMotor.getCurrentPosition() : 0;
    }

    /**
     * Returns information about a tag with the specified ID if it is currently detected.
     *
     * @param id ID of tag to detect.
     * @return Information about the tag detected.
     */
    @Nullable
    public AprilTagDetection detectTag(int id) {
        ArrayList<AprilTagDetection> t = tagProcessor.getDetections();
        for (int i = 0; i < t.size(); i++) if (t.get(i).id == id) return t.get(i);
        return null;
    }

    /**
     * Returns information about a tag with the specified ID if it is detected within a designated
     * timeout period.
     *
     * @param id      ID of tag to detect.
     * @param timeout Detection timeout (seconds).
     * @return Information about the tag detected.
     */
    @Nullable
    public AprilTagDetection detectTag(int id, double timeout) {
        AprilTagDetection a;
        while (active() && (runtime.milliseconds() < runtime.milliseconds() + timeout))
            if ((a = detectTag(id)) != null) return a;
        return null;
    }

    /**
     * Aligns the robot in front of the AprilTag.
     *
     * @param id ID of tag to align with.
     */
    public void align(int id) {
        AprilTagDetection a = detectTag(id, 1);
        turn(0);
        while (active() && (a != null && (abs(a.ftcPose.x) > 0.5 || abs(a.ftcPose.yaw) > 0.5))) {
            if ((a = detectTag(id, 1)) == null) return;
            print("Strafe", a.ftcPose.x);
            strafe(a.ftcPose.x);
            if ((a = detectTag(id, 1)) == null) return;
            print("Drive", -a.ftcPose.y + 5);
            drive(-a.ftcPose.y + 2);
            if ((a = detectTag(id, 1)) == null) return;
            print("Turn", a.ftcPose.yaw / 2);
            turn(a.ftcPose.yaw / 2);
            if (!loop) update();
        }
    }

    /**
     * Sends an exception message to Driver Station telemetry.
     *
     * @param e The exception.
     */
    public final void except(Object e) {
        print("Exception", e);
    }

    /**
     * Sleep a specified number of seconds.
     *
     * @param seconds The amount of seconds to sleep.
     */
    public final void s(double seconds) {
        sleep((long) seconds * 1000);
    }

    /**
     * Moves the horizontal lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveHorizontalLift(double encoders) {
        if (liftMotor == null) return;
        if (horizontalTouchSensor != null && horizontalTouchSensor.isPressed()) {
            liftMotor.setMode(STOP_AND_RESET_ENCODER);
        }
        liftMotor.setMode(RUN_USING_ENCODER);
        int direction = (int) signum(encoders - liftMotor.getCurrentPosition());
        liftMotor.setVelocity(LIFT_VEL * direction);

        runtime.reset();
        // The loop stops after being under the encoder goal when going down and when being
        // above the encoder goal when going up
        while (active() && ((liftMotor.getCurrentPosition() < encoders && encoders > 1) || (liftMotor.getCurrentPosition() > encoders && signum(encoders) == -1))) {
            // Display it for the driver.
            print("Position", liftMotor.getCurrentPosition());
            print("Goal", encoders);
            if (!loop) update();
        }
        liftMotor.setVelocity(0);
    }

    /** Retracts the horizontal lift motor. */
    public void retractHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[0]);
    }

    /** Retracts the horizontal lift motor. */
    public void extendHorizontalLift() {
        moveHorizontalLift(LIFT_BOUNDARIES[1]);
    }

    public int getHorizontalLiftPos() {
        if (liftMotor != null) return liftMotor.getCurrentPosition();
        return 0;
    }

    /**
     * Moves the vertical lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveVerticalLift(double encoders) {
        if (verticalMotorA == null) return;
        if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
            verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        verticalMotorA.setMode(RUN_WITHOUT_ENCODER);
        verticalMotorB.setMode(RUN_WITHOUT_ENCODER);
        double basePower;

        while (active()) {
            int vertA = verticalMotorA.getCurrentPosition();
            int vertB = verticalMotorB.getCurrentPosition();

            // Copy the non-zero value if one is zero but the other is large
            if (vertA == 0 && vertB > 100) vertA = vertB;
            if (vertB == 0 && vertA > 100) vertB = vertA;

            int vertAvg = (vertA + vertB) / 2;
            double error = encoders - vertAvg;

            // Stop if close enough
            if (Math.abs(error) < 20) break;

            if (error > 0) basePower = .85;
            else basePower = -.5;

            // Minor correction if one motor is off-center by >10
            if (verticalMotorA.getCurrentPosition() - vertAvg > 10)
                verticalMotorA.setPower(basePower * 0.95);
            else verticalMotorA.setPower(basePower);
            if (verticalMotorB.getCurrentPosition() - vertAvg > 10)
                verticalMotorB.setPower(basePower * 0.95);
            else verticalMotorB.setPower(basePower);

            // If we press the touch sensor, reset encoders
            if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
                verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Better telemetry
            print("Position", vertAvg);
            print("Goal", encoders);
            print("Error", error);
            print("A Power", verticalMotorA.getPower());
            print("B Power", verticalMotorB.getPower());
            if (!loop) update();
        }

        // Turn off power at the end
        verticalMotorA.setPower(0);
        verticalMotorB.setPower(0);
    }

    /**
     * Holds the vertical lift motor at a specified encoder mark
     *
     * @param vertGoal Number of encoders from zero position to turn the motor to
     */
    public void holdVerticalLift(int vertGoal) {
        if (verticalMotorA == null) return;
        hold = true;
        while (active() && hold) {
            int vertA = verticalMotorA.getCurrentPosition();
            int vertB = verticalMotorB.getCurrentPosition();
            if (vertA == 0 && vertB > 100) vertA = vertB;
            if (vertB == 0 && vertA > 100) vertB = vertA;
            int vertAvg = (vertA + vertB) / 2;
            double power = vertAvg < vertGoal - 20 ? 0.15 : (vertGoal - vertAvg) / 20.0 * .1;
            verticalMotorA.setPower(power);
            verticalMotorB.setPower(power);
            print("Holding Lift at", vertGoal);
            if (!loop) update();
        }
        verticalMotorA.setPower(0);
        verticalMotorB.setPower(0);
    }

    /** Retracts the horizontal lift motor. */
    public void retractVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDS[0]);
        sleep(10);
        if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
            verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Retracts the horizontal lift motor. */
    public void extendVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDS[1]);
    }

    /**
     * Gets the position of the vertical lift
     *
     * @return int, average encoder value of the vertical lift motors
     */
    public int getVertLiftPos() {
        if (verticalMotorA == null) return 0;
        int vertA = verticalMotorA.getCurrentPosition();
        int vertB = verticalMotorB.getCurrentPosition();
        int vertAvg = (vertA + vertB) / 2;
        if (vertB == 0 && vertA > 100) vertAvg = vertA;
        if (vertA == 0 && vertB > 100) vertAvg = vertB;
        return vertAvg;
    }

    /**
     * Initializes the AprilTag processor.
     *
     * @param camera The camera to use.
     */
    private void initProcessors(WebcamName camera) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(camera);
        builder.enableLiveView(true);
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving or not
     */
    public void drivetrainLogic(boolean fieldCentric) {
        double axial, lateral, yaw, xMove, yMove;
        double speedMultiplier = gamepad1.left_bumper ? speeds[0] : gamepad1.right_bumper ? speeds[2] : speeds[1];

        if (fieldCentric) {
            double angle = PI / 2 + (useOdometry ? -drive.localizer.getPose().heading.real : -imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle);

            double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));

            xMove = Math.cos(moveAngle) * magnitude;
            yMove = Math.sin(moveAngle) * magnitude;
        } else {
            xMove = gamepad1.left_stick_x;
            yMove = gamepad1.left_stick_y;
        }
        axial = yMove * baseSpeedMultiplier;
        lateral = -xMove * baseSpeedMultiplier;
        yaw = gamepad1.right_stick_x * baseTurnSpeed;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        if (lf != null) {
            lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
            rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
            lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
            rb.setVelocity(rightBackPower * 5000 * speedMultiplier);
        }

        if (abs(leftFrontPower) > .05 || abs(rightFrontPower) > .05 || abs(leftBackPower) > .05 || abs(rightBackPower) > .05) {
            if (following) drive.breakFollowing();
            following = false;
        }

        print("Speed Multiplier", speedMultiplier);
    }

    /** Logic for automatically moving during TeleOp */
    public void autoMovementLogic(boolean validPose) {
        if (!validPose) return;
        if (useOdometry) drive.localizer.update();
        else if (!gamepad1.a) return;
        Pose2d poseEstimate = drive.localizer.getPose();
        if (gamepad1.a) {
            following = true;
            Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                    .lineToLinearHeading(NET_ZONE_POSITION)
                    .build();
            drive.followTrajectoryAsync(trajectory);
        } else if (gamepad1.start) {
            following = true;
            Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                    .lineToLinearHeading(OBSERVATION_ZONE_POSITION)
                    .build();
            drive.followTrajectoryAsync(trajectory);
        }
    }

    /** Logic for the wrist motor during TeleOp */
    public void wristMotorLogic() {
        if (wristMotor == null) return;
        double power = 0;
        int wristMotorPos = wristMotor.getCurrentPosition();
        if (wristMotorPos < 25) {
            moveWristServoX(WRIST_S_GOALS[wristIndex = 2]);
            retractWristServoY();
        } else if (wristMotorPos > 35) hoverWristServoY();
        else moveWristServoY(0.5);
        if (gamepad2.right_stick_button) {
            wristMotorStopPos = 60;
            wristMotorTicksStopped = 15;
            hoverWristServoY();
            openIntake();
        }
        if (gamepad2.right_stick_y > .1 && wristMotorPos < WRIST_M_BOUNDS[1]) {
            power = ticksPowered <= 10 ? 2 * WRIST_MOTOR_POWER : ticksPowered <= 30 ? WRIST_MOTOR_POWER : WRIST_MOTOR_POWER * 0.5;
            wristMotorTicksStopped = 0;
            ticksPowered++;
        } else if (gamepad2.right_stick_y < -.1 && (wristMotorPos > WRIST_M_BOUNDS[0] || gamepad2.right_bumper)) {
            power = ticksPowered <= 10 ? 2 * -WRIST_MOTOR_POWER : ticksPowered <= 30 ? -WRIST_MOTOR_POWER : -WRIST_MOTOR_POWER * 0.5;
            wristMotorTicksStopped = 0;
            if (gamepad2.right_bumper) {
                WRIST_M_BOUNDS[1] += wristMotorPos - WRIST_M_BOUNDS[0];
                WRIST_M_BOUNDS[0] = wristMotorPos;
            }
            ticksPowered++;
        } else {
            int error = wristMotorStopPos - wristMotorPos;
            if (wristMotorTicksStopped < 5)
                wristMotorStopPos = min(WRIST_M_BOUNDS[1], max(WRIST_M_BOUNDS[0], wristMotorPos));
            else
                power = (abs(error) > 3 ? WRIST_MOTOR_POWER * error / 10.0 : 0);
            power = wristMotorTicksStopped < 15 ? max(min(power, WRIST_MOTOR_POWER * 2), -WRIST_MOTOR_POWER * 2) : wristMotorTicksStopped < 30 ? max(min(power, WRIST_MOTOR_POWER), -WRIST_MOTOR_POWER) : max(min(power, WRIST_MOTOR_POWER * 0.5), -WRIST_MOTOR_POWER * 0.5);
            ticksPowered = 0;
            wristMotorTicksStopped++;
        }
        wristMotor.setPower(power);
    }

    /** Logic for the wrist servo during TeleOp. Cycles from 1.0 to 0.5 to 0.0 to 0.5 to 1.0... */
    public void wristServoXLogic(boolean continuous) {
        if (continuous && hypot(gamepad2.left_stick_y, gamepad2.left_stick_x) > .2) {
            double angle = PI / 2 - atan2(gamepad2.left_stick_y, gamepad2.left_stick_x);
            moveWristServoX(min(max(0, (angle - WRIST_S_ANGLES[0]) / (WRIST_S_ANGLES[1] - WRIST_S_ANGLES[0])), 1));
            return;
        }
        if (gamepad2.x) moveWristServoX(WRIST_S_GOALS[wristIndex = 0]);
        if (gamepad2.a && !wasWristServoButtonPressed)
            moveWristServoX(WRIST_S_GOALS[wristIndex = (wristIndex % (WRIST_S_GOALS.length - 1) + 1)]);
        wasWristServoButtonPressed = gamepad2.a;
    }

    /** Logic for the wrist servo during TeleOp. Cycles from 1.0 to 0.5 to 0.0 to 0.5 to 1.0... */
    public void wristServoXLogic() {
        wristServoXLogic(false);
    }

    /** Logic for the intake servo during TeleOp */
    public void intakeLogic() {
        if (gamepad2.b && !wasIntakeServoButtonPressed) {
            if (getWristPos() <= 30)
                moveIntake(getIntakePosition() == 1 ? 0 : 1);
            else {
                if (getIntakePosition() == 1) { // 1 is open
                    wristMotorStopPos = 95;
                    wristMotorTicksStopped = 5;
                    extendWristServoY();
                    wristServoTimer = runtime.milliseconds() + 500;
                    closeIntake();
                } else {
                    wristMotorStopPos = 60;
                    wristMotorTicksStopped = 5;
                    hoverWristServoY();
                    openIntake();
                }
            }
        }
        wasIntakeServoButtonPressed = gamepad2.b;
        if ((runtime.milliseconds() >= wristServoTimer) && (wristServoTimer != 0)) {
            wristMotorStopPos = 60;
            wristMotorTicksStopped = 5;
            hoverWristServoY();
            wristServoTimer = 0;
        }
    }

    /** Logic for the horizontal lift during TeleOp */
    public void horizontalLiftLogic() {
        if (liftMotor == null) return;
        boolean slow = false, liftIn = false, liftOut = false;
        int liftPos = liftMotor.getCurrentPosition();
        if (gamepad2.right_trigger > .1) {
            liftRunToPos = true;
            liftGoal = LIFT_BOUNDARIES[1] - 300;
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right) liftRunToPos = handoff = false;
        if (liftRunToPos) {
            if (liftPos > liftGoal) liftIn = true;
            else liftOut = true;
            slow = abs(liftPos - liftGoal) < 50;
            liftRunToPos = abs(liftPos - liftGoal) > 20;
        }
        slow = gamepad2.left_bumper || slow;
        liftOut = liftOut || gamepad2.dpad_right;
        liftIn = liftIn || gamepad2.dpad_left;
        double power = 0;
        if (liftOut ^ liftIn) {
            // If the touch sensor isn't connected, assume it isn't pressed
            boolean touchSensorPressed = horizontalTouchSensor != null && horizontalTouchSensor.isPressed();
            double speed = slow ? 0.6 : 1;
            if (touchSensorPressed) {
                liftMotor.setMode(STOP_AND_RESET_ENCODER);
                liftMotor.setMode(RUN_WITHOUT_ENCODER);
            }
            if (liftOut) power = liftPos < LIFT_BOUNDARIES[1] ? speed : 0;
            else if (!touchSensorPressed) power = -speed;
        }
        liftMotor.setPower(power);
    }

    /** Logic for the vertical lift during TeleOp */
    public void verticalLiftLogic() {
        if (verticalMotorA == null) return;
        boolean vertUp = false, vertDown = false, slow = false;
        double power = 0;
        int vertA = verticalMotorA.getCurrentPosition();
        int vertB = verticalMotorB.getCurrentPosition();
        int vertAvg = (vertA + vertB) / 2;
        // Relies on one encoder if one seems disconnected
        if (vertB == 0 && vertA > 100) vertAvg = vertB = vertA;
        if (vertA == 0 && vertB > 100) vertAvg = vertA = vertB;
        if ((gamepad2.dpad_up || gamepad2.dpad_down) && !gamepad2.right_bumper)
            vertRunToPos = false;
        else {
            if (gamepad2.right_bumper && (isDpu ^ isDpd)) {
                vertGoal = vertRunToPos ? vertGoal : vertAvg;
                if (isDpu) {
                    for (int goal : V_LIFT_GOALS) {
                        if (goal > vertGoal + 50) {
                            vertGoal = goal;
                            break;
                        }
                    }
                } else { // isDpd is always true because of the conditions to enter the if
                    int newGoal = vertGoal;
                    for (int goal : V_LIFT_GOALS) {
                        if (goal < vertGoal - 50) newGoal = goal;
                        else break;
                    }
                    vertGoal = newGoal;
                }
                vertRunToPos = true; // Ensure the flag is set
            }
            if (vertAvg - 20 < vertGoal && vertGoal < vertAvg + 20) {
                vertRunToPos = false;
                vertStopped = true;
            }
        }
        if (vertRunToPos && (getRuntime() - verticalLiftTimer >= 0)) {
            if (vertAvg < vertGoal) {
                vertUp = true;
                if (vertAvg >= vertGoal - 50) slow = true;
            } else {
                vertDown = true;
                if (vertAvg < vertGoal + 50) slow = true;
            }
        }
        if (!gamepad2.right_bumper) {
            vertUp = gamepad2.dpad_up || vertUp;
            vertDown = gamepad2.dpad_down || vertDown;
        }
        slow = slow || gamepad2.left_bumper;
        if (vertUp == vertDown) {
            if (!vertStopped && !gamepad2.right_bumper) {
                vertStopped = true;
                vertGoal = vertAvg;
            }
            power = vertAvg < vertGoal - 20 ? 0.1 : (vertGoal - vertAvg) / 20.0 * .1;
        } else {
            vertStopped = false;
            // If the touch sensor isn't connected, assume it isn't pressed
            boolean touchSensorPressed = verticalTouchSensor != null && verticalTouchSensor.isPressed();
            if (touchSensorPressed) {
                verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
                verticalMotorB.setMode(STOP_AND_RESET_ENCODER);
                verticalMotorA.setMode(RUN_WITHOUT_ENCODER);
                verticalMotorB.setMode(RUN_WITHOUT_ENCODER);
            }
            if (vertUp && getWristPos() < 30) {
                wristMotorStopPos = 30;
                wristMotorTicksStopped = 5;
            }
            if (vertUp && !(vertAvg < 100 && getWristPos() < 25))
                power = vertAvg < V_LIFT_BOUNDS[1] ? slow ? 0.7 : 1 : 0;
            else if (!touchSensorPressed) power = vertAvg > V_LIFT_BOUNDS[0] ? slow ? -0.5 : -1 : 0;
        }
        if (vertA > vertB + 5 && power != 0) {
            verticalMotorA.setPower(power - .05);
            verticalMotorB.setPower(power + .05);
        } else if (vertB > vertA + 5 && power != 0) {
            verticalMotorA.setPower(power + .05);
            verticalMotorB.setPower(power - .05);
        } else {
            verticalMotorA.setPower(power);
            verticalMotorB.setPower(power);
        }
        print("Vertical Lift Goal", vertGoal);
    }

    /** Logic for the basket servo during TeleOp */
    public void basketServoLogic() {
        if (gamepad1.x && !wasBasketServoButtonPressed && !(getVertLiftPos() < 100 && getWristPos() < 25))
            moveBasketServo(getBasketPosition() == 0 ? 1 : 0);
        wasBasketServoButtonPressed = gamepad1.x && !(getVertLiftPos() < 100 && getWristPos() < 25);
    }

    /** Logic for the specimen servo during TeleOp */
    public void specimenServoLogic() {
        if (gamepad1.b && !wasSpecimenServoButtonPressed)
            moveSpecimenServo(getSpecimenPosition() == 0 ? 1 : 0);
        wasSpecimenServoButtonPressed = gamepad1.b;
    }

    /** Logic for y button handoff */
    public void handoffLogic() {
        if (handoff && getHorizontalLiftPos() < 5 && getWristPos() < 5) {
            handoff = false;
            openIntake();
            moveWristServoY(.5);
            verticalLiftTimer = getRuntime() + 0.5;
            vertRunToPos = true;
            vertGoal = V_LIFT_GOALS[4];
        }
        if (gamepad2.y) {
            handoff = true;
            wristMotorTicksStopped = 5;
            wristMotorStopPos = 0;
            moveWristServoX(WRIST_S_GOALS[wristIndex = 2]);
            retractWristServoY();
            liftRunToPos = true;
            liftGoal = 0;
        }
    }

    /** Logic for dpad buttons **/
    public void dpadLogic() {
        if (gamepad1.dpad_up && !isDpu && !wasDpu) isDpu = wasDpu = true;
        else if (gamepad1.dpad_up && isDpu && wasDpu) isDpu = false;
        else if (!gamepad1.dpad_up && wasDpu) wasDpu = false;
        else if (!gamepad1.dpad_down && isDpu) isDpu = wasDpu = false;

        if (gamepad1.dpad_down && !isDpd && !wasDpd) isDpd = wasDpd = true;
        else if (gamepad1.dpad_down && isDpd && wasDpd) isDpd = false;
        else if (!gamepad1.dpad_down && wasDpd) wasDpd = false;
        else if (!gamepad1.dpad_down && isDpd) isDpd = wasDpd = false;
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
        telemetry.addLine(content);
    }

    /**
     * Show something via telemetry and wait a specified number of seconds
     *
     * @param content Content to be shown to the user
     * @param seconds Seconds to wait after message is shown
     */
    public void printSeconds(String content, double seconds) {
        print(content);
        double t0 = getRuntime();
        if (!loop) update();
        if (!loop) s(seconds);
        else while (getRuntime() - t0 < seconds) print(content);
    }

    /** A less space consuming way to update the displayed telemetry. */
    public void update() {
        telemetry.update();
    }

    /**
     * Adds telemetry data from the last action
     *
     * @param message Message to be sent
     */
    public void addLastActionTelemetry(String message) {
        print("Last Action", message);
    }

    /**
     * Asks the user to confirm whether something happened or whether they want something to happen
     *
     * @param message Message the user will see before confirming
     */
    public boolean confirm(String message) {
        print(message);
        print("Press A to confirm and B to cancel (Gamepad 1)");
        update();
        while (active()) {
            if (gamepad1.a || gamepad1.b) break;
        }
        if (gamepad1.a) printSeconds("Confirmed.", .5);
        else printSeconds("Canceled.", .5);
        return gamepad1.a;
    }

    /** Repeatedly reports info about the robot via telemetry. Stopped by setting loop to false. */
    public void telemetryLoop() {
        loop = true;
        while (active() && loop) {
            updateAll();
            if (auto) saveOdometryPosition(drive.localizer.getPose());
        }
    }

    /**
     * Loads the current pose from a file.
     *
     * @return The position the robot ended at in the last Auto
     */
    @Nullable
    public Pose2d loadOdometryPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        if (file.exists()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
                String line = reader.readLine();
                if (line != null) {
                    String[] values = line.split(",");
                    double x = Double.parseDouble(values[0]); // X
                    double y = Double.parseDouble(values[1]); // Y
                    double h = Double.parseDouble(values[2]); // Heading
                    // Delete the file after reading it
                    if (!file.delete()) {
                        print("Error", "Failed to delete odometry file.");
                    }
                    return new Pose2d(x, y, h);
                }
            } catch (IOException e) {
                print("Error", "Failed to load odometry position: " + e.getMessage());
            }
        }
        return null;
    }

    /**
     * Saves the current pose to a file.
     *
     * @param pos Pose to save
     */
    public void saveOdometryPosition(@NonNull Pose2d pos) {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        try (FileWriter writer = new FileWriter(file, false)) {
            writer.write(pos.position.x + "," + pos.position.y + "," + pos.heading.real); // Write the latest position
        } catch (IOException e) {
            print("Error", "Failed to save odometry position: " + e.getMessage());
        }
    }

    /** Adds information messages to telemetry and updates it */
    public void updateAll() {
        if (lf == null) telemetry.addData("Drive Train", "Disconnected");
        if (verticalMotorA == null) print("Vertical Lift Motors", "Disconnected");
        else {
            print("Vertical Motor A Position", verticalMotorA.getCurrentPosition());
            print("Vertical Motor B Position", verticalMotorB.getCurrentPosition());
            print("Vertical Motor Power", (verticalMotorA.getPower() + verticalMotorB.getPower()) / 2.0);
            print("Vertical Run To Position", vertRunToPos);
        }
        if (liftMotor == null) print("Horizontal Lift Motor", "Disconnected");
        else {
            print("Horizontal Lift Motor Position", liftMotor.getCurrentPosition());
            print("Horizontal Lift Motor Power", liftMotor.getPower());
            print("Lift Run To Position", liftRunToPos);
            print("Lift Goal", liftGoal);
        }
        if (wristMotor == null) print("Wrist Motor", "Disconnected");
        else print("Wrist Motor Position", wristMotor.getCurrentPosition());

        if (wristServoX == null) print("Wrist Servo", "Disconnected");
        else print("Wrist Servo Position", wristServoX.getPosition());
        if (intakeServo == null) print("Intake Servo", "Disconnected");
        else print("Intake Servo Position", intakeServo.getPosition());
        if (specimenServo == null) print("Specimen Servo", "Disconnected");
        else print("Specimen Servo Position", specimenServo.getPosition());
        if (basketServo == null) print("Basket Servo", "Disconnected");
        else print("Basket Servo Position", basketServo.getPosition());

        if (verticalTouchSensor == null) print("Touch Sensor", "Disconnected");
        else print("Touch Sensor Pressed", verticalTouchSensor.isPressed());
        if (wristServoX == null) print("Intake Servo", "Disconnected");
        if (useOdometry) {
            Pose2d pos = drive.localizer.getPose();
            print(String.format(US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f°", pos.position.x, pos.position.y, toDegrees(pos.heading.real)));
        } else print("Odometry disabled");


        if (drive != null && drive.localizer instanceof RedundantLocalizer) {
            RedundantLocalizer localizer = (RedundantLocalizer) drive.localizer;
            boolean usingPrimary = localizer.isUsingPrimaryLocalizer();
            print("Using Primary Localizer", usingPrimary);
        } else print("Localizer is not a RedundantLocalizer");

        update();
    }

    /** Returns whether the robot is active. */
    public boolean active() {
        return opModeIsActive() && !isStopRequested() && running;
    }
}
