package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Observation Zone Single Specimen No Park", group = "!!!Primary", preselectTeleOp = "Main")
@Disabled
public class Auto_ObservationZone_SingleSpecimenNoPark extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        auto = true;
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        running = true;
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        useOdometry = false;
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        Thread holdWristOutOfWay = new Thread(this::holdWristOutOfWay);
        try {
            closeSpecimenServo();
            moveWristServoY(0.5);
            wristOutOfWay();
            holdWristOutOfWay.start();
            // Start both threads
            liftThread.start();
            liftThread.join();
            driveThread.start();
            holdLift.start();
            driveThread.join();
            useOdometry = true;
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY() - 30, currentPose.getHeading());
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 250);
            openSpecimenServo();
            moveWrist(0);
        } finally {
            running = false;
            hold = false;
            loop = false;
            holdWrist = false;
            telemetryThread.interrupt();
            driveThread.interrupt();
            liftThread.interrupt();
            holdLift.interrupt();
            holdWristOutOfWay.interrupt();
            stop();
        }
    }
}
