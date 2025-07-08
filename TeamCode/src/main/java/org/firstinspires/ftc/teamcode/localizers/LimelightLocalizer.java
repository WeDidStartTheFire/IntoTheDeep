package org.firstinspires.ftc.teamcode.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.jetbrains.annotations.NotNull;

@Config
public class LimelightLocalizer implements Localizer {
    private final Limelight3A limelight;
    private final IMU imu;
    private long prevTime;
    @Nullable
    private Pose2d pose;
    private Pose2d prevPose;

    public LimelightLocalizer(HardwareMap map, @NotNull Pose2d initialPose) {
        imu = map.get(IMU.class, "imu");
        limelight = map.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // How often Limelight is asked for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
        prevTime = System.currentTimeMillis();
        prevPose = initialPose;
    }

    @Nullable
    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose2d pose) {
    }

    @Override
    public PoseVelocity2d update() {
        long currTime = System.nanoTime();
        double dt = (currTime - prevTime) / 1_000_000.0;
        double robotYaw = imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;
        double yawRate = imu.getRobotAngularVelocity(RADIANS).zRotationRate;
        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, yawRate, 0, 0, 0, 0);

        pose = null;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            Pose3D botpose = result.getBotpose();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                pose = new Pose2d(x, y, botpose_mt2.getOrientation().getYaw());
            } else if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                pose = new Pose2d(x, y, botpose.getOrientation().getYaw());
            }
        }

        if (pose != null) {
            double dx = pose.position.x - prevPose.position.x;
            double dy = pose.position.y - prevPose.position.y;
            double dtheta = pose.heading.real - prevPose.heading.real;
            prevPose = pose;
            prevTime = currTime;
            return new PoseVelocity2d(new Vector2d(dx / dt, dy / dt), dtheta / dt);
        }
        return null;
    }
}
