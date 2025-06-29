package org.firstinspires.ftc.teamcode.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SparkFunLocalizer;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

public class RedundantLocalizer implements Localizer {

    private final Localizer primaryLocalizer;
    private final Localizer secondaryLocalizer;
    private boolean usePrimary = true;

    public RedundantLocalizer(@NonNull HardwareMap hardwareMap, Pose2d initialPose) {
        this.primaryLocalizer = new OTOSLocalizer(hardwareMap, initialPose);
        this.secondaryLocalizer = new OTOSLocalizer(hardwareMap, initialPose);
    }

    @Override
    public PoseVelocity2d update() {
        secondaryLocalizer.update();
        return primaryLocalizer.update();

        // Switch if SparkFun is unreliable; Skip check if already found unreliable
//        if (usePrimary && !isSparkFunReliable()) usePrimary = false;
    }

    public boolean isUsingPrimaryLocalizer() {
        return usePrimary;
    }

    @NonNull
    @Override
    public Pose2d getPose() {
        return usePrimary ? primaryLocalizer.getPose() : secondaryLocalizer.getPose();
    }

    @Override
    public void setPose(@NonNull Pose2d pose) {
        primaryLocalizer.getPose();
        secondaryLocalizer.getPose();
    }


    private boolean isSparkFunReliable() {
        Pose2d sfPose = primaryLocalizer.getPose();
        Pose2d encPose = secondaryLocalizer.getPose();
        double[] sfMeasurements = {sfPose.position.x, sfPose.position.y};
        double[] encMeasurements = {encPose.position.x, encPose.position.y};

        double error;
        double compounding_error = 0;
        for (int i = 0; i < sfMeasurements.length; i++) {
            error =  getError(sfMeasurements[i], encMeasurements[i]);
            if (error > .5) return false;
            else if (error > .2) compounding_error += error;
        }

        return !(compounding_error > 1);
    }

    /** Gets the normalized error between two numbers
     *
     * @param a First number
     * @param b Second number
     * @return Normalized error
     */
    private double getError(double a, double b) {
        return Math.abs(a - b) / ((Math.abs(a) + Math.abs(b)) / 2.0);
    }
}