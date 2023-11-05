/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="16657 Slow Power Play Clasper Manual Lift Cone Stack Faster Drive", group="Linear Opmode")
//@Disabled
public class SlowPowerPlayClasperManualLiftconestackfasterdrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    //private DigitalChannel touch;
    
    //declare servos for clasper, as looking down on robot from behind, servoCW is the servo on the
    // left that moves clockwise to clasp, servoCCW is on the right and moves counter-clockwise to clasp
    private Servo servoCW;
    private Servo servoCCW;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        //touch = (DigitalChannel) hardwareMap.get("touchSensor");
        
        servoCW = hardwareMap.servo.get("servoCW");
        servoCCW = hardwareMap.servo.get("servoCCW");



        /*// might possible need to comment out line 92 VERY IMPORTANT
        liftMotor.setDirection((DcMotor.Direction.REVERSE));
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touch.setMode(DigitalChannelController.Mode.INPUT);*/


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Setting motor to reverse so that it will lift cones when positive values are entered
        //Setting motor to brake so that it will maintain the encoder value and not slip
        //Resetting the motor to establish start position
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //these are the variables of the code, they represent changing information that makes the code operate

        double slow = 1.0;
        double strafeslow = 1.0;
        double liftPower = .8;
        double nudgePower = .9;
        double manualLiftPower = .5;
        boolean liftMoving = false;
        int targetEncoderValue = 0;
        //int plusEncoderValue = targetEncoderValue + 15;
        //int minusEncoderValue = targetEncoderValue - 15;
        int positionStart = 0;
        int coneStackTop = 500;
        int positionLow = 1275;
        int positionMid = 2200;
        int positionHigh = 3000;
        int liftNudge = 35;
        double servoCWPosition = 0;
        double servoCCWPosition =0;
        //set a constant for clasper servo open and closed position (in future, use constant not
        //variable)
        double openClasper = 0.05;
        double closedClasper = 0.75;




        // run until the end of the match (driver presses STOP)
        // this is code for the drivetrain, specifically for strafing
        while (opModeIsActive()) {
            double max;

            if (gamepad1.right_trigger > 0.7) {
                slow = 0.25;
                strafeslow = 0.3525;
            } else {
                slow = 0.5;
                strafeslow = 0.5;
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            //google if not sure

            axial = -gamepad1.left_stick_y * slow;
            lateral = gamepad1.left_stick_x * 0.75 * slow;
            yaw = gamepad1.right_stick_x * strafeslow;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;



            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //creates a manual system for controlling the lift using the dpad on controller B

            if (gamepad2.right_bumper) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition()+liftNudge);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(nudgePower);
            }


            if (gamepad2.left_bumper) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition()-liftNudge);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(nudgePower);
            }

            //when the d-pad down button is pressed, the lift system will run to the top position of the cone stack

            if (gamepad2.dpad_down && !liftMoving){
                targetEncoderValue = coneStackTop;
                liftMotor.setTargetPosition(targetEncoderValue);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(liftPower);
            }
            //various positions for the conestack, allows and sets certain heights


            if (gamepad2.y && !liftMoving){
                targetEncoderValue = positionLow;
                liftMotor.setTargetPosition(targetEncoderValue);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(liftPower);
            }

            if (gamepad2.b && !liftMoving){
                targetEncoderValue = positionMid;
                liftMotor.setTargetPosition(targetEncoderValue);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(liftPower);
            }

            if (gamepad2.a && !liftMoving)
            {
                targetEncoderValue = positionHigh;
                liftMotor.setTargetPosition(targetEncoderValue);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(liftPower);
            }

            if (gamepad2.x && !liftMoving)
            {
                targetEncoderValue = positionStart;
                liftMotor.setTargetPosition(targetEncoderValue);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMoving = true;
                liftMotor.setPower(liftPower);
            }


            if(liftMoving && (!liftMotor.isBusy())){
                liftMotor.setPower(0.0);
                sleep(200);
                liftMoving = false;
            }


            
            //Checking for the clasper, liftMoving based on current state
            //
            // FUTURE: Describe in non-programming terms why subtracting clasper target values
            // from 1.
            //
            //
            if (gamepad1.x) {
                // If clasper is closed, open it
                if (servoCWPosition == closedClasper && servoCCWPosition == 1-closedClasper) {
                    servoCWPosition = openClasper;
                    servoCCWPosition = 1-openClasper;
                    servoCW.setPosition(servoCWPosition);
                    servoCCW.setPosition(servoCCWPosition);
                }
                // else clasper is open, close it
                else {
                    servoCWPosition = closedClasper;
                    servoCCWPosition = 1-closedClasper;
                    servoCW.setPosition(servoCWPosition);
                    servoCCW.setPosition(servoCCWPosition);
                }
                sleep(250);
            }





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("CW/CCW", "%4.2f, %4.2f", servoCWPosition, servoCCWPosition);
            telemetry.addData("Status", "Target Encoder Value: " + targetEncoderValue);
            telemetry.addData("lift", " Encoder: " + liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }}