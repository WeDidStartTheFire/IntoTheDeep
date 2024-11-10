package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Zone Simple +", group = "IntoTheDeep", preselectTeleOp = "Main")
public class Auto_NetZoneSimplePlus extends Base {
    @Override
    public void runOpMode() {
        setup();
        strafe(2, RIGHT);
        drive(12, FORWARD);
        extendWrist();
        sleep(250);
        openIntake();
        sleep(250);
        drive(76, BACKWARD);
    }
}