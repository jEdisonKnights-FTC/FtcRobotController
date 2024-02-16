package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Tester extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor x = hardwareMap.get(DcMotor.class, "testing");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            x.setPower(1);
        }
    }
}
