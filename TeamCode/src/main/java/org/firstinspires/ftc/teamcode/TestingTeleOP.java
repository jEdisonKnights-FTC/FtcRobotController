package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class TestingTeleOP extends LinearOpMode {
    private DcMotor front_left, front_right, back_left, back_right;
    private Servo servo_1, servo_2;
    private RevColorSensorV3 color_sensor;
    private DcMotor subsystem_motor;
    private GamepadEx gamepad;
    private ButtonReader a, b, y, x;
    private TriggerReader right_trigger, left_trigger;

    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
        color_sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        subsystem_motor = hardwareMap.get(DcMotor.class, "subsystem_motor");

        gamepad = new GamepadEx(gamepad1);
        a = new ButtonReader(gamepad, GamepadKeys.Button.A);
        b = new ButtonReader(gamepad, GamepadKeys.Button.B);
        y = new ButtonReader(gamepad, GamepadKeys.Button.Y);
        x = new ButtonReader(gamepad, GamepadKeys.Button.X);
        right_trigger = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        left_trigger = new TriggerReader(gamepad, GamepadKeys.Trigger.LEFT_TRIGGER);

        while(opModeIsActive() && !isStopRequested()) {
            double max;

            double axial = -gamepad.getLeftY();
            double lateral = gamepad.getLeftX();
            double yaw =  gamepad.getRightX();

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            front_left.setPower(leftFrontPower);
            front_right.setPower(rightFrontPower);
            back_left.setPower(leftBackPower);
            back_right.setPower(rightBackPower);

            if (a.wasJustPressed()) {
                servo_2.setPosition(0);
            }

            if (b.wasJustPressed()) {
                servo_2.setPosition(1);
            }

            if (x.wasJustPressed()) {
                servo_1.setPosition(0);
            }

            if (y.wasJustPressed()) {
                servo_1.setPosition(1);
            }

            if (right_trigger.wasJustPressed()) {
                subsystem_motor.setPower(1);
            }
            else if (left_trigger.wasJustPressed()){
                subsystem_motor.setPower(-1);
            }
            else {
                subsystem_motor.setPower(0);
            }

            telemetry.addData("color sensor data", color_sensor.getLightDetected());
            telemetry.addData("subsystem_motor_pos", subsystem_motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
