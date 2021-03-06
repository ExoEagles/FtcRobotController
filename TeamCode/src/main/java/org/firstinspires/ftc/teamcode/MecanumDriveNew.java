package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.Robot;

@TeleOp(name="Mecanum Drive New", group="TeleOP")
public class MecanumDriveNew extends OpMode
{
    public Robot robot = null;

    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void loop()
    {
        final double ROOT2 = 1.41421356237;

        final double magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        final double rightX = gamepad1.right_stick_x;

        final double fld = ((magnitude * Math.cos(robotAngle) + rightX) * ROOT2);
        final double frd = ((magnitude * Math.sin(robotAngle) - rightX) * ROOT2);
        final double brd = ((magnitude * Math.cos(robotAngle) - rightX) * ROOT2);
        final double bld = ((magnitude * Math.sin(robotAngle) + rightX) * ROOT2);

        robot.setPowerAll(fld, frd, brd, bld);

        robot.leftShooter.setPower(gamepad2.right_trigger * robot.SHOOTER_POWER);
        robot.rightShooter.setPower(gamepad2.right_trigger * robot.SHOOTER_POWER);

        robot.stackServo.setPower(gamepad2.left_stick_y);

        robot.wobbleCoreMotor.setPower(-gamepad2.right_stick_y);

        robot.frontIntake.setPower(gamepad1.left_trigger);
        robot.frontIntake.setPower(-gamepad1.right_trigger);

        if (gamepad1.dpad_left) { robot.wobbleServo.setPosition(robot.WOBBLE_CLOSED); }
        if (gamepad1.dpad_right) { robot.wobbleServo.setPosition(robot.WOBBLE_OPEN); }

        if (gamepad2.a && robot.debounceOK()) { robot.uptakeOn = !robot.uptakeOn; robot.elapsedTime.reset(); }

        if (gamepad2.y) { robot.ringServo.setPosition(robot.RING_UP); }
        if (gamepad2.x) { robot.ringServo.setPosition(robot.SERVO_LOCK); }

        if (robot.uptakeOn) { robot.uptakeServo.setPower(robot.MAX_POWER); }
        else { robot.uptakeServo.setPower(robot.MIN_POWER); }
    }
}