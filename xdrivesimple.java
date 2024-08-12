package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class xdrivesimple extends LinearOpMode {
    private double camMotorPosition = 0;
    private double accumulatedAngle = 0;
    private double lastYaw = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor camMotor = hardwareMap.dcMotor.get("camMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        camMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
        
        waitForStart();
        
        double kP = 0.4;
        double targetAngle = 0;
        double error;
        double botHeading;
        double ticksPerDegree = 560 / 360;
        float posCam = 0;

        while (opModeIsActive()) {
            posCam += gamepad2.right_stick_x * 0.1;
            int position = camMotor.getCurrentPosition();
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double deltaYaw = botHeading - lastYaw;
            deltaYaw = normalizeAngle(deltaYaw);
            accumulatedAngle += deltaYaw;
            lastYaw = botHeading;

            targetAngle += gamepad1.right_stick_x * 0.06;
            targetAngle = normalizeAngle(targetAngle);
            
            error = normalizeAngle(targetAngle - botHeading);
            
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x; 
            double rx = error * kP;

            if (gamepad1.options) {
                imu.resetYaw();
                accumulatedAngle = 0;
                lastYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; 
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            camMotor.setTargetPosition((int) Math.round((((accumulatedAngle + posCam) * (180 / Math.PI)) * -1) * ticksPerDegree));
            
            camMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            camMotor.setPower(1);

            telemetry.addData("accumulated angle: ", accumulatedAngle);
            telemetry.addData("current angle:", botHeading);
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
