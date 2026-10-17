package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumDriveTeleOp", group = "Linear OpMode")
public class MecanumDriveTeleOp extends LinearOpMode {
    private DcMotor LUmotor;
    private DcMotor LBmotor;
    private DcMotor RUmotor;
    private DcMotor RBmotor;

    private DcMotor intake;
    private DcMotor Lcornerintake;
    private DcMotor Rcornerintake;

    private DcMotor flywheel;
	private void setup(){
		LUmotor = hardwareMap.get(DcMotor.class, "LUmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RUmotor = hardwareMap.get(DcMotor.class, "RUmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");

        intake = hardwareMap.get(DcMotor.class, "intake");
        Lcornerintake = hardwareMap.get(DcMotor.class, "Lcornerintake");
        Rcornerintake = hardwareMap.get(DcMotor.class, "Rcornerintake");

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RUmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
	}
	private handleMovement(){
	 	double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            
            double luPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double ruPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            LUmotor.setPower(luPower);
            LBmotor.setPower(lbPower);
            RUmotor.setPower(ruPower);
            RBmotor.setPower(rbPower);

            if (gamepad1.a) {
                intake.setPower(1.0);
                Lcornerintake.setPower(1.0);
                Rcornerintake.setPower(1.0);
            } else {
                intake.setPower(0.0);
                Lcornerintake.setPower(0.0);
                Rcornerintake.setPower(0.0);
            }

            if (gamepad1.b) {
                flywheel.setPower(1.0);
            } else {
                flywheel.setPower(0.0);
            }

            telemetry.addData("Drive Motors", "LU: %.2f, RU: %.2f", luPower, ruPower);
            telemetry.addData("Intake Status", gamepad1.a ? "Running" : "Stopped");
            telemetry.addData("Flywheel Status", gamepad1.b ? "Running" : "Stopped");
            
	}
    @Override
    public void runOpMode() {
    	this.setup();
        waitForStart();

        while (opModeIsActive()) {
           this.handleMovement();
           telemetry.update();
        }
    }
}
