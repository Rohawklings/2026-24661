package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
@TeleOp(name = "MecanumDriveTeleOp", group = "Linear OpMode")
public class MecanumDriveTeleOp extends LinearOpMode {
    //Motor definitions
    private DcMotor LUmotor;
    private DcMotor LBmotor;
    private DcMotor RUmotor;
    private DcMotor RBmotor;
    private DcMotor intake;//main i
    private DcMotor outtake;

    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() {
        LUmotor = hardwareMap.get(DcMotor.class, "LUmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RUmotor = hardwareMap.get(DcMotor.class, "RUmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        //Lcornerintake = hardwareMap.get(DcMotor.class, "Lcornerintake");
        //Rcornerintake = hardwareMap.get(DcMotor.class, "Rcornerintake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RUmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            double y = - gamepad1.left_stick_y;
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
                intake.setPower(-1.0);
                //Lcornerintake.setPower(-1.0);
                //Rcornerintake.setPower(1.0);
            } else if (gamepad1.x){
                intake.setPower(2.0);
            } else {
                intake.setPower(0.0);
                //Lcornerintake.setPower(0.0);
                //Rcornerintake.setPower(0.0);
            }
            if (gamepad1.y){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                 //fires outtake
                outtake.setPower(-1.0);
            }

            telemetry.addData("Drive Motors", "LU: %.2f, RU: %.2f LB: %.2f RB: %.2f", luPower, ruPower,lbPower, rbPower);
            telemetry.addData("Intake Status", gamepad1.a ? "Running" : "Stopped");
            telemetry.addData("Outtake Status", gamepad1.y ? "Running" : "Stopped");
            telemetry.addData("Odomety","X: %.2f, Y: %.2f",odo.getPosX(),odo.getPosY());
            telemetry.update();
        }
    }
}


