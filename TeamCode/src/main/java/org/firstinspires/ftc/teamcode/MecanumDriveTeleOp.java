package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MecanumDriveTeleOp", group = "Linear OpMode")
public class MecanumDriveTeleOp extends LinearOpMode {

    // declare motors and odom
    private DcMotorEx LUmotor;
    private DcMotorEx LBmotor;
    private DcMotorEx RUmotor;
    private DcMotorEx RBmotor;
    private DcMotorEx intake;
    private DcMotorEx outtake;
    private DcMotorEx transfer;

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // initalize motors and odo
        LUmotor = hardwareMap.get(DcMotorEx.class, "LUmotor");
        LBmotor = hardwareMap.get(DcMotorEx.class, "LBmotor");
        RUmotor = hardwareMap.get(DcMotorEx.class, "RUmotor");
        RBmotor = hardwareMap.get(DcMotorEx.class, "RBmotor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // define motor directions
        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RUmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        // setup odometry
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // report status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            // basic drive code
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

            //intake
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0) {
                // may be reversed, ftc kids plz fix as i have no access to actual robot
                intakePower = gamepad1.a ? -1.0 : 1.0;
            }
            intake.setPower(intakePower);

            // reset odometry
            if (gamepad1.y) {
                odo.resetPosAndIMU();
            }

            // outtake velocity scales with left trigger, transfer runs at full power while outtaking
            double outtakeInput = gamepad1.left_trigger;
            outtake.setPower(-outtakeInput);
            transfer.setPower(outtakeInput > 0 ? 1.0 : 0.0);

            telemetry.addData("Drive Motors", "LU: %.2f, RU: %.2f LB: %.2f RB: %.2f",
                    luPower, ruPower, lbPower, rbPower);
            telemetry.addData("Outtake Status", outtakeInput > 0 ? "Running" : "Stopped");
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f", odo.getPosX(), odo.getPosY());
            telemetry.update();
        }
    }
}
