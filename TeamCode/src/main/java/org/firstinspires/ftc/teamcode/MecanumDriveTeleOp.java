package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private int shootRPM;

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // initalize motors and odo
        shootRPM = 1000;
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
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // setup odometry
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        // update to actual values later if these are off
        // probably the most likely to be off right now
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // report status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorPosUndAttend(transfer,99);
        waitForStart();

        while (opModeIsActive()) {
            double target_tps = 0;
            //tps readigns is in increments of 10 idk why

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
                intakePower = gamepad1.a ? 1.0 : -1.0;
            }
            intake.setPower(intakePower);

            // reset odometry
            if (gamepad1.y) {
                odo.resetPosAndIMU();
            }
            double outtakeInput = gamepad1.left_trigger;
            if (outtakeInput > 0.0) {
                target_tps = rpm_to_tps(shootRPM);
                //tps readings is in increments of 10 idk why
                outtake.setVelocity(target_tps);
                double tolerence = 0.1;
                double upper = target_tps * (1 + tolerence);
                double lower = target_tps * (1 - tolerence);
                while (
                        !((outtake.getVelocity() < upper)
                                && (outtake.getVelocity() > lower))
                                && !(gamepad1.b)
                ) {

                    sleep(50);
                    telemetry.addData("current outtake rpm","target: %.2f, cur: %.2f", target_tps,outtake.getVelocity());
                    telemetry.update();
                }
                setMotorPosUndAttend(transfer,-57);
                setMotorPosUndAttend(transfer,99);
            }

            if(gamepad1.b){
                target_tps=rpm_to_tps(shootRPM/3);
                outtakeInput=0.0;
                outtake.setVelocity(target_tps);
                transfer.setPower(0.0);
                telemetry.addData("shooting halted","");
            }
            telemetry.addData("Drive Motors", "LU: %.2f, RU: %.2f LB: %.2f RB: %.2f", luPower, ruPower, lbPower, rbPower);
            telemetry.addData("Outtake Status", outtakeInput > 0 ? "Running" : "Stopped");
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f", odo.getPosX(), odo.getPosY());
            telemetry.update();
        }

    }
    private void setMotorPosUndAttend(DcMotorEx motor,int pos){
        motor.setTargetPosition(pos);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setPower(1);
        double tar_pos_upper=transfer.getTargetPosition()*1.1;
        double tar_pos_under=transfer.getTargetPosition()*0.9;
        while((transfer.getCurrentPosition()< tar_pos_under) || (transfer.getCurrentPosition()>tar_pos_upper)){
            telemetry.addData("current transfer pos","target: %d, cur: %d", transfer.getTargetPosition(),transfer.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
        motor.setPower(0);
    }
    private double rpm_to_tps(double rpm) {
        int ticks_per_round = 28;
        return rpm * ticks_per_round / 60;
    }
}

/*


Right Trigger - Intake (A to reverse)
Left Trigger - Outtake (scaled by trigger value) & Transfer
Y - Reset odometry
B - Halt shot

FOR YGS TO DO (DO NOT DELETE THIS COMMENT):
- figure out what works for ygs (the controls, whether yall want to use two drivers, etc)
- states for storing the balls in transfer without shooting
- after limelight is mounted, work out auto-aiming and shooting
    - velocity interpolation for your flywheel is necessary, ask me if you're stuck or unsure
    - may need velocity adjustment for shotoing multiple balls in a row. 
- if yall are going to use 2 limelights, one in the front and the back, figure out how you're going to spin 180
    and reidentify the apriltag in the other limelight so you can shoot. (your ramp/flywheel shoots out the back, so this is a 
    concern of mine)
    - One limelight for one side is ENOUGH if calibrated correctly
- Pedro Pathing setup (ASAP, ask me if you have problems, I had a TON last year)
- If yall need help with servos (the pollen something), i can also help 
FRANCAIS
212 470 2128
www.elitefrenchtutoring.com
*/