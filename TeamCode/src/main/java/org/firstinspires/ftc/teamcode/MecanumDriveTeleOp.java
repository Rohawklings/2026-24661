package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumDriveTeleOp extends OpMode {

    static final int GATE_CLOSED_POS = 0;
    static final int GATE_OPEN_POS = 150;
    static final double GATE_POWER = 0.7;
    static final double GATE_JOG_POWER = 0.3;

    static final double FLYWHEEL_POWER = 1.0;
    static final double FLYWHEEL_READY_VELOCITY = 0;
    static final long FLYWHEEL_MIN_SPINUP_MS = 1000;
    static final long FLYWHEEL_MAX_SPINUP_MS = 1500;

    static final double INTAKE_POWER = 1.0;
    static final boolean INTAKE_REVERSED = false;
    static final long DEPOL_SPINUP_TIME=3000;

    private DcMotor LBmotor, LUmotor, RBmotor, RUmotor;
    private GoBildaPinpointDriver odo;
    private DcMotor intake;
    private DcMotorEx outtake;
    private DcMotor transfer;
    Limelight3A limelight;
    private Servo depollinator;

    private boolean flywheelOn = false;
    private boolean gateOpen = false;
    private boolean jogging = false;
    private boolean depollinate = false;
    private int gateTarget = GATE_CLOSED_POS;

    private boolean lastA = false, lastB = false, lastLeft = false;

    private long flywheelStartTime = 0;
    private long depolStart=0;

    private double peakVelocity = 0;
    private int intc=0;

    @Override
    public void init() {
        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);*/
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        LUmotor = hardwareMap.get(DcMotor.class, "LUmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");
        RUmotor = hardwareMap.get(DcMotor.class, "RUmotor");
        depollinator= hardwareMap.get(Servo.class, "depop");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // setup odometry
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        // update to actual values later if these are off
        // probably the most likely to be off right now
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RUmotor.setDirection(DcMotorSimple.Direction.REVERSE);


        DcMotor[] driveMotors = {LBmotor, LUmotor, RBmotor, RUmotor};
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(GATE_CLOSED_POS);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(GATE_POWER);

        telemetry.addLine("Gate must be CLOSED at init.");
        telemetry.addLine("DPAD UP/DOWN = jog gate, DPAD LEFT = set current gate pos as closed");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;

        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        /*
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //do smth with limelight
        } else {
            telemetry.addData("Limelight", "No Targets");
        }*/
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        LUmotor.setPower((y + x + rx) / denom);
        LBmotor.setPower((y - x + rx) / denom);
        RUmotor.setPower((y - x - rx) / denom);
        RBmotor.setPower((y + x - rx) / denom);
        odo.update();
        if(gamepad1.xWasPressed()){
            intc++;
        }
        double intakePower = 0.0;
        if (intc%3==1) {
            intakePower = INTAKE_POWER;
        }else if (intc%3==0){
            intakePower=0.0;
        }else if(intc%3==2){
            intakePower=-INTAKE_POWER;
        }
        intake.setPower(intakePower);

        boolean a = gamepad1.a;
        if (a && !lastA) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                flywheelStartTime = System.currentTimeMillis();
                peakVelocity = 0;
            }
        }
        lastA = a;
        if(gamepad1.yWasPressed()){
            depollinate=!depollinate;
        }
        outtake.setPower(flywheelOn ? FLYWHEEL_POWER : 0);

        double vel = Math.abs(outtake.getVelocity());
        if (flywheelOn && vel > peakVelocity) peakVelocity = vel;
        boolean flywheelReady = flywheelOn && isFlywheelAtSpeed(vel);

        boolean b = gamepad1.b;
        if (b && !lastB) {
            if (gateOpen) {
                gateOpen = false;
                gateTarget = GATE_CLOSED_POS;
            } else if (flywheelReady) {
                gateOpen = true;
                gateTarget = GATE_OPEN_POS;
            }
        }
        lastB = b;

        if (!flywheelOn && gateOpen) {
            gateOpen = false;
            gateTarget = GATE_CLOSED_POS;
        }

        boolean left = gamepad1.dpad_left;
        if (left && !lastLeft) {
            transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gateOpen = false;
            gateTarget = GATE_CLOSED_POS;
            jogging = false;
        }
        lastLeft = left;

        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            jogging = true;
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setPower(gamepad1.dpad_up ? GATE_JOG_POWER : -GATE_JOG_POWER);
        } else {
            if (jogging) {
                jogging = false;
                gateOpen = false;
                gateTarget = transfer.getCurrentPosition();
            }
            transfer.setTargetPosition(gateTarget);
            transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            transfer.setPower(GATE_POWER);
        }
        if(depollinate){
            depollinator.setPosition(0.5);
            depolStart = System.currentTimeMillis();
            depollinate=false;
        }

        if((System.currentTimeMillis()-depolStart)>DEPOL_SPINUP_TIME){
            depollinator.setPosition(0.0);
            depolStart=0;
        }
        /*
        //range tester
        if(gamepad1.dpadLeftWasPressed()){depollinator.setPosition(depollinator.getPosition()+0.1);}
        if(gamepad1.dpadRightWasPressed()){depollinator.setPosition(depollinator.getPosition()-0.1);}*/
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Intake state", intc);
        telemetry.addData("Flywheel on / ready", "%b / %b", flywheelOn, flywheelReady);
        telemetry.addData("Flywheel vel / peak", "%.0f / %.0f", vel, peakVelocity);
        telemetry.addData("Gate open", gateOpen);
        telemetry.addData("Depollinator Position", depollinator.getPosition());
        telemetry.addData("Gate pos / target", "%d / %d",
                transfer.getCurrentPosition(), gateTarget);
        telemetry.addData("Motor power: ", "LU: %.2f LB: %.2f RU: %.2f RB: %.2f", LUmotor.getPower(), LBmotor.getPower(), RUmotor.getPower(), RBmotor.getPower());
        telemetry.update();
    }

    private boolean isFlywheelAtSpeed(double vel) {
        long elapsed = System.currentTimeMillis() - flywheelStartTime;
        if (elapsed >= FLYWHEEL_MAX_SPINUP_MS) return true;
        if (elapsed < FLYWHEEL_MIN_SPINUP_MS) return false;
        if (FLYWHEEL_READY_VELOCITY > 0) return vel >= FLYWHEEL_READY_VELOCITY;
        return peakVelocity > 0 && vel >= 0.95 * peakVelocity;
    }

    @Override
    public void stop() {
        LBmotor.setPower(0);
        LUmotor.setPower(0);
        RBmotor.setPower(0);
        RUmotor.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
        transfer.setPower(0);
        depollinator.setPosition(0);
    }

    private double rpm_to_tps(double rpm) {
        int ticks_per_round = 28;
        return rpm * ticks_per_round / 60;
    }
}

/*


Right Trigger - Intake (A to reverse)
Left Trigger - Outtake (scaled by trigger value) & Transfer
X - Intake (1 press forward, next is back, next is stop)
Y - Depolinatte
B - Gate
A- Fire, press again to cancel/stop
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