package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Minibot", group="Linear Opmode")
public class Minibot extends LinearOpMode {


    static final double     TICKS_PER_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    /*static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;*/

    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor intake = null;
    private DcMotor launcher1 = null;
    private DcMotor arm = null;
    private CRServo extendo =  null;
    private Servo claw = null;
    private Servo blocker = null;
    // private DcMotor launcher2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setDirection(DcMotor.Direction.FORWARD);

        arm =  hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        extendo =  hardwareMap.get(CRServo.class, "extendo");
        extendo.setDirection(CRServo.Direction.FORWARD);

        claw = hardwareMap.get(Servo.class, "claw");

        blocker = hardwareMap.get(Servo.class, "blocker");



       /* launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setDirection(DcMotor.Direction.FORWARD);*/

        back_left.setPower(0);
        back_right.setPower(0);
        intake.setPower(0);
        launcher1.setPower(0);
        arm.setPower(0);
        extendo.setPower(0);
        claw.setPosition(0);
        blocker.setPosition(0);

        //launcher2.setPower(0);


        waitForStart();


        while (opModeIsActive()) {


            /*left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

            double leftPower;
            double rightPower;

            double drive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            back_left.setPower(leftPower);
            back_right.setPower(rightPower);

            if (gamepad1.a) {
                launcher1.setPower(-0.9);
               // launcher2.setPower(0.7);
            } else if (gamepad1.b) {
                launcher1.setPower(0);
               // launcher2.setPower(0);
            }

            if (gamepad1.x) {
                intake.setPower(0.6);
            } else if (gamepad1.y) {
                intake.setPower(0);
            }

            if (gamepad1.dpad_up) {
                arm.setPower(-0.7);
            } else if (gamepad1.dpad_down){
                arm.setPower(0.7);
            } else {
                arm.setPower(0);
            }

            extendo.setPower(gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {
                claw.setPosition(1);
            } else if (gamepad1.left_bumper) {
                claw.setPosition(0);
            }



        }

    }

}
