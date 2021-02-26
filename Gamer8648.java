package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.ViewLogsActivity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name = "Gamer 8648", group = "LinearOpMode")
//@Disabled
public class Gamer8648 extends LinearOpMode {

    private ElapsedTime runtime      = new ElapsedTime();



    //New Motors Names
    private DcMotor flMotor          = null;
    private DcMotor rlMotor          = null;
    private DcMotor frMotor          = null;
    private DcMotor rrMotor          = null;

    //Control Hub
    private DcMotor shooter = null;
    private DcMotor llArmMotor = null;
    private DcMotor rrArmMotor = null;

    //New Servos
    private Servo rg2 = null;
    private Servo rg1 = null;
    private Servo at2 = null;
    private Servo plunger = null;
    private Servo rtAim = null;


    //old stuff
    private DcMotor armUP            = null;
    private DcMotor slider           = null;

    private Servo hRight;
    private Servo hLeft;

    private Servo armMain;
    private Servo armHold;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*ltPower       = hardwareMap.get(DcMotor.class, "motortl");
        lbPower       = hardwareMap.get(DcMotor.class, "motorbl");
        rtPower       = hardwareMap.get(DcMotor.class, "motortr");
        rbPower       = hardwareMap.get(DcMotor.class, "motorbr");*/


        flMotor       = hardwareMap.get(DcMotor.class, "flMotor");
        rlMotor       = hardwareMap.get(DcMotor.class, "rlMotor");
        frMotor       = hardwareMap.get(DcMotor.class, "frMotor");
        rrMotor       = hardwareMap.get(DcMotor.class, "rrMotor");

        shooter       = hardwareMap.get(DcMotor.class, "shooter");
        llArmMotor       = hardwareMap.get(DcMotor.class, "LLArmMotor");
        rrArmMotor       = hardwareMap.get(DcMotor.class, "RRArmMotor");





        rg2        = hardwareMap.get(Servo.class, "RG2");
        rg1        = hardwareMap.get(Servo.class, "RG1");
        at2       = hardwareMap.get(Servo.class, "AT2");
        plunger       = hardwareMap.get(Servo.class, "Plunger");
        rtAim       = hardwareMap.get(Servo.class, "RTAim");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /*ltPower.setDirection(DcMotor.Direction.FORWARD);
        lbPower.setDirection(DcMotor.Direction.FORWARD);
        rtPower.setDirection(DcMotor.Direction.REVERSE);
        rbPower.setDirection(DcMotor.Direction.REVERSE);*/

        flMotor.setDirection(DcMotor.Direction.FORWARD);
        rlMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.REVERSE);





        plunger.setPosition(0);
        plunger.setPosition(1);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            double leftBackPower;
            double rightBackPower;

            double leftFrontPower;
            double rightFrontPower;

            double  drive            = -gamepad1.left_stick_y;
            double  turn             = gamepad1.right_stick_x;
            boolean armUpControl     = gamepad2.left_bumper;
            boolean armDownConroler  = gamepad2.right_bumper;
            boolean sliderPushControl= gamepad2.y;
            boolean sliderPushReturn = gamepad2.b;
            boolean armUse           = gamepad2.x;
            double strafeRight  = gamepad1.left_trigger;
            double strafeLeft   = gamepad1.right_trigger;
            boolean driverBringDown = gamepad1.a;

            ////////////////////////////////////////////////////////////////////////////////////////
            //                                                                                    //
            //STRAFING LEFT                                                                       //
            //                                                                                    //
            ////////////////////////////////////////////////////////////////////////////////////////
            if (strafeLeft == 1.0) {

                flMotor.setPower(1);
                rlMotor.setPower(-1);
                frMotor.setPower(-1);
                rrMotor.setPower(1);

            }else{

                flMotor.setPower(0);
                rlMotor.setPower(0);
                frMotor.setPower(0);
                rrMotor.setPower(0);

            }

            ////////////////////////////////////////////////////////////////////////////////////////
            //                                                                                    //
            //STRAFING RIGHT                                                                      //
            //                                                                                    //
            ////////////////////////////////////////////////////////////////////////////////////////
            if (strafeRight == 1.0) {

                flMotor.setPower(-1);
                rlMotor.setPower(1);
                frMotor.setPower(1);
                rrMotor.setPower(-1);

            }

            if(armUpControl){

                armUP.setPower(1);

            }else{

                armUP.setPower(0);
            }

            if (armDownConroler) {

                armUP.setPower(-1);

            }else{

                armUP.setPower(0);

            }

            if (sliderPushControl) {

                slider.setPower(-1);

            }else{

                slider.setPower(0);

            }

            if (sliderPushReturn) {

                slider.setPower(1);

            }else{

                slider.setPower(0);

            }
            if (armUse) {


                //armMain.setPosition(1);
                //armMain.setPosition(2);
                //sleep(500);
                //armHold.setPosition(0);
                at2.setPosition(1);



            }else{

                at2.setPosition(0);
                //armHold.setPosition(1);
                //sleep(2000);
                //armMain.setPosition(0);


            }


            if(driverBringDown){
                rg1.setPosition(1);
                rg2.setPosition(0);
            }
            else{
                rg1.setPosition(0);
                rg2.setPosition(1);
            }



            leftFrontPower   = Range.clip(drive + turn, -1.0, 1.0) ;
            leftBackPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightFrontPower  = Range.clip(drive - turn, -1.0, 1.0);
            rightBackPower   = Range.clip(drive - turn, -1.0, 1.0);

            flMotor.setPower(leftFrontPower);
            rlMotor.setPower(leftBackPower);
            frMotor.setPower(rightBackPower);
            rrMotor.setPower(rightFrontPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower, rightFrontPower, leftFrontPower);

        }

    }

}
