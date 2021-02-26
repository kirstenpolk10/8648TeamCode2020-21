package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



//@Autonomous(name="Chassis 8648", group="Pushbot")
//@Disabled
public class Chassis8648 extends LinearOpMode {

    private DcMotor flMotor          = null;
    private DcMotor frMotor          = null;
    private DcMotor rlMotor          = null;
    private DcMotor rrMotor          = null;

    @Override
    public void runOpMode() {

        flMotor       = hardwareMap.get(DcMotor.class, "FLmotor");
        rlMotor       = hardwareMap.get(DcMotor.class, "FRmotor");
        frMotor       = hardwareMap.get(DcMotor.class, "RLmotor");
        rrMotor       = hardwareMap.get(DcMotor.class, "RRmotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        rlMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        rrMotor.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        //Go left
        flMotor.setPower(.5);
        rlMotor.setPower(.5);
        frMotor.setPower(.5);
        rrMotor.setPower(.5);
        sleep(500);


    }


    }
