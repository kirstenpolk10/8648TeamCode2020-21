package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="Team 8648 Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class Team8648PushbotTeleopTank extends OpMode {
    /* Declare OpMode members. */
    Team8648HardwarePushbot robot       = new Team8648HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          TablePosition = 0.0;
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    final double    TAB_SPEED = 0.01;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*public DcMotor flMotor          = null;
        public DcMotor rlMotor          = null;
        public DcMotor frMotor          = null;
        public DcMotor rrMotor          = null;*/



        double left;
        double right;

        double leftTrigger;
        double rightTrigger;





        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;


        //leftTrigger = -gamepad1.left_trigger;
        //rightTrigger = -gamepad1.right_trigger;


        //        if (gamepad1.right_trigger)
//            robot.sliderMotor.setPower(1);
//        else if (gamepad1.left_trigger)
//            robot.sliderMotor.setPower(0);





        //robot.leftDrive.setPower(left);
        //robot.rightDrive.setPower(right);

        robot.flMotor.setPower(right);
        robot.rlMotor.setPower(right);
        robot.frMotor.setPower(left);
        robot.rrMotor.setPower(left);

        //robot.sliderMotor.setPower(leftTrigger);
        //robot.sliderMotor.setPower(rightTrigger);



        // Use gamepad left & right Bumpers to open and close the claw
       // if (gamepad2.right_bumper)
         //   clawOffset += CLAW_SPEED;
       // else if (gamepad2.left_bumper)
          //  clawOffset -= CLAW_SPEED;

        if (gamepad1.right_bumper)
            robot.claw.setPosition(1);
        else
            robot.claw.setPosition(0);
        // Move both servos to new position.  Assume servos are mirror image of each other.
        //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        //robot.rg1.setPosition(robot.MID_SERVO + clawOffset);
        //robot.rg2.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        //if (gamepad2.dpad_up)
            //robot.llarmmotor.setPower(robot.ARM_UP_POWER);
        //else if (gamepad2.dpad_down)
            //robot.llarmmotor.setPower(robot.ARM_DOWN_POWER);
        //else
            //robot.llarmmotor.setPower(0.0);




        if(gamepad2.right_bumper)
            robot.shooter.setPower(robot.SHOOTER);
        else
            robot.shooter.setPower(0.0);

        if(gamepad2.a)
            robot.conveyorMotor.setPower(-0.80);
        else if (gamepad2.y)
            robot.conveyorMotor.setPower(0.80);
        else
            robot.conveyorMotor.setPower(0);


        //if(gamepad1.y)
            //robot.conveyorMotor.setPower(.80);
       // else
           // robot.conveyorMotor.setPower(0);

        /*if(gamepad1.a)
            robot.plunger.setPosition(1);
        else if(gamepad1.y)
            robot.plunger.setPosition(0);

        if(gamepad1.dpad_up)
            robot.rtAim.setPosition(1);
        else if(gamepad1.dpad_down)
            robot.rtAim.setPosition(0);

        /*if(gamepad2.dpad_left)
            robot.at2.setPower(1);
        else if(gamepad2.dpad_right)
            robot.at2.setPower(-1);
        else robot.at2.setPower(0);

       /* if(gamepad2.b)
            robot.rg1.setPosition(1);
        else if(gamepad2.x)
            robot.rg1.setPosition(0);


       /* if (gamepad2.y)
            robot.rg2.setPosition(1);
        else  if(gamepad2.a)
            robot.rg2.setPosition(0);



        if (gamepad1.dpad_right)
          robot.slider.setPower(1);
        else if (gamepad1.dpad_left)
        robot.slider.setPower(-1);
        else
          robot.slider.setPower(0.0);



       /* if(gamepad1.right_stick_y) {
            robot.bicepMotor.setPower(.80);
        else if(gamepad1.left_bumper)
            robot.bicepMotor.setPower(-.80);
        else robot.bicepMotor.setPower(0);

        robot.bicepMotor.setPower();*/

       // if (gamepad2.right_bumper)
            //robot.claw.setPosition(1);
       // else (gamepad2.left_bumper)
           // robot.claw.setPosition(0);


       //robot.bicepMotor.setPower(gamepad2.left_stick_y);

       if(gamepad1.dpad_down)
       robot.bicepMotor.setPower(1);
       else
           robot.bicepMotor.setPower(0);


      if(gamepad1.dpad_up)
           robot.bicepMotor.setPower(-1);
       else
           robot.bicepMotor.setPower(0);

       //robot.slider.setPower(gamepad2.left_stick_y);

       // if (gamepad1.x)
        //    robot.wc.setPosition(1);
        //else
            //robot.wc.setPosition(0);


        //if(gamepad2.dpad_left)
            //robot.at2.setPosition(0.5);
        //else if(gamepad2.dpad_right)
            //robot.at2.setPosition(-0.5);







        // Send telemetry message to signify robot running;

        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
