package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.BitSet;




/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */





/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Team8648HardwarePushbot
{
    /* Public OpMode members. */
    //public DcMotor leftDrive   = null;
    //public DcMotor  rightDrive  = null;

    //f- front
    //r - rear
    //l - left
    //r - right

    public DcMotor flMotor          = null;
    public DcMotor rlMotor          = null;
    public DcMotor frMotor          = null;
    public DcMotor rrMotor          = null;

    public DcMotor shooter = null;
    public DcMotor bicepMotor = null;
    public DcMotor conveyorMotor = null;





    public enum WobbleTargetZone {
        RED_A,
        RED_B,
        RED_C,
        BLUE_A,
        BLUE_B,
        BLUE_C

    }


    /*public DcMotor  leftArm     = null;
    public Servo leftClaw    = null;
    public Servo    rightClaw   = null;

    */



    public Servo claw = null;

    //public Servo rtAim = null;
    //public CRServo slider = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double SHOOTER    =  0.80;
    public static final double _POWER = 0.40;
    public static final double CONVEYOR = -0.80;








    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Team8648HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        flMotor  = hwMap.get(DcMotor.class, "FLmotor");
        rlMotor = hwMap.get(DcMotor.class, "RLmotor");
        frMotor  = hwMap.get(DcMotor.class, "FRmotor");
        rrMotor = hwMap.get(DcMotor.class, "RRmotor");

        shooter = hwMap.get(DcMotor.class, "Shooter");
        bicepMotor = hwMap.get(DcMotor.class, "BicepMotor");
        conveyorMotor = hwMap.get(DcMotor.class, "ConveyorMotor");



        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        flMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rlMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rrMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        flMotor.setPower(0);
        rlMotor.setPower(0);
        frMotor.setPower(0);
        rrMotor.setPower(0);

        shooter.setPower(0);
        bicepMotor.setPower(0);
        conveyorMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bicepMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bicepMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //bicepMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bicepMotor.setDirection(DcMotor.Direction.FORWARD);







        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        /*leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/


        //slider = hwMap.get(CRServo.class, "slider");
        //rtAim = hwMap.get(Servo.class, "RTaim");
        claw = hwMap.get(Servo.class, "claw");

        //slider.setPower(0);
        claw.setPosition(MID_SERVO);
        //rtAim.setPosition(MID_SERVO);








    }
}
