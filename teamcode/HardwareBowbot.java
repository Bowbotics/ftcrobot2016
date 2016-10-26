package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Bowbot.
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
public class HardwareBowbot
{
    /* Public OpMode members. */
    public DcMotor  Left_Drive   = null;
    public DcMotor  Right_Drive  = null;
    public DcMotor  Sweeper_Motor    = null;
  //  public Servo    buttonpusher    = null;//


    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBowbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Left_Drive   = hwMap.dcMotor.get("Left_Drive");
        Right_Drive  = hwMap.dcMotor.get("Right_Drive");
        Sweeper_Motor    = hwMap.dcMotor.get("Sweeper_Motor");
        Left_Drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        Right_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        Left_Drive.setPower(0);
        Right_Drive.setPower(0);
        Sweeper_Motor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Sweeper_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
       // buttonpusher = hwMap.servo.get(buttonpusher);//
        //rightClaw = hwMap.servo.get("right_hand");//
      //  leftClaw.setPosition(MID_SERVO);//
        //rightClaw.setPosition(MID_SERVO);//
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     * ???????
     *  profit
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass. O.O
        period.reset();
    }
}

