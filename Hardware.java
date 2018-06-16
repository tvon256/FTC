package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    /*Define hardware variables to be used in Opmodes*/
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public BNO055IMU imu = null;

    /*Define variables that tells phone this class is a hardware class*/
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /*Constructor: Allows methods and variables from this class to be accessed in others*/
    public Hardware(){

    }
    /*Initialize Hardware from phone config and set it up how we want it*/
    public void init(HardwareMap ahwMap){
        //reads and stores phone config
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //Set motor runmodes
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Define Servos

        //Define Sensors
        imu = hwMap.get(BNO055IMU.class, "imu");    //must config as i2c port 0
    }
}
