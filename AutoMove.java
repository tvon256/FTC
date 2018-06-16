package org.firstinspires.ftc.teamcode;

public class AutoMove extends Movement {

    public AutoMove(){

    }

    Hardware robot = new Hardware();

    public void forward(double power, int direction){
        robot.leftDrive.setPower(power*direction);
        robot.rightDrive.setPower(power*direction);
    }

}
