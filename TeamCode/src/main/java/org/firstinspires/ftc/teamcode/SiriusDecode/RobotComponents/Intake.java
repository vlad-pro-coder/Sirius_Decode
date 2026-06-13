package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;


import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;

public class Intake {


    public static CRServoPlus spinner;
    public Intake(){

    }

    public void takeIn(){
        spinner.setPower(1);
    }
    public void takeIn(double power){
        spinner.setPower(Math.abs(power));
    }

    public void spit(){
        spinner.setPower(-1);
    }
    public void spit(double power){
        spinner.setPower(-Math.abs(power));
    }

    public void stop(){
        spinner.setPower(0);
    }
}
