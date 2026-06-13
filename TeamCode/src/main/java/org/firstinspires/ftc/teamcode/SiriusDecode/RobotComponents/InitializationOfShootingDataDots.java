package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

public class InitializationOfShootingDataDots {

    public static ShooterCalculatorWithDots calc;
    public static void InitDots(){

        calc = new ShooterCalculatorWithDots(Turret.goalPosition.x,Turret.goalPosition.y,3200,3200,100,-1,-1,4);

        calc.setParameters(4180,  3300, 36.0, 0.008);
        calc.setParameters(4176,  3200, 37.0, 0.004);
        calc.setParameters(4172,  3050, 37.0, 0.008);
        calc.setParameters(4168,  3000, 37.0, 0.008);
        calc.setParameters(4164,  3100, 36.0, 0.0);
        calc.setParameters(4160,  3100, 36.0, 0.003);
        calc.setParameters(4156,  3100, 36.0, 0.005);
        calc.setParameters(4152,  3050, 36.0, 0.008);
        calc.setParameters(4148,  3100, 36.0, 0.00);
        calc.setParameters(4144,  3000, 36.0, 0.005);
        calc.setParameters(4140,  2880, 36.0, 0.013);
        calc.setParameters(4136,  2800, 38.0, 0.008);
        calc.setParameters(4132,  2800, 38.0, 0.007);
        calc.setParameters(4044,  3100, 37.0, 0.005);
        calc.setParameters(4040,  3000, 37.0, 0.001);
        calc.setParameters(4036,  2900, 37.0, 0.002);
        calc.setParameters(4032,  2950, 38.0, 0.005);
        calc.setParameters(4028,  3000, 36.0, 0.002);
        calc.setParameters(4024,  2950, 37.0, 0.006);
        calc.setParameters(4020,  3000, 36.0, 0);
        calc.setParameters(4016,  3000, 36.0, 0);
        calc.setParameters(4012,  3000, 36.0, 0.005);
        calc.setParameters(4008,  2900, 36.0, 0.005);
        calc.setParameters(3908,  3200, 36.0, 0);
        calc.setParameters(3904,  3000, 36.0, 0.007);
        calc.setParameters(3900,  3000, 36.0, 0.003);
        calc.setParameters(3896,  3000, 36.0, 0.004);
        calc.setParameters(3888,  2900, 36.0, 0.005);
        calc.setParameters(3884,  2850, 37.0, 0.006);
        calc.setParameters(3880,  2800, 37.0, 0.008);
        calc.setParameters(3760,  3100, 36.0, 0.00);
        calc.setParameters(3764,  3050, 36.0, 0);
        calc.setParameters(3768,  3050, 36.0, 0.003);
        calc.setParameters(3772,  2900, 38.0, 0.009);
        calc.setParameters(3756,  2850, 36.0, 0.004);
        calc.setParameters(3636,  2850, 37.0, 0.01);
        calc.setParameters(3632,  2850, 38.0, 0.002);
        calc.setParameters(3628,  2850, 38.0, 0.002);
        calc.setParameters(3624,  2800, 37.0, 0);
    }
}
