package org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(name = "CRServoPlus", xmlTag = "CrservoPlus")
public class CRServoPlus extends ServoImpl implements Servo, HardwareDevice {
    public CRServoPlus(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public CRServoPlus(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }

    public CRServoPlus(Servo s) {
        super(s.getController(), s.getPortNumber(), s.getDirection());

    }

    public synchronized void setPower(double val){
        setPosition((val+1.0)/2.0);
    }


}