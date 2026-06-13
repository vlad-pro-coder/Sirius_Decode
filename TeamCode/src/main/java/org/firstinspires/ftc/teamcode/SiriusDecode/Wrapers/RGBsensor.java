package org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.Colors;

@I2cDeviceType
@DeviceProperties(
        xmlTag = "RGBsensor",
        name = "RGB sensor - REV 3"
)
public class RGBsensor extends RevColorSensorV3 implements HardwareDevice {
    public class ColorRangeSensorPacket {
        public double R, G, B, A;
        public NormalizedRGBA colors;
        public double D;
        public ColorRangeSensorPacket(){
            R = G = B = 0;
            D = 0;
        }
        @Override
        public String toString(){
            return Double.toString(R) + ' ' + Double.toString(G) + ' ' + Double.toString(B);
        }
    }
    private long timeDistance = 0, timeRGB = 0;
    private double freq = 50;
    private double lowPassFilter = 1;
    public ColorRangeSensorPacket p = new ColorRangeSensorPacket();
    public ColorRangeSensorPacket RGB = new ColorRangeSensorPacket();
    public RGBsensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        changeLEDsettings(LEDPulseModulation.LED_PULSE_100kHz, LEDCurrent.CURRENT_5mA);
        timeDistance = System.currentTimeMillis();
        timeRGB = System.currentTimeMillis();
        setFreqToUpdate(50);
    }
    public void changeLEDsettings(LEDPulseModulation l, LEDCurrent c){
        setLEDParameters(l, c);
    }
    // in reads / s
    public void setFreqToUpdate(double x){
        freq =  x;
    }
    public void setLowPassFilterCoefficient(double k){
        this.lowPassFilter = k;
    }
    @Override
    public double getDistance(DistanceUnit unit){
        if(System.currentTimeMillis() - timeDistance > freq){
            p.D = unit.fromUnit(DistanceUnit.INCH, this.inFromOptical(this.rawOptical()));
            timeDistance = System.currentTimeMillis();
        }
        return p.D;
    }

    public Colors.ColorType getColorSeenBySensor(){
        try {
            if (System.currentTimeMillis() - timeRGB > freq) {
                RGB.colors = this.getNormalizedColors();
                float scale = 256.f;

                RGB.R = Range.clip(RGB.colors.red * scale, 0, 255);
                RGB.G = Range.clip(RGB.colors.green * scale, 0, 255);
                RGB.B = Range.clip(RGB.colors.blue * scale, 0, 255);
                RGB.D = getDistance(DistanceUnit.CM);

                timeRGB = System.currentTimeMillis();
            }
        } catch (Exception ignored){
            RobotLog.ii("cur mort de senzor","mort");
        }
        return Colors.getColorFromHoop(new Colors.Color(RGB.R, RGB.G, RGB.B, RGB.D));
    }
}