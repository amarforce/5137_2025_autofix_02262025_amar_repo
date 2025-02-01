package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;
import frc.robot.other.ImprovedNoise;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

@SuppressWarnings("unused")
public class LED extends SubsystemBase
{
    private int i = 0;
    private double x = 0;
    private int y = 0;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDSim ledSim;
    private int timing = 0;
    public LED() {

        led = new AddressableLED(LedConstants.LEDPort);
        ledBuffer = new AddressableLEDBuffer(LedConstants.LEDStrip);

        ledSim = new AddressableLEDSim(led);
        ledSim.setRunning(true);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void red()
    {
        i++;
        for(int a=0;a<LedConstants.LEDSections;a++)
        {
            for(int p=0;p<LedConstants.LEDStrip/LedConstants.LEDSections;p++)
            {
                ledBuffer.setRGB((p+i)%(LedConstants.LEDStrip/LedConstants.LEDSections) + (a*LedConstants.LEDStrip/LedConstants.LEDSections),200,200 + (p*15),0 );
            }
        }
    }
    public void green()
    {
        for(int a=0;a<LedConstants.LEDStrip;a++)
        {
            
            ledBuffer.setRGB(a, 0, 255, 0);
            
        }
    }
    public void blue()
    {
        i++;
        for(int a=0;a<LedConstants.LEDSections;a++)
        {
            for(int p=0;p<LedConstants.LEDStrip/LedConstants.LEDSections;p++)
            {
                ledBuffer.setRGB((p+i)%(LedConstants.LEDStrip/LedConstants.LEDSections) + (a*LedConstants.LEDStrip/LedConstants.LEDSections),0, 0, 250-(p*15));
            }
        }
    }
    public void fire()
    {
        x++;
        if (x>50)
        {
            x = 0;
        }
        if (-1<x && x<9 || (41<x && x<50))
        {
            x = x / 10;
            y = (int)((-(x-2.5)*(x-2.5)+6.25)*(LedConstants.LEDLength/(20/3)));
            x = x * 10;
        }
        if(9<x && x<41)
        {
            x = x / 10;
            y = (int)((1*(Math.sin(5*x))+ 3.5)*(LedConstants.LEDLength/(20/3)));
            x = x * 10;
        }
        for(int p=0;p<LedConstants.LEDLength-y;p++)
        {
            ledBuffer.setRGB(p,255, 240 - (int)(p*(240/(LedConstants.LEDLength-y))),100);
            ledBuffer.setRGB(LedConstants.LEDStrip-1-p,255, 240 - (int)(p*(240/(LedConstants.LEDLength-y))),100);
        }
        for (int cat = LedConstants.LEDLength-y;cat<LedConstants.LEDLength;cat++){
            ledBuffer.setRGB(cat,0, 0,0);
            ledBuffer.setRGB(LedConstants.LEDStrip-1-cat,0, 0,0);
        }
    }

    public void betterFire()
    {
            int y= (int)ImprovedNoise.noise(3.14,42,7);
            for(int p=0;p<LedConstants.LEDLength-y;p++)
        {
            ledBuffer.setRGB(p,255, 240 - (int)(p*(240/(LedConstants.LEDLength-y))),100);
            ledBuffer.setRGB(LedConstants.LEDStrip-1-p,255, 240 - (int)(p*(240/(LedConstants.LEDLength-y))),100);
        }
        for (int cat = LedConstants.LEDLength-y;cat<LedConstants.LEDLength;cat++){
            ledBuffer.setRGB(cat,0, 0,0);
            ledBuffer.setRGB(LedConstants.LEDStrip-1-cat,0, 0,0);
        }
    }
    

    @Override
    public void periodic()
    {
        
        byte[] dataConvert = new byte[LedConstants.LEDStrip*4];
        for(int i= 0; i<LedConstants.LEDStrip; i++)
        {
            dataConvert[i*4] = (byte)(ledBuffer.getBlue(i));
            dataConvert[i*4+1] = (byte)(ledBuffer.getGreen(i));
            dataConvert[i*4+2] = (byte)(ledBuffer.getRed(i));

        }
        led.setData(ledBuffer);
        ledSim.setData(dataConvert);
    }

    @Override
    public void simulationPeriodic()
    {
        timing += 1;
        if(timing % 4 == 0)
        {
            betterFire();
        }
    }
}

