package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LED extends SubsystemBase
{
    private int i = 0;
    private double x = 0;
    private int y = 0;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView ledLeft;
    private AddressableLEDBufferView ledRight;
    private AddressableLEDSim ledSim;
    private int timing = 0;
    {

        led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(60);

        ledLeft = ledBuffer.createView(0, 29);
        ledRight = ledBuffer.createView(30, 59).reversed();

        ledSim = new AddressableLEDSim(led);
        ledSim.setRunning(true);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void setReverse(LEDPattern pattern)
    {
        
        pattern.reversed().applyTo(ledLeft);
        pattern.reversed().applyTo(ledRight);
    }

    public void set(LEDPattern pattern)
    {
        pattern.applyTo(ledLeft);
        pattern.applyTo(ledRight);
    }

    public void red()
    {
        i++;
        for(int a=0;a<6;a++)
        {
            for(int p=0;p<10;p++)
            {
                ledBuffer.setRGB((p+i)%10 + (a*10),200,200 + (p*15),0 );
            }
        }
    }
    public void green()
    {
        for(int a=0;a<60;a++)
        {
            
            ledBuffer.setRGB(a, 0, 255, 0);
            
        }
    }
    public void mlue()
    {
        i++;
        for(int a=0;a<6;a++)
        {
            for(int p=0;p<10;p++)
            {
                ledBuffer.setRGB((p+i)%10 + (a*10),0, 0, 250-(p*15));
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
            ledBuffer.setRGB(59-p,255, 240 - (int)(p*(240/(LedConstants.LEDLength-y))),100);
        }
        for (int cat = LedConstants.LEDLength-y;cat<LedConstants.LEDLength;cat++){
            ledBuffer.setRGB(cat,0, 0,0);
            ledBuffer.setRGB(59-cat,0, 0,0);
        }


    };

    @Override
    public void periodic()
    {
        timing += 1;
        if(timing % 4 == 0)
        {
            fire();
        }
        byte[] dataConvert = new byte[240];
        for(int i= 0; i<60; i++)
        {
            dataConvert[i*4] = (byte)(ledBuffer.getBlue(i));
            dataConvert[i*4+1] = (byte)(ledBuffer.getGreen(i));
            dataConvert[i*4+2] = (byte)(ledBuffer.getRed(i));

        }
        led.setData(ledBuffer);
        ledSim.setData(dataConvert);
    }

                                                                                                                                                                                                                                                            };;;;;;;

