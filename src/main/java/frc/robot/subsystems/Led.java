package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private int j = 0;
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
        
        if(i>14){
            i=0;
        }
        else{
            i++;
            j=i;
        }


        for(int a=0;a<4;a++)
        {
            for(int p=0;p<15;p++){
                ledBuffer.setRGB((p+i)%15 + (a*15),200,200 + (p*15),0 );
            }
        }

        


    }
    @Override
    public void periodic()
    {
        timing += 1;
        if(timing % 4 == 0)
        {
            red();
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

