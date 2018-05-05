package us.ihmc.scsVisualizers.filters;

import us.ihmc.robotics.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.robotics.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.robotics.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;
import us.ihmc.simulationConstructionSet.util.RobotController;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FirstOrderFilteredYoVariableVisualizer
{
   public static void main(String[] args)
   {
      final double DT = 0.0025;
      final int simulationTicksPerControlTick = 1;

      Robot nullRobot = new Robot("Nullbot");

      FilterController testFilterController = new FilterController(nullRobot, DT, "testFilterController");
      nullRobot.setController(testFilterController, simulationTicksPerControlTick);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot, parameters);

      scs.setDT(DT, 1);

      scs.setupGraphGroup("High Pass Filter", new String[][] { { "filterInput", "highPass", "properAmplitudeHighPass" }, { "filterInput", "lowPass", "properAmplitudeLowPass" } , { "filterInput", "bandPass", "properAmplitudeBandPass" }});
      
      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs, true);
      sliderBoard.setSlider(0, "minFreq_RadPerSec", scs, 0.001, 15.0);
      sliderBoard.setSlider(1, "maxFreq_radPerSec", scs, 0.1, 20.0);
      sliderBoard.setSlider(2, "inputFrequency_radPerSec", scs, 0.1, 5.0);


      Thread thread = new Thread(scs);
      thread.start();
   }

   private static class FilterController implements RobotController
   {
      private String name;
      
      private final YoDouble t;

      private final YoVariableRegistry registry;

      private final YoDouble filterInput;

      private final YoDouble inputFrequency_radPerSec;

      private final YoDouble minPassThroughFreq_radPerSec;
      private final YoDouble maxPassThroughFreq_radPerSec;

      private final FirstOrderFilteredYoVariable highPass;
      private final FirstOrderFilteredYoVariable lowPass;
      private final FirstOrderBandPassFilteredYoVariable bandPass;
      
      private final YoDouble properAmplitudeHighPass;
      private final YoDouble properAmplitudeLowPass;
      private final YoDouble properAmplitudeBandPass;


      private final double DT;

      public FilterController(Robot robot, double DT, String name)
      {
         this.name = name;
         this.DT = DT;
         
         registry = new YoVariableRegistry("TestHighPassFilterController");

         filterInput = new YoDouble("filterInput", registry);

         inputFrequency_radPerSec = new YoDouble("inputFrequency_radPerSec", registry);
         inputFrequency_radPerSec.set(1.0);

         this.t = (YoDouble) robot.getVariable("t");

         this.minPassThroughFreq_radPerSec = new YoDouble("minFreq_RadPerSec", registry);
         minPassThroughFreq_radPerSec.set(1.0);
         
         this.maxPassThroughFreq_radPerSec = new YoDouble("maxFreq_radPerSec", registry);
         maxPassThroughFreq_radPerSec.set(100.0);
         
         this.highPass = new FirstOrderFilteredYoVariable("highPass", "", minPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI), robot.getYoTime(), FirstOrderFilterType.HIGH_PASS, registry);
         this.lowPass = new FirstOrderFilteredYoVariable("lowPass", "", maxPassThroughFreq_radPerSec.getDoubleValue()  / (2.0*Math.PI), robot.getYoTime(), FirstOrderFilterType.LOW_PASS, registry);
         this.bandPass = new FirstOrderBandPassFilteredYoVariable("bandPass", "", minPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI), maxPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI), robot.getYoTime(), registry);
         
         this.properAmplitudeHighPass = new YoDouble("properAmplitudeHighPass", registry);
         this.properAmplitudeLowPass = new YoDouble("properAmplitudeLowPass", registry);
         this.properAmplitudeBandPass = new YoDouble("properAmplitudeBandPass", registry);
      }

      public void doControl()
      {       
         filterInput.set( Math.sin(inputFrequency_radPerSec.getDoubleValue() * t.getDoubleValue()) );
         
         highPass.setCutoffFrequencyHz(minPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI));
         highPass.update(filterInput.getDoubleValue());
                 
         lowPass.setCutoffFrequencyHz(maxPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI));
         lowPass.update(filterInput.getDoubleValue());
                 
         bandPass.setPassBand(minPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI), maxPassThroughFreq_radPerSec.getDoubleValue() / (2.0*Math.PI));
         bandPass.update(filterInput.getDoubleValue());
         
         double a = minPassThroughFreq_radPerSec.getDoubleValue();      
         double b = maxPassThroughFreq_radPerSec.getDoubleValue();
         double omega = inputFrequency_radPerSec.getDoubleValue(); 
         properAmplitudeHighPass.set( omega / Math.sqrt( omega*omega + a*a ));
         properAmplitudeLowPass.set( b / Math.sqrt( omega*omega + b*b ));
         properAmplitudeBandPass.set(properAmplitudeHighPass.getDoubleValue() * properAmplitudeLowPass.getDoubleValue());
      }
      
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return this.name;
      }

      public void initialize()
      {
      }

      public String getDescription()
      {
         return getName();
      }
      
      
      private double computeRK4(double z, double u, double u_old)
      {
         double k1 = computeZdotHighPass(z, u, u_old);
         double k2 = computeZdotHighPass(z + DT/2*k1, u, u_old);
         double k3 = computeZdotHighPass(z + DT/2*k2, u, u_old);
         double k4 = computeZdotHighPass(z + DT*k3, u, u_old);
         
         double ret = z + DT/6 * (k1 + 2*k2 + 2*k3 + k4);
         
         return ret;
      }
      
      private double computeZdotHighPass(double z, double u, double u_old)
      {
         double ret = -minPassThroughFreq_radPerSec.getDoubleValue()*z  +  (u - u_old)/DT;
         
         return ret;
      }
   }
}
