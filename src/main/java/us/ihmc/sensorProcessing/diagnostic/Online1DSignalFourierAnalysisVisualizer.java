package us.ihmc.sensorProcessing.diagnostic;

import java.util.Random;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class Online1DSignalFourierAnalysisVisualizer
{

   private static final Random random = new Random(6541651L);
   private static final int BUFFER = 10000;

   public Online1DSignalFourierAnalysisVisualizer()
   {
      double dt = 0.001;
      YoVariableRegistry registry = new YoVariableRegistry("blup");
      Robot robot = new Robot("Dummy");
      YoDouble yoTime = robot.getYoTime();

      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("signal", yoTime, registry);
      functionGenerator.setChirpFrequencyMaxHz(40.0);
      functionGenerator.setFrequency(10.0);
      functionGenerator.setAmplitude(1.0);
      functionGenerator.setResetTime(BUFFER * dt);
      functionGenerator.setMode(YoFunctionGeneratorMode.SINE);

      Online1DSignalFourierAnalysis online1dSignalFrequencyAnalysis = new Online1DSignalFourierAnalysis("blop", 1.0, dt, registry);

      YoDouble signal = new YoDouble("signal", registry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, BUFFER);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.hideViewport();
      scs.startOnAThread();


      for (int i = 0; i < BUFFER; i++)
      {
         signal.set(functionGenerator.getValue());
         signal.add(0.5 * (random.nextDouble() - 0.5) + 0.0);
         signal.add(0.3 * Math.sin(2.0 * Math.PI * 16.0 * yoTime.getDoubleValue() + 0.33 * Math.PI));

         online1dSignalFrequencyAnalysis.update(signal.getDoubleValue());

         yoTime.add(dt);
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new Online1DSignalFourierAnalysisVisualizer();
   }
}
