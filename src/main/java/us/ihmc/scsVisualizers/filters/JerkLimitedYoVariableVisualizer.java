package us.ihmc.scsVisualizers.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.JerkLimitedYoDouble;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class JerkLimitedYoVariableVisualizer
{
   private final SimulationConstructionSet scs;
   
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   
   private final YoDouble time = new YoDouble("time", registry);
   private final YoDouble maxAcceleration = new YoDouble("maxAcceleration", registry);
   private final YoDouble maxJerk = new YoDouble("maxJerk", registry);
   private final YoDouble inputPosition = new YoDouble("inputPosition", registry);
   private final YoDouble inputVelocity = new YoDouble("inputVelocity", registry);
   private final YoDouble inputAcceleration = new YoDouble("inputAcceleration", registry);

   double dt = 0.006;
   JerkLimitedYoDouble smoothedYoVariable = new JerkLimitedYoDouble("smoothedPosition", registry, maxAcceleration, maxJerk, inputPosition, inputVelocity,
                                                                    inputAcceleration, dt);

   public static void main(String[] args)
   {
      new JerkLimitedYoVariableVisualizer();
   }

   // Really more of an evaluator than a test case. Need to turn it into a few good test cases.
   public JerkLimitedYoVariableVisualizer()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(32000);
      scs = new SimulationConstructionSet(new Robot("test"), parameters);
      
      scs.addYoRegistry(registry);
      scs.setDT(0.0001, 10);
      scs.hideViewport();

      scs.startOnAThread();

      double w0 = 2.0 * Math.PI * 0.3;
      double w1 = 2.0 * Math.PI * 0.3;
      double zeta = 1.0;

      smoothedYoVariable.setGainsByPolePlacement(w0, w1, zeta);
      smoothedYoVariable.setMaximumAcceleration(0.8 * 9.81);
      smoothedYoVariable.setMaximumJerk(20.0 * 9.81);

      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("functionGenerator", registry);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      functionGenerator.setChirpFrequencyMaxHz(0.5);
      functionGenerator.setResetTime(60.0);

      double maximumOmega0 = maxAcceleration.getDoubleValue() / maxJerk.getDoubleValue();
      System.out.println("maximumOmega0 = " + maximumOmega0 + " = " + maximumOmega0 / (2.0 * Math.PI) + " hertz.");

      smoothedYoVariable.update(0.0);

      double amplitude = 0.1;
      inputPosition.set(amplitude);
      functionGenerator.setAmplitude(amplitude);

      inputVelocity.set(0.0);
      inputAcceleration.set(0.0);
      update();
      inputPosition.set(0.0);

      inputVelocity.set(0.1);
      update();
      inputVelocity.set(0.0);

      inputAcceleration.set(0.1);
      update();
      inputAcceleration.set(0.0);

      for (double localTime = 0.0; localTime < 5.0; localTime += dt)
      {
         smoothedYoVariable.update();
         time.add(dt);
         scs.tickAndUpdate();
      }

      for (double localTime = 0.0; localTime < 60.0; localTime += dt)
      {
         inputPosition.set(functionGenerator.getValue(localTime));
         inputVelocity.set(functionGenerator.getValueDot());

         smoothedYoVariable.update();
         time.add(dt);
         scs.tickAndUpdate();
      }

      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   private void update()
   {
      for (double localTime = 0.0; localTime < 1.0; localTime += dt)
      {
         smoothedYoVariable.update();
         time.add(dt);
         scs.tickAndUpdate();
      }
   }
}
