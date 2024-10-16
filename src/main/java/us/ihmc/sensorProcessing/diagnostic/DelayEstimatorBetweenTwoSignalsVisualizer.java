package us.ihmc.sensorProcessing.diagnostic;

import java.util.Random;

import us.ihmc.yoVariables.filters.DelayedYoDouble;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class DelayEstimatorBetweenTwoSignalsVisualizer
{
   private static final Random random = new Random(51651L);
   private static final int BUFFER = 10000;
   private static final double dt = 0.001;

   public DelayEstimatorBetweenTwoSignalsVisualizer()
   {
      YoRegistry registry = new YoRegistry("Blop");
      Robot robot = new Robot("Dummy");
      YoDouble yoTime = robot.getYoTime();
      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("foo", yoTime, registry);
      functionGenerator.setChirpFrequencyMaxHz(40.0);
      functionGenerator.setAmplitude(1.0);
      functionGenerator.setResetTime(BUFFER * dt);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      YoDouble referenceSignal = new YoDouble("referenceSignal", registry);
      DelayedYoDouble delayedSignal = new DelayedYoDouble("delayedSignal", "", referenceSignal, 12, registry);
//      AlphaFilteredYoVariable delayedSignal = new AlphaFilteredYoVariable("delayedSignal", registry, 0.9, referenceSignal);
      YoDouble noisyDelayedSignal = new YoDouble("noisyDelayedSignal", registry);
      DelayEstimatorBetweenTwoSignals delayEstimatorBetweenTwoSignals = new DelayEstimatorBetweenTwoSignals(noisyDelayedSignal.getName(), referenceSignal, noisyDelayedSignal, dt, registry);
      delayEstimatorBetweenTwoSignals.enable();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, BUFFER);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.startOnAThread();

      for (int i = 0; i < BUFFER; i++)
      {
//         double sine = functionGenerator.getValue();
         double sine = 0.5 * Math.sin(2.0 * Math.PI * 16.0 * i * dt);
//         double sineDelayed = 1.0 * Math.sin(2.0 * Math.PI / 100 * i - 0.5 * Math.PI / 2.0) + 0.5*new Random().nextDouble();
         referenceSignal.set(sine);
         delayedSignal.update();
         noisyDelayedSignal.set(delayedSignal.getDoubleValue());
         noisyDelayedSignal.add(0.5 * (random.nextDouble() - 0.5) + 1.5);
         delayEstimatorBetweenTwoSignals.update();

         yoTime.add(dt);
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new DelayEstimatorBetweenTwoSignalsVisualizer();
   }
}
