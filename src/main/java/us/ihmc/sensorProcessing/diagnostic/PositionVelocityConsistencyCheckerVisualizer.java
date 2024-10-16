package us.ihmc.sensorProcessing.diagnostic;

import java.util.Random;

import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.filters.DelayedYoDouble;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class PositionVelocityConsistencyCheckerVisualizer
{
   private static final Random random = new Random(6541651L);
   private static final int BUFFER = 100000;
   private static final double ENCODER_RESOLUTION = 2.0 * Math.PI / (Math.pow(2, 9) - 1.0);

   public PositionVelocityConsistencyCheckerVisualizer()
   {
      YoRegistry registry = new YoRegistry("blop");
      double dt = 0.001;
      Robot robot = new Robot("Dummy");

      YoDouble yoTime = robot.getYoTime();
      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("signal", yoTime, registry);
      functionGenerator.setChirpFrequencyMaxHz(40.0);
      functionGenerator.setAmplitude(1.0);
      functionGenerator.setResetTime(BUFFER * dt);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);
      YoDouble positionSignal = new YoDouble("positionSignal", registry);
      YoDouble digitizedPositionSignal = new YoDouble("digitizedPositionSignal", registry);
      double alpha = AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(16.0, dt);
      AlphaFilteredYoVariable processedPositionSignal = new AlphaFilteredYoVariable("processedPositionSignal", registry, alpha, digitizedPositionSignal);
      YoDouble velocitySignal = new YoDouble("velocitySignal", registry);
      DelayedYoDouble delayedVelocitySignal = new DelayedYoDouble("delayedVelocitySignal", "", velocitySignal, 9, registry);
      YoDouble noisyVelocitySignal = new YoDouble("velocitySignalToCheck", registry);
      AlphaFilteredYoVariable processedVelocitySignal = new AlphaFilteredYoVariable("processedVelocitySignal", registry, alpha, noisyVelocitySignal);

      PositionVelocity1DConsistencyChecker positionVelocityConsistencyChecker = new PositionVelocity1DConsistencyChecker("signal", digitizedPositionSignal,
            noisyVelocitySignal, processedPositionSignal, processedVelocitySignal, dt, registry);
      positionVelocityConsistencyChecker.enable();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, BUFFER);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.hideViewport();
      scs.startOnAThread();

      for (int i = 0; i < BUFFER; i++)
      {
         positionSignal.set(functionGenerator.getValue());
         positionSignal.add(0.5 * (random.nextDouble() - 0.5) + 0.0);
         int nTicks = (int) ((positionSignal.getDoubleValue() + Math.PI) / ENCODER_RESOLUTION);
         digitizedPositionSignal.set(nTicks * ENCODER_RESOLUTION - Math.PI);
         velocitySignal.set(functionGenerator.getValueDot());
         delayedVelocitySignal.update();

         noisyVelocitySignal.set(delayedVelocitySignal.getDoubleValue());
         noisyVelocitySignal.add(50.0 * (random.nextDouble() - 0.5) + 5.5);

         processedPositionSignal.update();
         processedVelocitySignal.update();

         positionVelocityConsistencyChecker.update();

         yoTime.add(dt);
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new PositionVelocityConsistencyCheckerVisualizer();
   }
}
