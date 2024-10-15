package us.ihmc.scsVisualizers.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.AccelerationLimitedYoVariable;
import us.ihmc.robotics.math.filters.FilteredFiniteDifferenceYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;

public class AccelerationLimitedYoVariableVisualizer
{
   private static final double maxRate = Double.POSITIVE_INFINITY; //0.75;
   private static final double maxAcceleration = Double.POSITIVE_INFINITY; //10.0;
   
   private static final double zeta = 1.0;
   private static final double w0 = 2.0 * Math.PI * 16.0; //maxAcceleration / (zeta * maxRate);
   
   
   private static final double dt = 0.001;
   private static final double totalTime = 15.0;

   private final YoDouble raw;
   private final YoDouble alphaVariable;
   private final FilteredFiniteDifferenceYoVariable rawRate;
   private final FilteredFiniteDifferenceYoVariable rawAcceleration;

   private static double amplitude = 2.0;
   private static double frequency = 0.1;

   private static final int bufferSize = (int) (totalTime / dt) + 5; //make sure it fits in the graph
   private final Robot robot = new Robot("generic_robot");
   private String nameYo = "processed";
   private SimulationConstructionSet scs;
   private YoRegistry registry;

   public AccelerationLimitedYoVariableVisualizer()
   {
      registry = new YoRegistry("generic_registry");

      raw = new YoDouble("raw", registry);
      alphaVariable = new YoDouble("alpha", registry);
      alphaVariable.set(0.0); //set to zero to prevent smoothing of velocity
      rawRate = new FilteredFiniteDifferenceYoVariable("rawRate", "", alphaVariable, raw, dt, registry);
      rawAcceleration = new FilteredFiniteDifferenceYoVariable("rawAcceleration", "", alphaVariable, rawRate, dt, registry);

      scs = new SimulationConstructionSet(robot);
      scs.changeBufferSize(bufferSize);
      scs.getRootRegistry().addChild(registry);

      YoDouble maxRateYo = new YoDouble("max_Rate", registry);
      maxRateYo.set(maxRate);
      YoDouble maxAccelerationYo = new YoDouble("max_Acceleration", registry);
      maxAccelerationYo.set(maxAcceleration);
      AccelerationLimitedYoVariable processed = new AccelerationLimitedYoVariable(nameYo, registry, maxRateYo, maxAccelerationYo, dt);
      processed.setGainsByPolePlacement(w0, zeta);

      YoDouble timeYo = new YoDouble("time", registry);
      YoFunctionGenerator squareFunction = new YoFunctionGenerator("sineFunction", timeYo, registry);
      squareFunction.setChirpFrequencyMaxHz(20.0);
      squareFunction.setResetTime(20.0);
      squareFunction.setMode(YoFunctionGeneratorMode.SQUARE);
      squareFunction.setFrequency(frequency);
      squareFunction.setAmplitude(amplitude);

      double value = 0.0;

      for (double time = 0.0; time < totalTime; time += dt)
      {
         timeYo.set(time);
         value = squareFunction.getValue();

         raw.set(value);
         rawRate.update(raw.getDoubleValue());
         rawAcceleration.update(rawRate.getDoubleValue());
         processed.update(raw.getDoubleValue());
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
   
   public static void main(String[] args)
   {
      new AccelerationLimitedYoVariableVisualizer();
   }
}
