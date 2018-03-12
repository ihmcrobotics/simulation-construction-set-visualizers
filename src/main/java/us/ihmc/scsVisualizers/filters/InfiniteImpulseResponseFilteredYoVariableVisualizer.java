package us.ihmc.scsVisualizers.filters;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable.ButterworthFilterType;
import us.ihmc.robotics.math.filters.InfiniteImpulseResponseFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class InfiniteImpulseResponseFilteredYoVariableVisualizer
{
   public static void main(String[] args)
   {
      final double DT = 0.0025;
      final int simulationTicksPerControlTick = 1;

      Robot nullRobot = new Robot("Nullbot");

      TestIIRFilterController testIIRFilterController = new TestIIRFilterController(nullRobot, DT, "testIIRFilterController");
      nullRobot.setController(testIIRFilterController, simulationTicksPerControlTick);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot, parameters);

      scs.setDT(DT, 1);

      scs.setupGraphGroup("IIR Filter", new String[][] { { "input", "alphaFilterOutput", "alphaFilterTestOutput" },
            { "input", "butterworthOneFilterOutput", "butterworthOneFilterTestOutput" }, { "input", "highPass", "highPassInverse" } });

      scs.setupGraphGroup("Nonlinear Filter", new String[][] { { "input", "highPassInverse" }, { "highPass" }, { "input", "doubleFiltered" },
            { "input", "lowPass" } });

      //    scs.createNewGraphWindow("IIR Filter", 1, true);
      scs.createNewGraphWindow("Nonlinear Filter", 1, true);

      Thread thread = new Thread(scs);
      thread.start();

   }

   private static class TestIIRFilterController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("TestIIRFilterController");

      private final YoDouble input = new YoDouble("input", registry);

      private final YoDouble chirp = new YoDouble("chirp", registry);
      private final YoDouble square = new YoDouble("square", registry);
      private final YoDouble oneHertz = new YoDouble("oneHertz", registry);
      private final YoDouble threeHertz = new YoDouble("threeHertz", registry);
      private final YoDouble sixHertz = new YoDouble("sixHertz", registry);
      private final YoDouble twelveHertz = new YoDouble("twelveHertz", registry);

      private final YoDouble t;

      private final AlphaFilteredYoVariable alphaFilterTestOutput = new AlphaFilteredYoVariable("alphaFilterTestOutput", registry, 0.95, input);
      private final ButterworthFilteredYoVariable butterworthOneFilterTestOutput = new ButterworthFilteredYoVariable("butterworthOneFilterTestOutput",
            registry, 0.95, input, ButterworthFilterType.LOW_PASS);
      private final InfiniteImpulseResponseFilteredYoVariable alphaFilterOutput = new InfiniteImpulseResponseFilteredYoVariable("alphaFilterOutput", 1, 0,
            registry);
      private final InfiniteImpulseResponseFilteredYoVariable butterworthOneFilterOutput = new InfiniteImpulseResponseFilteredYoVariable(
            "butterworthOneFilterOutput", 1, 1, registry);

      private final InfiniteImpulseResponseFilteredYoVariable lowPass = new InfiniteImpulseResponseFilteredYoVariable("lowPass", 4, 4, registry);

      private final InfiniteImpulseResponseFilteredYoVariable highPass = new InfiniteImpulseResponseFilteredYoVariable("highPass", 2, 2, registry);
      private final InfiniteImpulseResponseFilteredYoVariable highPassInverse = new InfiniteImpulseResponseFilteredYoVariable("highPassInverse", 2, 2, registry);
      private final YoDouble saturation = new YoDouble("saturation", registry);
      private final YoDouble amplitude = new YoDouble("amplitude", registry);

      private final YoDouble alphaButterworth = new YoDouble("alphaButterworth", registry);
      private final ButterworthFilteredYoVariable doubleFilteredOutput = new ButterworthFilteredYoVariable("doubleFiltered", registry, alphaButterworth,
            highPassInverse, ButterworthFilterType.LOW_PASS);

      private final YoDouble maxRate;
      private final RateLimitedYoVariable rateLimitFilterd;

      private String name;

      public TestIIRFilterController(Robot robot, double DT, String name)
      {
         this.name = name;
         maxRate = new YoDouble("maxRate", registry);
         rateLimitFilterd = new RateLimitedYoVariable("rateLimitFilter", registry, maxRate, input, DT);

         maxRate.set(0.5);

         this.t = (YoDouble) robot.getVariable("t");

         double alpha = 0.95;

         alphaFilterOutput.setPolesAndZeros(1.0 - alpha, new double[] { alpha }, null, null, null);
         butterworthOneFilterOutput.setPolesAndZeros(0.5 * (1.0 - alpha), new double[] { alpha }, null, new double[] { -1.0 }, null);

         //       double alphaOne = 0.9;
         //       double alphaTwo = 0.98;

         //       highPass.setPolesAndZeros(1.0, null, new double[]{alphaOne, alphaOne, alphaOne, alphaOne}, null, new double[]{alphaTwo, alphaTwo, alphaTwo, alphaTwo});
         //       highPassInverse.setPolesAndZeros(1.0, null, new double[]{alphaTwo, alphaTwo, alphaTwo, alphaTwo}, null, new double[]{alphaOne, alphaOne, alphaOne, alphaOne});

         //       double a1 = 0.99, b1 = 0.02; // Low Frequency
         //       double a1 = 0.975, b1 = 0.0; // Low Frequency
         double a1 = 0.98, b1 = 0.02; // Low Frequency

         //       double a1 = 0.98, b1 = 0.03; // Low Frequency
         //       double a1 = 0.99, b1 = 0.0; // Low Frequency

         //       double a2 = 0.8, b2 = 0.2; // High Frequency
         double a2 = 0.85, b2 = 0.15; // High Frequency

         //       double a2 = 0.2, b2 = 0.0; // High Frequency
         //       double a2 = 0.6, b2 = 0.4; // High Frequency
         //       double a2 = 0.90, b2 = 0.05; // High Frequency
         //       double a2 = 0.96, b2 = 0.05;  //

         ComplexNumber[] complexZeroPairs = new ComplexNumber[] { new ComplexNumber(a1, b1) }; // Low frequency.
         ComplexNumber[] complexPolePairs = new ComplexNumber[] { new ComplexNumber(a2, b2) }; // High frequency.

         double gainAtZero = (1.0 - 2.0 * a1 + a1 * a1 + b1 * b1) / (1.0 - 2.0 * a2 + a2 * a2 + b2 * b2);

         highPass.setPolesAndZeros(1.0 / gainAtZero, null, complexPolePairs, null, complexZeroPairs);
         highPassInverse.setPolesAndZeros(gainAtZero, null, complexZeroPairs, null, complexPolePairs);

         //       double a1LowPass = 0.92, b1LowPass = 0.08; // Low Frequency
         double a1LowPass = 0.92, b1LowPass = 0.04; // Low Frequency
         ComplexNumber[] lowPassComplexPolPairs = new ComplexNumber[] { new ComplexNumber(a1LowPass, b1LowPass), new ComplexNumber(a1LowPass, b1LowPass) }; // Low frequency.
         lowPass.setPolesAndZeros(null, lowPassComplexPolPairs, new double[] { -1.0, -1.0, -1.0, -1.0 }, null);

         saturation.set(0.3);
         amplitude.set(0.3);

         alphaButterworth.set(0.9);
      }

      public void doControl()
      {
         double time = t.getDoubleValue() % 5.0;
         chirp.set(amplitude.getDoubleValue() * Math.sin(4.0 * 2.0 * Math.PI * time * time));

         oneHertz.set(amplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * 1.0 * t.getDoubleValue()));
         threeHertz.set(amplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * 3.0 * t.getDoubleValue()));
         sixHertz.set(amplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * 6.0 * t.getDoubleValue()));
         twelveHertz.set(amplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * 12.0 * t.getDoubleValue()));

         square.set(amplitude.getDoubleValue() * Math.sin(2.0 * Math.PI * 1.0 * time));
         if (square.getDoubleValue() > 0.0)
            square.set(amplitude.getDoubleValue());
         if (square.getDoubleValue() < 0.0)
            square.set(amplitude.getDoubleValue());

         if (t.getDoubleValue() < 2.0)
            input.set(oneHertz.getDoubleValue());
         else if (t.getDoubleValue() < 4.0)
            input.set(threeHertz.getDoubleValue());
         else if (t.getDoubleValue() < 6.0)
            input.set(sixHertz.getDoubleValue());
         else if (t.getDoubleValue() < 8.0)
            input.set(twelveHertz.getDoubleValue());
         else if (t.getDoubleValue() < 10.0)
            input.set(square.getDoubleValue());
         else
            input.set(chirp.getDoubleValue());

         alphaFilterOutput.update(input.getDoubleValue());
         butterworthOneFilterOutput.update(input.getDoubleValue());

         alphaFilterTestOutput.update();
         butterworthOneFilterTestOutput.update();

         lowPass.update(input.getDoubleValue());
         highPass.update(input.getDoubleValue());

         double saturatedHighPass = highPass.getDoubleValue();
         if (saturatedHighPass > saturation.getDoubleValue())
            saturatedHighPass = saturation.getDoubleValue();
         if (saturatedHighPass < -saturation.getDoubleValue())
            saturatedHighPass = -saturation.getDoubleValue();

         highPassInverse.update(saturatedHighPass);
         doubleFilteredOutput.update();

         rateLimitFilterd.update();
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
   }
}
