package us.ihmc.robotics.math.filters;

import java.util.Random;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class IntegratorBiasCompensatorYoVariableVisualizer
{
   public IntegratorBiasCompensatorYoVariableVisualizer()
   {
      double dt = 0.001;
      Robot robot = new Robot("dummy");
      robot.setController(new FilterController(robot.getYoTime(), dt));
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, new SimulationConstructionSetParameters(true, 65536));
      scs.setDT(dt, 1);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.hideViewport();
      scs.startOnAThread();
   }

   private static class FilterController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final YoDouble sourceAngle = new YoDouble("sourceAngle", registry);
      private final YoDouble sourceAngleRate = new YoDouble("sourceAngleRate", registry);
      private final YoDouble sourceFrequency = new YoDouble("sourceFrequency", registry);
      private final YoDouble sourceAmplitude = new YoDouble("sourceAmplitude", registry);
      private final YoDouble sourcePosition = new YoDouble("sourcePosition", registry);
      private final YoDouble sourceRate = new YoDouble("sourceRate", registry);

      private final YoDouble kp = new YoDouble("kp", registry);
      private final YoDouble ki = new YoDouble("ki", registry);
      private final YoDouble positionGaussianNoise = new YoDouble("positionGaussianNoise", registry);
      private final YoDouble rateBias = new YoDouble("rateBias", registry);

      private final YoDouble filterPositionInput = new YoDouble("filterPositionInput", registry);
      private final YoDouble filterRateInput = new YoDouble("filterRateInput", registry);
      private final IntegratorBiasCompensatorYoVariable filter;

      private final YoDouble filterPositionOutputError = new YoDouble("filterPositionOutputError", registry);
      private final YoDouble filterRateOutputError = new YoDouble("filterRateOutputError", registry);

      private final double dt;

      public FilterController(DoubleProvider time, double dt)
      {
         this.dt = dt;
         sourceAmplitude.set(10.0);
         sourceFrequency.set(1.0);

         positionGaussianNoise.set(1.0e-3);
         rateBias.set(0.1);

         kp.set(0.1);
         ki.set(0.01);
         filter = new IntegratorBiasCompensatorYoVariable("filter", registry, kp, ki, filterPositionInput, filterRateInput, dt);
      }

      @Override
      public void initialize()
      {
      }

      private final Random random = new Random();

      @Override
      public void doControl()
      {
         sourceAngleRate.set(2.0 * Math.PI * sourceFrequency.getValue());
         sourceAngle.set(AngleTools.shiftAngleToStartOfRange(sourceAngle.getValue() + sourceAngleRate.getValue() * dt, 0.0));
         sourcePosition.set(sourceAmplitude.getValue() * Math.sin(sourceAngle.getValue()));
         sourceRate.set(sourceAngleRate.getValue() * sourceAmplitude.getValue() * Math.cos(sourceAngle.getValue()));

         filterPositionInput.set(sourcePosition.getValue() + positionGaussianNoise.getValue() * random.nextGaussian());
         filterRateInput.set(sourceRate.getValue() + rateBias.getValue());
         filter.update(filterPositionInput.getValue(), filterRateInput.getValue());

         filterPositionOutputError.set(sourcePosition.getValue() - filter.getValue());
         filterRateOutputError.set(sourceRate.getValue() - filter.getRateEstimation().getValue());
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }

   public static void main(String[] args)
   {
      new IntegratorBiasCompensatorYoVariableVisualizer();
   }
}
