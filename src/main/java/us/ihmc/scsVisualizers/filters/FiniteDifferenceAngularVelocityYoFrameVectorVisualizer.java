package us.ihmc.scsVisualizers.filters;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FiniteDifferenceAngularVelocityYoFrameVectorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double dt = 0.001;
   private static final int BUFFER = 10000;

   public FiniteDifferenceAngularVelocityYoFrameVectorVisualizer()
   {
      YoRegistry registry = new YoRegistry("blop");
      SimpleOrientationTrajectoryGenerator traj = new SimpleOrientationTrajectoryGenerator("traj", worldFrame, registry);
      traj.setInitialOrientation(new FrameQuaternion());
      traj.setFinalOrientation(new FrameQuaternion(worldFrame, 0.8, -1.0, 2.1));
      traj.setTrajectoryTime(BUFFER * dt);
      traj.initialize();

      YoFrameQuaternion orientationToDifferentiate = new YoFrameQuaternion("orientationToDifferentiate", worldFrame, registry);
      FiniteDifferenceAngularVelocityYoFrameVector filteredAngularVelocityYoFrameVector = new FiniteDifferenceAngularVelocityYoFrameVector("angularVelocityFD", orientationToDifferentiate, dt, registry);

      Robot robot = new Robot("Dummy");
      YoDouble yoTime = robot.getYoTime();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, BUFFER);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.hideViewport();
      scs.startOnAThread();

      FrameQuaternion orientation = new FrameQuaternion();

      for (int i = 0; i < BUFFER ; i++)
      {
         traj.compute(yoTime.getDoubleValue());
         orientationToDifferentiate.set(traj.getOrientation());
         filteredAngularVelocityYoFrameVector.update();

         yoTime.add(dt);
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new FiniteDifferenceAngularVelocityYoFrameVectorVisualizer();
   }
}
