package us.ihmc.scsVisualizers.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class FiniteDifferenceAngularVelocityYoFrameVectorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double dt = 0.001;
   private static final int BUFFER = 10000;

   public FiniteDifferenceAngularVelocityYoFrameVectorVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("blop");
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
      scs.addYoVariableRegistry(registry);
      scs.hideViewport();
      scs.startOnAThread();

      FrameQuaternion orientation = new FrameQuaternion();

      for (int i = 0; i < BUFFER ; i++)
      {
         traj.compute(yoTime.getDoubleValue());
         traj.getOrientation(orientation);
         orientationToDifferentiate.set(orientation);
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
