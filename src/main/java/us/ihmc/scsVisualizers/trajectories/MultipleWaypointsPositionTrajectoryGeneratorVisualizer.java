package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.math.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MultipleWaypointsPositionTrajectoryGeneratorVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 5.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MultipleWaypointsPositionTrajectoryGenerator traj;
   private final StraightLinePoseTrajectoryGenerator simpleTraj;

   public MultipleWaypointsPositionTrajectoryGeneratorVisualizer()
   {
      FramePoint3D initialPosition = new FramePoint3D(worldFrame, 1.0, 0.0, 1.0);
      FramePoint3D finalPosition = new FramePoint3D(worldFrame, 0.2, 1.0, 0.4);
      simpleTraj = new StraightLinePoseTrajectoryGenerator("simpleTraj",
                                                               worldFrame,
                                                               registry);
      simpleTraj.setInitialPose(new FramePose3D(initialPosition, new FrameQuaternion()));
      simpleTraj.setFinalPose(new FramePose3D(finalPosition, new FrameQuaternion()));
      simpleTraj.setTrajectoryTime(trajectoryTime);
      simpleTraj.initialize();

      traj = new MultipleWaypointsPositionTrajectoryGenerator("testedTraj", 200, worldFrame, registry);
      traj.clear();

      int numberOfWaypoints = 11;

      FramePoint3D waypointPosition = new FramePoint3D();
      FrameVector3D waypointVelocity = new FrameVector3D();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         waypointPosition.setIncludingFrame(simpleTraj.getPosition());
         waypointVelocity.setIncludingFrame(simpleTraj.getVelocity());
         traj.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      }
      traj.initialize();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);
         simpleTraj.compute(t);

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new MultipleWaypointsPositionTrajectoryGeneratorVisualizer();
   }
}
