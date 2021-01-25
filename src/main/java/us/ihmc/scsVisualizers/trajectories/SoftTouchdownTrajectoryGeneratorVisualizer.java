package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SoftTouchdownTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 10.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final SoftTouchdownPositionTrajectoryGenerator traj;

   private final YoFramePoint3D position = new YoFramePoint3D("position", worldFrame, registry);
   private final YoFrameVector3D velocity = new YoFrameVector3D("velocity", worldFrame, registry);
   private final YoFrameVector3D acceleration = new YoFrameVector3D("acceleration", worldFrame, registry);

   public SoftTouchdownTrajectoryGeneratorVisualizer()
   {
      traj = new SoftTouchdownPositionTrajectoryGenerator("Traj", registry);
      traj.setLinearTrajectory(0, new FramePoint3D(worldFrame, 1.0, 1.0, 1.0), new FrameVector3D(worldFrame, 0.0, 0.01, -0.1), new FrameVector3D(worldFrame, 0.0001, 0.0, -0.01));
      traj.initialize();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);
         position.set(traj.getPosition());
         velocity.set(traj.getVelocity());
         acceleration.set(traj.getAcceleration());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new SoftTouchdownTrajectoryGeneratorVisualizer();
   }
}
