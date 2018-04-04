package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commonWalkingControlModules.trajectories.FinalApproachPositionTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FinalApproachPositionTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame aFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("aFrame", worldFrame, TransformTools.createTransformFromTranslationAndEulerAngles(0.0, 0.0, 0.5, 1.57, 0.0, 0.0));
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);;

   private final FinalApproachPositionTrajectoryGenerator traj;

   private final YoFramePoint position = new YoFramePoint("position", aFrame, registry);
   private final YoFrameVector velocity = new YoFrameVector("velocity", aFrame, registry);
   private final YoFrameVector acceleration = new YoFrameVector("acceleration", aFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public FinalApproachPositionTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      traj = new FinalApproachPositionTrajectoryGenerator("Traj", aFrame, registry, true, yoGraphicsListRegistry);
      traj.setInitialPosition(0.0, 0.0, 1.0);
      traj.setFinalPosition(1.0, 0.0, 1.0);
      traj.setFinalApproach(new FrameVector3D(aFrame, 0.0, 1.0, 1.0), 0.1);
      traj.setTrajectoryTime(trajectoryTime);
      traj.initialize();
      traj.showVisualization();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
//      yoGraphicsListRegistry.addYoGraphicsObjectListsToSimulationConstructionSet(scs, true);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);
         traj.getPosition(tempPoint);
         position.set(tempPoint);
         traj.getVelocity(tempVector);
         velocity.set(tempVector);
         traj.getAcceleration(tempVector);
         acceleration.set(tempVector);

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new FinalApproachPositionTrajectoryGeneratorVisualizer();
   }
}
