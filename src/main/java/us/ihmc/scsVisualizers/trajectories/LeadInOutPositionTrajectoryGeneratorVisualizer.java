package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commonWalkingControlModules.trajectories.LeadInOutPositionTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class LeadInOutPositionTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame aFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("aFrame", worldFrame, TransformTools.createTransformFromTranslationAndEulerAngles(0.0, 0.0, 0.5, 1.57, 0.0, 0.0));
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final LeadInOutPositionTrajectoryGenerator traj;

   private final YoFramePoint3D position = new YoFramePoint3D("position", worldFrame, registry);
   private final YoFrameVector3D velocity = new YoFrameVector3D("velocity", worldFrame, registry);
   private final YoFrameVector3D acceleration = new YoFrameVector3D("acceleration", worldFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public LeadInOutPositionTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      traj = new LeadInOutPositionTrajectoryGenerator("Traj", worldFrame, registry, true, yoGraphicsListRegistry);
      traj.setInitialLeadOut(new FramePoint3D(worldFrame, 0.0, 0.0, 0.5), new FrameVector3D(worldFrame, 0.0, 0.0, 1.0), 0.1);
      traj.setFinalLeadIn(new FramePoint3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, -0.0, 0.0, 1.0), 0.1);
      traj.setTrajectoryTime(trajectoryTime);
      traj.initialize();
      traj.showVisualization();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
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
      new LeadInOutPositionTrajectoryGeneratorVisualizer();
   }
}
