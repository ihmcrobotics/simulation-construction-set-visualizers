package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VelocityConstrainedPositionTrajectoryGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", new Random(1654151L), worldFrame);

   private final VelocityConstrainedPositionTrajectoryGenerator traj1;
   private final VelocityConstrainedPositionTrajectoryGenerator traj2;

   public VelocityConstrainedPositionTrajectoryGeneratorVisualizer()
   {
      traj1 = new VelocityConstrainedPositionTrajectoryGenerator("traj1", worldFrame, registry);
      traj2 = new VelocityConstrainedPositionTrajectoryGenerator("traj2", worldFrame, registry);

      FramePoint3D initialPosition = new FramePoint3D(worldFrame, 0.0, 0.0, 0.5);
      FrameVector3D initialVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
      FramePoint3D finalPosition = new FramePoint3D(worldFrame, 1.0, 0.0, 0.5);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, 0.1, 0.0, -1.0);
      traj1.setTrajectoryParameters(trajectoryTime, initialPosition, initialVelocity, finalPosition, finalVelocity);
      traj2.setTrajectoryParameters(trajectoryTime, initialPosition, initialVelocity, finalPosition, finalVelocity);

      traj1.initialize();
      traj2.initialize();
      traj2.changeFrame(aFrame);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);

      BagOfBalls balls1 = new BagOfBalls(bufferSize, 0.03, "trajBalls1", YoAppearance.AliceBlue(), registry, yoGraphicsListRegistry);
      BagOfBalls balls2 = new BagOfBalls(bufferSize, 0.03, "trajBalls2", YoAppearance.DarkGoldenRod(), registry, yoGraphicsListRegistry);


      YoGraphicReferenceFrame aFrameViz = new YoGraphicReferenceFrame(aFrame, registry, true, 0.3);
      aFrameViz.update();
      yoGraphicsListRegistry.registerYoGraphic("aFrame", aFrameViz);
//      yoGraphicsListRegistry.registerYoGraphic("traj3PositionViz", traj3PositionViz);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj1.compute(t);
         traj2.compute(t);

         FramePoint3D temp = new FramePoint3D();
         traj1.getPosition(temp);
         temp.changeFrame(worldFrame);
         balls1.setBall(temp);

         traj2.getPosition(temp);
         temp.changeFrame(worldFrame);
         balls2.setBall(temp);

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new VelocityConstrainedPositionTrajectoryGeneratorVisualizer();
   }
}
