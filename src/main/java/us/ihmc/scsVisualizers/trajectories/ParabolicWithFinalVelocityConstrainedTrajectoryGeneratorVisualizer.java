package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class ParabolicWithFinalVelocityConstrainedTrajectoryGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final double trajectoryTime = 0.3;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator traj;
   private final ParabolicCartesianTrajectoryGenerator parabolicCartesianTrajectoryGenerator;
   private final YoFramePoint3D trajPosition = new YoFramePoint3D("position", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition trajPositionViz = new YoGraphicPosition("trajPosition", trajPosition, 0.02,  YoAppearance.Blue(), GraphicType.BALL_WITH_CROSS);

   private final YoFramePoint3D trajPositionCartesean = new YoFramePoint3D("positionCartesean", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition trajPositionCarteseanViz = new YoGraphicPosition("positionCartesean", trajPositionCartesean, 0.02,  YoAppearance.Red(), GraphicType.BALL_WITH_CROSS);

   public ParabolicWithFinalVelocityConstrainedTrajectoryGeneratorVisualizer()
   {
      
      traj = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator("traj", ReferenceFrame.getWorldFrame(), registry);
      
      FramePoint3D initialPosition = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      double swingHeight = 0.05;
      FramePoint3D finalPosition = new FramePoint3D(worldFrame, 0.1,0.0,0.0);
      FrameVector3D finalVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
      traj.setTrajectoryParameters(trajectoryTime, initialPosition, swingHeight, finalPosition, finalVelocity);
      
      traj.initialize();

      //This is for comparison method
      DoubleProvider stepTimeProvider = new DoubleProvider()
      {
         @Override public double getValue()
         {
            return trajectoryTime;
         }
      };
      parabolicCartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("other", worldFrame, stepTimeProvider, swingHeight, registry);
      FrameVector3D zeroVector = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
      parabolicCartesianTrajectoryGenerator.initialize(initialPosition, zeroVector, zeroVector, finalPosition, finalVelocity);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
      yoGraphicsListRegistry.registerYoGraphic("traj3PositionViz", trajPositionViz);
      yoGraphicsListRegistry.registerArtifact("traj3PositionArtifact", trajPositionViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphic("trajCartPositionViz", trajPositionCarteseanViz);
      yoGraphicsListRegistry.registerArtifact("trajCartPositionArtifact", trajPositionCarteseanViz.createArtifact());

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);


      FramePoint3D cartPosition = new FramePoint3D(worldFrame);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(dt);
         traj.get(trajPosition);

         parabolicCartesianTrajectoryGenerator.compute(t);
         parabolicCartesianTrajectoryGenerator.getPosition(cartPosition);
         trajPositionCartesean.set(cartPosition.getX(), cartPosition.getY(), cartPosition.getZ());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ParabolicWithFinalVelocityConstrainedTrajectoryGeneratorVisualizer();
   }
}
