package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commonWalkingControlModules.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class CirclePoseTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame aFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("aFrame", worldFrame, TransformTools.createTransformFromTranslationAndEulerAngles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);;

   private final CirclePoseTrajectoryGenerator traj;

   private final YoFramePoint position = new YoFramePoint("position", aFrame, registry);
   private final YoFrameVector velocity = new YoFrameVector("velocity", aFrame, registry);
   private final YoFrameVector acceleration = new YoFrameVector("acceleration", aFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private final Random random = new Random(46561L);

   public CirclePoseTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      String namePrefix = "traj";
      ReferenceFrame trajectoryFrame = aFrame;
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      traj = new CirclePoseTrajectoryGenerator(namePrefix, trajectoryFrame, trajectoryTimeProvider, registry, yoGraphicsListRegistry);
      FramePoint3D circleCenter = new FramePoint3D(aFrame, 1.0, 0.0, 0.3);
      FrameVector3D circleNormal = new FrameVector3D(aFrame, 0.0, -1.0, 1.0);//RandomTools.generateRandomFrameVector(random, aFrame);
      circleNormal.normalize();
      traj.updateCircleFrame(circleCenter, circleNormal);
      ReferenceFrame circleFrame = traj.getCircleFrame();
      FramePose3D initialPose = new FramePose3D(circleFrame);
      initialPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      initialPose.setPosition(-0.5, 0.0, 0.0);
      traj.setInitialPose(initialPose);
      traj.setDesiredRotationAngle(Math.PI);
      traj.initialize();
      traj.showVisualization();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Graphics3DObject circleFrameGraphics = new Graphics3DObject();
      circleFrameGraphics.transform(circleFrame.getTransformToWorldFrame());
      circleFrameGraphics.addCoordinateSystem(0.2);
      scs.addStaticLinkGraphics(circleFrameGraphics);

      Graphics3DObject circleNormalGraphics = new Graphics3DObject();
      circleNormalGraphics.translate(circleCenter);
      circleNormalGraphics.rotate(EuclidGeometryTools.axisAngleFromZUpToVector3D(circleNormal));
      circleNormalGraphics.addArrow(0.4, YoAppearance.Black(), YoAppearance.Black());
      scs.addStaticLinkGraphics(circleNormalGraphics);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);
         traj.getPosition(tempPoint);
         tempPoint.changeFrame(position.getReferenceFrame());
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
      new CirclePoseTrajectoryGeneratorVisualizer();
   }
}
