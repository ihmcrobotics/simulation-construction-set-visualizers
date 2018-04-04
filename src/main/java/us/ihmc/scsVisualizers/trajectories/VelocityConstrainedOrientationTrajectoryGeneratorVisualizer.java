package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VelocityConstrainedOrientationTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(4566561L);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 3.0;
   private final double dt = 0.0001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private FrameQuaternion orientationCurr = new FrameQuaternion();
   private FrameQuaternion orientationPrev = new FrameQuaternion();
   private Quaternion quatCurr = new Quaternion();
   private Quaternion quatPrev = new Quaternion();
   private Quaternion quatDelta = new Quaternion();
   private FrameVector3D angularVelocityCurr = new FrameVector3D();
   private FrameVector3D angularVelocityPrev = new FrameVector3D();
   double angle, omega;

   private final FiniteDifferenceAngularVelocityYoFrameVector FD2AngularVelocity = new FiniteDifferenceAngularVelocityYoFrameVector("FD2AngularVelocity", worldFrame, dt, registry);
   
   private final YoFrameVector FDAngularVelocity = new YoFrameVector("FDAngularVelocity", worldFrame, registry);
   private final YoDouble FDAngularVelocityMagnitude = new YoDouble("FDAngularVelocityMagnitude", registry);
   private final YoFrameVector FDAngularVelocityDirection = new YoFrameVector("FDAngularVelocityDirection", worldFrame, registry);
   private final YoFrameVector FDAngularAcceleration = new YoFrameVector("FDAngularAcceleration", worldFrame, registry);

   private final YoFrameVector trajAngularVelocityDirection = new YoFrameVector("trajAngularVelocityDirectionViz", worldFrame, registry);

   private final YoDouble initialDriftFullRotations = new YoDouble("initialDriftFullRotations", registry);
   private final YoDouble finalDriftFullRotations = new YoDouble("finalDriftFullRotations", registry);

   public VelocityConstrainedOrientationTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);

      initialOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      initialAngularVelocity.set(1.0, 0.0, 0.0);
      finalOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      finalAngularVelocity.set(0.0, 1.0, 0.0);

      VelocityConstrainedOrientationTrajectoryGenerator traj1 = new VelocityConstrainedOrientationTrajectoryGenerator("traj1", worldFrame, registry);
      traj1.setInitialConditions(initialOrientation, initialAngularVelocity);
      traj1.setFinalConditions(finalOrientation, finalAngularVelocity);
//      SimpleOrientationTrajectoryGenerator traj1 = new SimpleOrientationTrajectoryGenerator("traj1", worldFrame, registry);
//      traj1.setInitialOrientation(initialOrientation);
//      traj1.setFinalOrientation(finalOrientation);
      traj1.setTrajectoryTime(trajectoryTime);
      traj1.initialize();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      YoFramePose desiredOrientationViz = new YoFramePose("desiredOrientationViz", "viz", worldFrame, registry);
      YoFramePose orientationFromIntegrationViz = new YoFramePose("orientationFromIntegrationViz", "viz", worldFrame, registry);
//      pose.setZ(1.0);
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicCoordinateSystem("blopDes", desiredOrientationViz, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicCoordinateSystem("blopInt", orientationFromIntegrationViz, 1.0, YoAppearance.GreenYellow()));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicVector("blopDir", desiredOrientationViz.getPosition(), trajAngularVelocityDirection, YoAppearance.BlueViolet()));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicVector("blopDirCheck", desiredOrientationViz.getPosition(), FDAngularVelocityDirection, YoAppearance.Black()));

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
//      initialOrientationDrifted.set(initialOrientation);
//      finalOrientationDrifted.set(finalOrientation);

      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);
         traj1.compute(t);
         traj1.getOrientation(orientationCurr); 
         FD2AngularVelocity.update(quatCurr);

         desiredOrientationViz.setOrientation(orientationCurr);
         
         initialDriftFullRotations.set(initialAngularVelocity.length() * t / (2.0 * Math.PI));
         finalDriftFullRotations.set(finalAngularVelocity.length() * t / (2.0 * Math.PI));
         
         
         Point3D initialPosition = new Point3D();
         Point3D finalPosition = new Point3D();
         Point3D desiredPosition = new Point3D();

         desiredPosition.interpolate(initialPosition, finalPosition, t / trajectoryTime);
         desiredOrientationViz.setPosition(desiredPosition);

         if (t >= dt)
         {
            quatCurr.set(orientationCurr);
            quatPrev.set(orientationPrev);

            quatPrev.inverse();
            quatDelta.multiply(quatCurr, quatPrev);

            quatDelta.normalize();
            angle = AngleTools.trimAngleMinusPiToPi(Math.acos(quatDelta.getS()) * 2.0);
            omega = angle / (dt);
            angularVelocityCurr.setIncludingFrame(worldFrame, quatDelta.getX(), quatDelta.getY(), quatDelta.getZ());

            if (angularVelocityCurr.length() > 0.0)
            {
               angularVelocityCurr.normalize();
            }
            angularVelocityCurr.scale(omega);

            FDAngularVelocity.set(angularVelocityCurr);
            FDAngularVelocityMagnitude.set(FDAngularVelocity.length());
            if (FDAngularVelocityMagnitude.getDoubleValue() > 1.0e-7)
            {
               FDAngularVelocityDirection.set(FDAngularVelocity);
               FDAngularVelocityDirection.scale(1.0 / FDAngularVelocityMagnitude.getDoubleValue());
            }
         }

         FrameVector3D direction = new FrameVector3D();
         traj1.getAngularVelocity(direction);
         if (direction.length() < 1.0e-10)
            direction.set(0.0, 0.0, 0.0);
         else
            direction.normalize();
         trajAngularVelocityDirection.set(direction);

         if (t >= 1.9999 * dt)
         {
            FDAngularAcceleration.sub(angularVelocityCurr, angularVelocityPrev);
            FDAngularAcceleration.scale(1.0 / dt);
         }

         orientationPrev.set(orientationCurr);
         angularVelocityPrev.set(angularVelocityCurr);
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new VelocityConstrainedOrientationTrajectoryGeneratorVisualizer();
   }
}
