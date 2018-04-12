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
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class HermiteCurveBasedOrientationTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(4566561L);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 2.0;
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

   private final FiniteDifferenceAngularVelocityYoFrameVector FD2AngularVelocity = new FiniteDifferenceAngularVelocityYoFrameVector("FD2AngularVelocity",
                                                                                                                                    worldFrame, dt, registry);

   private final YoFrameVector3D FDAngularVelocity = new YoFrameVector3D("FDAngularVelocity", worldFrame, registry);
   private final YoDouble FDAngularVelocityMagnitude = new YoDouble("FDAngularVelocityMagnitude", registry);
   private final YoFrameVector3D FDAngularVelocityDirection = new YoFrameVector3D("FDAngularVelocityDirection", worldFrame, registry);
   private final YoFrameVector3D FDAngularAcceleration = new YoFrameVector3D("FDAngularAcceleration", worldFrame, registry);

   private final YoFrameVector3D trajAngularVelocityDirection = new YoFrameVector3D("trajAngularVelocityDirectionViz", worldFrame, registry);

   private final YoDouble initialDriftFullRotations = new YoDouble("initialDriftFullRotations", registry);
   private final YoDouble finalDriftFullRotations = new YoDouble("finalDriftFullRotations", registry);

   public HermiteCurveBasedOrientationTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);

      //      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, 1.0, 1.26540, -0.15600, 2.114);
      //      FrameVector initialAngularVelocity = new FrameVector(worldFrame, Math.sqrt(0)*Math.PI, -Math.sqrt(4)*Math.PI, Math.sqrt(30)*Math.PI);
      //      FrameOrientation finalOrientation = new FrameOrientation(worldFrame, 0, 0, 0, 0);
      //      FrameVector finalAngularVelocity = new FrameVector(worldFrame, Math.sqrt(0)*Math.PI, -Math.sqrt(4)*Math.PI, Math.sqrt(37)*Math.PI);

      //      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, -0.28384529974368705, 0.4872396085355831, 0.6585703700057955, 0.49831162683399);
      //      FrameVector initialAngularVelocity = new FrameVector(worldFrame, -3.3077237748627013, 5.0821487165957695, 3.505184333541667);
      //      FrameOrientation finalOrientation = new FrameOrientation(worldFrame, 0.5293423132777587, 0.6796170799867696, 0.2552631232628004, 0.43904222788489117);
      //      FrameVector finalAngularVelocity = new FrameVector(worldFrame, -5.025525775767217, -9.375274016759331, -4.5078148494826875);

      //                  initialOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      //                  initialAngularVelocity.set(0.0, 0.0, 0.0);
      //                  finalOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      //                  finalAngularVelocity.set(0.0, 1.0, 0.0);

      System.out.println("Initial angular velocity: " + initialAngularVelocity);
      System.out.println("Final   angular velocity: " + finalAngularVelocity);

      HermiteCurveBasedOrientationTrajectoryGenerator traj1 = new HermiteCurveBasedOrientationTrajectoryGenerator("traj1", worldFrame, registry);
      traj1.setInitialConditions(initialOrientation, initialAngularVelocity);
      traj1.setFinalConditions(finalOrientation, finalAngularVelocity);
      traj1.setTrajectoryTime(trajectoryTime);
      traj1.setNumberOfRevolutions(0);
      traj1.initialize();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      Point3D initialPosition = new Point3D(-1.0, 0.0, 0.5);
      Point3D finalPosition = new Point3D(1.0, 0.0, 0.5);
      Point3D desiredPosition = new Point3D(initialPosition);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      YoFramePoseUsingYawPitchRoll initialOrientationViz = new YoFramePoseUsingYawPitchRoll("initialOrientationViz", "viz", worldFrame, registry);
      initialOrientationViz.setPosition(initialPosition);
      initialOrientationViz.setOrientation(initialOrientation);
      YoFramePoseUsingYawPitchRoll finalOrientationViz = new YoFramePoseUsingYawPitchRoll("finalOrientationViz", "viz", worldFrame, registry);
      finalOrientationViz.setPosition(finalPosition);
      finalOrientationViz.setOrientation(finalOrientation);
      YoFramePoseUsingYawPitchRoll desiredOrientationViz = new YoFramePoseUsingYawPitchRoll("desiredOrientationViz", "viz", worldFrame, registry);
      desiredOrientationViz.set(initialOrientationViz);

      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicCoordinateSystem("blopDes", desiredOrientationViz, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicCoordinateSystem("blopInitial", initialOrientationViz, 0.5, YoAppearance.Orange()));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicCoordinateSystem("blopFinal", finalOrientationViz, 0.5, YoAppearance.Purple()));

      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicVector("blopDir", desiredOrientationViz.getPosition(), trajAngularVelocityDirection,
                                                                              YoAppearance.BlueViolet()));
      yoGraphicsListRegistry.registerYoGraphic("trajViz", new YoGraphicVector("blopDirCheck", desiredOrientationViz.getPosition(), FDAngularVelocityDirection,
                                                                              YoAppearance.Black()));

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);
         traj1.compute(t);
         traj1.getOrientation(orientationCurr);
         FD2AngularVelocity.update(quatCurr);

         desiredPosition.interpolate(initialPosition, finalPosition, t / trajectoryTime);
         desiredOrientationViz.setPosition(desiredPosition);
         desiredOrientationViz.setOrientation(orientationCurr);

         initialDriftFullRotations.set(initialAngularVelocity.length() * t / (2.0 * Math.PI));
         finalDriftFullRotations.set(finalAngularVelocity.length() * t / (2.0 * Math.PI));

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
      new HermiteCurveBasedOrientationTrajectoryGeneratorVisualizer();
   }
}
