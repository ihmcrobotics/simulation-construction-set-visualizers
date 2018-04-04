package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.VelocityConstrainedPoseTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VelocityConstrainedPoseTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(46561L);
   private static final ReferenceFrame aFrame = worldFrame;//RandomTools.generateRandomReferenceFrame("aFrame", random, worldFrame);
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);;

   private final VelocityConstrainedPoseTrajectoryGenerator traj1;
   private final StraightLinePoseTrajectoryGenerator traj2;

   private final YoFramePoint position = new YoFramePoint("position", worldFrame, registry);
   private final YoFrameVector velocity = new YoFrameVector("velocity", worldFrame, registry);
   private final YoFrameVector acceleration = new YoFrameVector("acceleration", worldFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   // FD velocity
   
   private FrameQuaternion orientation3 = new FrameQuaternion();
   private FrameQuaternion orientation1 = new FrameQuaternion();
   private Quaternion quat3 = new Quaternion();
   private Quaternion quat1 = new Quaternion();
   private Quaternion quatDelta = new Quaternion();
   private FrameVector3D vectorDelta = new FrameVector3D();
   double angle, omega;
   private final YoFrameVector FDAngularVelocity = new YoFrameVector("FDAngularVelocity", worldFrame, registry);
   
   double FDdt;
   
   
   public VelocityConstrainedPoseTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      String namePrefix = "traj";
      ReferenceFrame trajectoryFrame = aFrame;
      
      
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      traj1 = new VelocityConstrainedPoseTrajectoryGenerator(namePrefix + "1", true, trajectoryFrame, registry, true, yoGraphicsListRegistry);
      traj2 = new StraightLinePoseTrajectoryGenerator(namePrefix + "2", true, trajectoryFrame, registry, true, yoGraphicsListRegistry);
      
      traj1.registerNewTrajectoryFrame(worldFrame);
      traj1.registerNewTrajectoryFrame(aFrame);
      traj1.switchTrajectoryFrame(aFrame);
      
      traj2.registerNewTrajectoryFrame(worldFrame);
      traj2.registerNewTrajectoryFrame(aFrame);
      traj2.switchTrajectoryFrame(aFrame);

      
      FrameVector3D initialVelocity = new FrameVector3D(aFrame);
      initialVelocity.set(-2.0, 0.5, -0.5);
      FrameVector3D initialAngularVelocity =new FrameVector3D(aFrame);// =RandomTools.generateRandomFrameVector(random, aFrame);//new FrameVector(aFrame);
      
      traj1.setTrajectoryTime(trajectoryTime);
      traj2.setTrajectoryTime(trajectoryTime);
      
      FramePose3D initialPose = new FramePose3D(aFrame);//RandomTools.generateRandomFramePose(random, aFrame, 2.0, 2.0, 2.0);
      initialPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      initialPose.setPosition(0.5, 0.5, 0.5);
      initialAngularVelocity.set(1.0, 0.5, 5.0);
      
      FramePose3D finalPose = /*new FramePose(aFrame);*/ EuclidFrameRandomTools.nextFramePose3D(random, aFrame, 2.0, 2.0, 2.0);
//      finalPose.setOrientation(0.0, 0.0, Math.PI / 2.0);
      finalPose.setOrientationYawPitchRoll(1.0, 1.0, 1.0);
      
      
      finalPose.setPosition(1.0, 1.0, 1.0);
      
      FrameVector3D finalVelocity = new FrameVector3D(aFrame, 0.0, 0.0, 0.0 );
      FrameVector3D finalAngularVelocity = new FrameVector3D(aFrame, 1.0, 1.0, 3.0 );
//     
      
      
      traj1.setInitialPoseWithInitialVelocity(initialPose, initialVelocity, initialAngularVelocity);
      traj2.setInitialPose(initialPose);

//      traj1.setFinalPoseWithFinalVelocity(finalPose, finalVelocity, finalAngularVelocity);
      traj1.setFinalPoseWithoutFinalVelocity(finalPose);
      traj2.setFinalPose(finalPose);
      
      traj1.changeFrame(worldFrame);
      traj2.changeFrame(worldFrame);
      
      
      traj1.initialize();
      traj2.initialize();
      traj1.showVisualization();
      traj2.showVisualization();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         robot.getYoTime().set(t);
         traj1.compute(t);
         traj1.getPosition(tempPoint);
         position.set(tempPoint);
         traj1.getVelocity(tempVector);
         velocity.set(tempVector);
         traj1.getAcceleration(tempVector);
         acceleration.set(tempVector);

         traj2.compute(t);
         traj2.getPosition(tempPoint);
         position.set(tempPoint);
         traj2.getVelocity(tempVector);
         velocity.set(tempVector);
         traj2.getAcceleration(tempVector);
         acceleration.set(tempVector);
         
         traj1.showVisualization();
         traj2.showVisualization();
         
         
         FDdt = 5e-6;
         if(t >= dt && t <= trajectoryTime -dt)
         {
            traj1.compute(t + FDdt);
            traj1.getOrientation(orientation3);
            traj1.compute(t - FDdt);
            traj1.getOrientation(orientation1);
            
            
            orientation3.changeFrame(worldFrame);
            orientation1.changeFrame(worldFrame);
            
            quat3.set(orientation3);
            quat1.set(orientation1);

            quat1.inverse();
            quatDelta.multiply(quat3, quat1);
            
            
            quatDelta.normalize();
            
            angle = Math.acos(quatDelta.getS()) * 2.0;
            omega = angle / (2.0 * FDdt);

            
            vectorDelta.setIncludingFrame(worldFrame, quatDelta.getX(), quatDelta.getY(), quatDelta.getZ());
            
            
            if(vectorDelta.length() > 0.0)
            {
               vectorDelta.normalize();
            }
            vectorDelta.scale(omega);
            
            FDAngularVelocity.set(vectorDelta);
            
            
            
         }

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new VelocityConstrainedPoseTrajectoryGeneratorVisualizer();
   }
}