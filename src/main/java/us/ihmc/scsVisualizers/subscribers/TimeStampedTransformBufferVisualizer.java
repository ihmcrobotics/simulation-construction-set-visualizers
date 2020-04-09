package us.ihmc.scsVisualizers.subscribers;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class TimeStampedTransformBufferVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoLong timeStampYoVariable = new YoLong("timeStamp", registry);
   
   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);
   
   public TimeStampedTransformBufferVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      int numberOfTransforms = 10;
      long firstTimeStamp = 1000;
      long lastTimeStamp = 2000;
      Random random = new Random(1987L);
      
      TimeStampedTransformBuffer timeStampedPelvisPoseBuffer = new TimeStampedTransformBuffer(numberOfTransforms);

      YoFramePoseUsingYawPitchRoll pelvisPose = new YoFramePoseUsingYawPitchRoll("pelvisPose", worldFrame, registry);
      YoGraphicCoordinateSystem pelvisPoseGraphics = new YoGraphicCoordinateSystem("pelvisPoseGraph", pelvisPose, 0.5, YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("pelvisPoses", pelvisPoseGraphics);
      
      for(int i = 0; i < numberOfTransforms ; i++)
      {
         RigidBodyTransform currentTransform = new RigidBodyTransform();
         long currentTimeStamp = (long) (i * (lastTimeStamp - firstTimeStamp)/numberOfTransforms + firstTimeStamp); 
         currentTransform.getTranslation().set((double)(20.0/numberOfTransforms * i) - 5.0, RandomNumbers.nextDouble(random, -1.5, 1.5), RandomNumbers.nextDouble(random, 0.1, 2.0));
         currentTransform.getRotation().set(RandomGeometry.nextQuaternion(random));
         timeStampedPelvisPoseBuffer.put(currentTransform, currentTimeStamp);
         
         YoFramePoseUsingYawPitchRoll currentTransformYoFramePose = new YoFramePoseUsingYawPitchRoll("yoFramePose_" + i, worldFrame, registry);
         
         FramePose3D tempFramePose = new FramePose3D(worldFrame, currentTransform);
         currentTransformYoFramePose.set(tempFramePose);
         YoGraphicCoordinateSystem yoGraphicCoordinateSystem = new YoGraphicCoordinateSystem("transform_" + i, currentTransformYoFramePose , 0.3, YoAppearance.Blue());
         yoGraphicsListRegistry.registerYoGraphic("Transforms", yoGraphicCoordinateSystem);
      }

      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry); 
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(linkGraphics);
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      
      TimeStampedTransform3D timeStampTransform3DMoving = new TimeStampedTransform3D();
      
      FramePose3D tempFramePose = new FramePose3D(worldFrame);
      
      for(long timeStamp = firstTimeStamp; timeStamp < lastTimeStamp; timeStamp++)
      {
         timeStampYoVariable.set(timeStamp);
         timeStampedPelvisPoseBuffer.findTransform(timeStamp, timeStampTransform3DMoving);
         tempFramePose.set(timeStampTransform3DMoving.getTransform3D());
         pelvisPose.set(tempFramePose);
         scs.tickAndUpdate();
      }
      
      scs.startOnAThread();
      ThreadTools.sleepForever();
      
   }
   
   public static void main(String[] args)
   {
      new TimeStampedTransformBufferVisualizer();
   }
   
}
