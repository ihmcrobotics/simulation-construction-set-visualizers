package us.ihmc.scsVisualizers.geometry;

import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class FrameMatrix3DVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PoseReferenceFrame aFrame = new PoseReferenceFrame("aFrame", worldFrame);

   private final YoFramePoint3D zero = new YoFramePoint3D("zero", worldFrame, registry);
   private final YoFrameVector3D yoVectorOriginal = new YoFrameVector3D("original", worldFrame, registry);
   private final YoFrameVector3D yoVectorTransformedInWorld = new YoFrameVector3D("transformedInWorld", worldFrame, registry);
   private final YoFrameVector3D yoVectorTransformedInAFrame = new YoFrameVector3D("transformedInAFrame", worldFrame, registry);

   private final FramePose3D aFramePose = new FramePose3D();
   private final FrameVector3D originalVector = new FrameVector3D();
   private final FrameVector3D transformedVector = new FrameVector3D();
   private final FrameMatrix3D transformationFrameMatrix = new FrameMatrix3D(worldFrame);
   
   private final Random random = new Random(546546L);

   public FrameMatrix3DVisualizer()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.addYoVariableRegistry(registry);
      YoGraphicReferenceFrame aFrameViz = createYoGraphics(scs);

      for (int i = 0; i < 500; i++)
      {
         aFramePose.setOrientationYawPitchRoll(1.0, 1.7, -0.6);
         aFramePose.setPosition(-1.0, 2.0, 1.0);
         aFrame.setPoseAndUpdate(aFramePose);

         aFrameViz.update();

         originalVector.setIncludingFrame(worldFrame, 1.0, 0.5, 2.0);
         yoVectorOriginal.set(originalVector);

         // Build a random transformation matrix
         transformationFrameMatrix.setIncludingFrame(worldFrame, RandomGeometry.nextMatrix3D(random, 1.0));

         // Transform the originalVector while in worldFrame
         transformationFrameMatrix.transform(originalVector, transformedVector);
         // Update the viz that is in worldFrame
         yoVectorTransformedInWorld.set(transformedVector);

         // Change frame to apply the transformation into aFrame
         originalVector.changeFrame(aFrame);
         transformationFrameMatrix.changeFrame(aFrame);
         // Transform the originalVector while in aFrame 
         transformationFrameMatrix.transform(originalVector, transformedVector);
         // Update the viz that is in worldFrame to check that we obtain the same result as before
         yoVectorTransformedInAFrame.setMatchingFrame(transformedVector);
         
         scs.tickAndUpdate();
      }
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private YoGraphicReferenceFrame createYoGraphics(SimulationConstructionSet scs)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      String listName = "blop";
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicPosition("p", zero, 0.05, YoAppearance.Black()));
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicVector("original", zero, yoVectorOriginal, YoAppearance.Blue()));
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicVector("transformedInWorld", zero, yoVectorTransformedInWorld, YoAppearance.AluminumMaterial()));
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicVector("transformedInAFrame", zero, yoVectorTransformedInAFrame, YoAppearance.Chocolate()));
      YoGraphicReferenceFrame aFrameViz = new YoGraphicReferenceFrame(aFrame, registry, true, 1.0);
      yoGraphicsListRegistry.registerYoGraphic(listName, aFrameViz);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      return aFrameViz;
   }

   public static void main(String[] args)
   {
      new FrameMatrix3DVisualizer();
   }
}
