package us.ihmc.scsVisualizers.geometry;

import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

public class Box3dVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   
   private final SimulationConstructionSet scs;
   private final YoRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   
   private final YoFramePoseUsingYawPitchRoll yoBoxPose;
   private final YoFrameVector3D yoBoxSize;
   private final YoFramePoint3D[] yoBoxVertices;
   private final YoFramePoseUsingYawPitchRoll yoBox2Pose;
   private final YoFrameVector3D yoBox2Size;
   private final YoFramePoint3D[] yoBox2Vertices;
   
   private final YoFramePoint3D isInsidePoint;
   private final YoBoolean isInside;
   
   private final Box3D box;
   private final Point3D[] boxVertices;
   private final Box3D box2;
   private final Point3D[] box2Vertices;
   
   private final RigidBodyTransform boxTransform;
   
   private final String prefix = "boxVisualizer";
   private final double scale = 0.05;
   private final Random random;
   
   public Box3dVisualizer()
   {
      YoVariableChangedListener doATickListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            doATick();
         }
      };
      
      random = new Random(546546L);
      
      registry = new YoRegistry(getClass().getSimpleName());
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      yoBoxPose = new YoFramePoseUsingYawPitchRoll(prefix + "Pose", WORLD, registry);
      yoBoxSize = new YoFrameVector3D(prefix + "Size", WORLD, registry);
      yoBox2Pose = new YoFramePoseUsingYawPitchRoll(prefix + "Pose2", WORLD, registry);
      yoBox2Size = new YoFrameVector3D(prefix + "Size2", WORLD, registry);
      
      boxTransform = new RigidBodyTransform();
      
      isInsidePoint = new YoFramePoint3D(prefix + "IsInside", WORLD, registry);
      isInside = new YoBoolean(prefix + "IsInside", registry);
      yoGraphicsListRegistry.registerYoGraphic("Box", new YoGraphicPosition("isInside", isInsidePoint, scale , YoAppearance.Orange()));
      isInsidePoint.attachVariableChangedListener(doATickListener);
      
      box = new Box3D();
      box2 = new Box3D();
      boxVertices = new Point3D[8];
      box2Vertices = new Point3D[8];
      yoBoxVertices = new YoFramePoint3D[8];
      yoBox2Vertices = new YoFramePoint3D[8];
      for (int i = 0; i < 8; i++)
      {
         boxVertices[i] = new Point3D();
         box2Vertices[i] = new Point3D();
         yoBoxVertices[i] = new YoFramePoint3D(prefix + "Vertex" + i, WORLD, registry);
         yoBox2Vertices[i] = new YoFramePoint3D(prefix + "Vertex" + i + "2", WORLD, registry);
         yoGraphicsListRegistry.registerYoGraphic("Box", new YoGraphicPosition("vertex" + i, yoBoxVertices[i], scale , YoAppearance.Blue()));
         yoGraphicsListRegistry.registerYoGraphic("Box", new YoGraphicPosition("vertex" + i + "2", yoBox2Vertices[i], scale , YoAppearance.Gray()));
      }
      
      yoGraphicsListRegistry.registerYoGraphic("Box", new YoGraphicCoordinateSystem("center", yoBoxPose, 0.5)); 
      yoGraphicsListRegistry.registerYoGraphic("Box", new YoGraphicCoordinateSystem("center2", yoBox2Pose, 0.5)); 
      
      scs = new SimulationConstructionSet(new Robot("Robot"));
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(1.0, 1);
      
      yoBoxPose.attachVariableChangedListener(doATickListener);
      
      for (int i = 0; i < 1; i++)
      {
         yoBoxSize.set(0.9333700941864332, 0.7184905128592369, 0.9686485749324065);
         yoBoxPose.setPosition(0.4573906006655194, 0.2700946102963995, 0.4594864866836388);
         yoBoxPose.setOrientation(new Quaternion(0.378697127565547, -0.4330011737771966, 0.7058772508997949, -0.41332284686830684));
         isInsidePoint.set(0.45875994943142195, 0.8716506061253778, -0.5261917155326467);
         
         boxTransform.setRotationEulerAndZeroTranslation(0.5, 0.5, 0.5);
         boxTransform.getTranslation().set(1.0, 1.0, 1.0);
         
         doATick();
      }
      
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private void doATick()
   {
      box.getPose().set(yoBoxPose);
      box.getSize().set(yoBoxSize.getX(), yoBoxSize.getY(), yoBoxSize.getZ());
      for (int i = 0; i < boxVertices.length; i++)
      {
         box.getVertex(i, boxVertices[i]);
         yoBoxVertices[i].set(boxVertices[i]);
      }
      isInside.set(box.isPointInside(isInsidePoint));
      
      box2.set(box);
      box2.getPose().multiply(boxTransform);
      Point3D point = new Point3D(box2.getPosition());
      yoBox2Pose.setPosition(point);
      RotationMatrix matrix3dCopy = new RotationMatrix(box2.getOrientation());
      yoBox2Pose.setOrientation(matrix3dCopy);
      yoBox2Size.set(box2.getSizeX(), box.getSizeY(), box.getSizeZ());
      for (int i = 0; i < box2Vertices.length; i++)
      {
         box2.getVertex(i, box2Vertices[i]);
         yoBox2Vertices[i].set(box2Vertices[i]);
      }
      
      scs.tickAndUpdate();
   }
   
   public static void main(String[] args)
   {
      new Box3dVisualizer();
   }
}
