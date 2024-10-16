package us.ihmc.scsVisualizers.graphics;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ChangingMeshGraphicsVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoFrameConvexPolygon2D yoConvexPolygon2d = new YoFrameConvexPolygon2D("convexPolygon", ReferenceFrame.getWorldFrame(), 10, registry);
   private YoFramePoseUsingYawPitchRoll framePose = new YoFramePoseUsingYawPitchRoll("pose", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPolygon graphicPolygon = new YoGraphicPolygon("graphicPolygon", yoConvexPolygon2d, framePose, 1.0, YoAppearance.Purple());

   private final YoGraphicPosition pointOne = new YoGraphicPosition("point1", "", registry, 0.01, YoAppearance.Red());

   public ChangingMeshGraphicsVisualizer()
   {
      Robot robot = new Robot("null");

      pointOne.setPosition(new Point3D(1.0, 1.0, 0.01));

      yoGraphicsListRegistry.registerYoGraphic("Polygon", graphicPolygon);
      yoGraphicsListRegistry.registerYoGraphic("Points", pointOne);
      yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(graphicPolygon);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(staticLinkGraphics);

      scs.getRootRegistry().addChild(registry);
      scs.startOnAThread();

      scs.tickAndUpdate();


      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(new double[][]{{0.0, 0.0, 0.01}, {0.0, 1.0, 0.01}, {1.0, 1.0, 0.01}, {1.0, 0.0, 0.01}}));
      yoConvexPolygon2d.set(polygon);
      graphicPolygon.update();

      scs.tickAndUpdate();

      polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(new double[][]{{0.0, 1.0, 0.01}, {0.0, 2.0, 0.01}, {1.0, 2.0, 0.01}, {1.0, 1.0, 0.01}}));
      yoConvexPolygon2d.set(polygon);
      graphicPolygon.update();

      scs.tickAndUpdate();

   }


   public static void main(String[] args)
   {
      new ChangingMeshGraphicsVisualizer();
   }
}
