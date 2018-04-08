package us.ihmc.graphics3DDescription.yoGraphics;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class YoGraphicPlanarRegionsListVisualizer
{
   public static void main(String[] args)
   {
      setupWithSCS();
//      setupWithRemoteVisualizer();
   }

   private static void setupWithSCS()
   {
      Robot robot = new Robot("Dummy");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      YoVariableRegistry registry = scs.getRootRegistry();
      final YoGraphicPlanarRegionsList graphic = new YoGraphicPlanarRegionsList("planarRegion", 1000, 14, true, 10, registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("test", graphic);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      scs.addScript(new Script()
      {
         @Override
         public void doScript(double t)
         {
            graphic.submitPlanarRegionsListToRender(createPlanaRegionsList(), Color.green);
            graphic.processPlanarRegionsListQueue(false);
            graphic.update();
         }
      });
      scs.startOnAThread();
   }

   private static void setupWithRemoteVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("blop");
      final YoGraphicPlanarRegionsList graphic = new YoGraphicPlanarRegionsList("planarRegion", 1000, 14, true, 10, registry);
      graphic.submitPlanarRegionsListToRender(createPlanaRegionsList(), Color.green);
      YoGraphicCoordinateSystem worldCoordinates = new YoGraphicCoordinateSystem("world", "", registry, 1.0);
      YoGraphicPolygon polygon = createYoGraphicPolygon(registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("test", graphic);
      yoGraphicsListRegistry.registerYoGraphic("test", worldCoordinates);
      yoGraphicsListRegistry.registerYoGraphic("test", polygon);

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      final YoVariableServer yoVariableServer = new YoVariableServer(YoGraphicPlanarRegionsListVisualizer.class, scheduler, null, LogSettings.TOOLBOX, 0.001);
      yoVariableServer.setMainRegistry(registry, null, yoGraphicsListRegistry);
      yoVariableServer.start();
      Runnable command = new Runnable()
      {
         long count = 0L;
         @Override
         public void run()
         {
            graphic.submitPlanarRegionsListToRender(createPlanaRegionsList(), Color.GREEN);
            graphic.processPlanarRegionsListQueue();
            yoVariableServer.update(count += Conversions.millisecondsToNanoseconds(10L));
         }
      };
      Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(command, 0L, 10L, TimeUnit.MILLISECONDS);

      JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
      spawner.spawn(SCSVisualizer.class);
   }

   private static YoGraphicPolygon createYoGraphicPolygon(YoVariableRegistry registry)
   {
      YoFrameConvexPolygon2D convexPolygon2d = new YoFrameConvexPolygon2D("poupou", ReferenceFrame.getWorldFrame(), 30, registry);
      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(new Point2D(1.0, 0.0), 0.1)));
      convexPolygon2d.set(polygon);
      return new YoGraphicPolygon("poly", convexPolygon2d, "shnoup", "", registry, 1.0, YoAppearance.Red());
   }

   private static PlanarRegionsList createPlanaRegionsList()
   {

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslation(0.0, 0.0, 0.2);
      List<ConvexPolygon2D> polygons1 = new ArrayList<>();
      polygons1.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(0.1))));
      polygons1.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(new Point2D(0.5, -0.2), 0.35))));
      polygons1.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(new Point2D(-0.5, -0.1), 0.4))));
      PlanarRegion planarRegion1 = new PlanarRegion(transformToWorld, polygons1);
      transformToWorld.setRotationRollAndZeroTranslation(0.3 * Math.PI);
      transformToWorld.setTranslation(0.0, 0.3, 0.1);
      PlanarRegion planarRegion2 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(0.3))));
      Random random = new Random(45L);
      transformToWorld = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      PlanarRegion planarRegion3 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(0.3))));
      transformToWorld = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      PlanarRegion planarRegion4 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(0.3))));
      transformToWorld = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      PlanarRegion planarRegion5 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(createCircle(0.3))));

      return new PlanarRegionsList(planarRegion1, planarRegion2, planarRegion3, planarRegion4, planarRegion5);
   }

   private static List<Point2D> createCircle(double radius)
   {
      return createCircle(new Point2D(), radius);
   }

   private static List<Point2D> createCircle(Point2D center, double radius)
   {
      List<Point2D> vertices = new ArrayList<>();

      int numberOfPoints = 10;
      for (int i = 0; i < numberOfPoints; i++)
      {
         double angle = i / (double) numberOfPoints * 2.0 * Math.PI;
         double x = center.getX() + radius * Math.cos(angle);
         double y = center.getY() + radius * Math.sin(angle);
         vertices.add(new Point2D(x, y));
      }
      return vertices;
   }
}
