package us.ihmc.scsVisualizers.geometry;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

public class ConvexPolygon2dVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AtlasPhysicalProperties atlasPhysicalProperties = new AtlasPhysicalProperties(1);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble amountToShrink = new YoDouble("amountToShrink", registry);
   private final YoBoolean shrinkSucceed = new YoBoolean("shrinkSucceed", registry);

   private final int maxNumberOfVertices = 30;
   private final int maxNumberOfEdges = maxNumberOfVertices;

   private final YoFrameConvexPolygon2D yoOriginalPolygon = new YoFrameConvexPolygon2D("originalPolygon", "", worldFrame, maxNumberOfVertices, registry);
   private final YoFrameConvexPolygon2D yoShrunkPolygon = new YoFrameConvexPolygon2D("shrunkPolygon", "", worldFrame, maxNumberOfVertices, registry);

   private final YoFrameLine2D[] edgeLines = new YoFrameLine2D[maxNumberOfEdges];

   private final YoArtifactPolygon originalPolygonArtifact = new YoArtifactPolygon("originalPolygon", yoOriginalPolygon, Color.blue, false);
   private final YoArtifactPolygon shrunkPolygonArtifact = new YoArtifactPolygon("shrunkPolygon", yoShrunkPolygon, Color.gray, false);

   private final Random random = new Random(1651651L);

   public ConvexPolygon2dVisualizer()
   {
      Robot robot = new Robot("DummyRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 8000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      List<ConvexPolygon2D> convexPolygonsToShrink = createConvexPolygonsToShrink();

      yoGraphicsListRegistry.registerArtifact("Viz", originalPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("Viz", shrunkPolygonArtifact);

      for (int i = 0; i < maxNumberOfEdges; i++)
      {
         YoFrameLine2D edgeLine = new YoFrameLine2D("edgeLine_" + i, "", worldFrame, registry);
         edgeLines[i] = edgeLine;

         yoGraphicsListRegistry.registerArtifact("Viz", new YoArtifactLine2d("edgeViz_" + i, edgeLine, Color.yellow));
      }

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoVariableRegistry(registry);
      
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();

//      shrinkPolygonsAndDisplay(scs, convexPolygonsToShrink, RandomTools.generateRandomDoubleArray(random, 100, 0.0, 0.2));
      TDoubleArrayList shrinkDistances = new TDoubleArrayList();
      for (double shrink = -0.1; shrink <= 2.0 * atlasPhysicalProperties.getFootWidthForControl(); shrink+=0.001)
      {
         shrinkDistances.add(shrink);
      }
      
      shrinkPolygonsAndDisplay(scs, convexPolygonsToShrink, shrinkDistances.toArray());
      
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }
   
   public List<ConvexPolygon2D> createConvexPolygonsToShrink()
   {

      Random random = new Random(1986L);
      
      ArrayList<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      convexPolygons.add(createTypicalAtlasPolygon());
      convexPolygons.add(createLongAndSkinnyPolygon());
      convexPolygons.add(createLinePolygon());
      convexPolygons.add(createPointPolygon());
      
      
      double maxAbsoluteXY = 0.3;
      
      int numberOfPolygons = 20;
      
      for (int i=0; i<numberOfPolygons; i++)
      {
         int numberOfPossiblePoints = RandomNumbers.nextInt(random, 3, 10);
         convexPolygons.add(EuclidGeometryRandomTools.nextConvexPolygon2D(random, maxAbsoluteXY, numberOfPossiblePoints));
      }

      return convexPolygons;
   }

   private ConvexPolygon2D createTypicalAtlasPolygon()
   {
      List<Point2D> contactPointsFromAtlas = new ArrayList<Point2D>();
      contactPointsFromAtlas.add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0, atlasPhysicalProperties.getToeWidthForControl() / 2.0));
      contactPointsFromAtlas.add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 1.5, 0.0));
      contactPointsFromAtlas.add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0, -atlasPhysicalProperties.getToeWidthForControl() / 2.0));
      contactPointsFromAtlas.add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0, -atlasPhysicalProperties.getFootWidthForControl() / 2.0));
      contactPointsFromAtlas.add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0, atlasPhysicalProperties.getFootWidthForControl() / 2.0));
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPointsFromAtlas));
      return convexPolygon;
   }
   
   private ConvexPolygon2D createLongAndSkinnyPolygon()
   {
      List<Point2D> contactPointsFromAtlas = new ArrayList<Point2D>();
      contactPointsFromAtlas.add(new Point2D(0.0, 0.0));
      contactPointsFromAtlas.add(new Point2D(1.0, 0.0));
      contactPointsFromAtlas.add(new Point2D(1.0, 0.2));
      contactPointsFromAtlas.add(new Point2D(0.0, 0.2));
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPointsFromAtlas));
      return convexPolygon;
   }

   private ConvexPolygon2D createLinePolygon()
   {
      List<Point2D> line = new ArrayList<Point2D>();
      line.add(new Point2D(0.0, 0.1));
      line.add(new Point2D(1.0, 0.1));
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(line));
      return convexPolygon;
   }

   private ConvexPolygon2D createPointPolygon()
   {
      List<Point2D> point = new ArrayList<Point2D>();
      point.add(new Point2D(0.1, 0.1));
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(point));
      return convexPolygon;
   }
   
   
 

   private void shrinkPolygonsAndDisplay(SimulationConstructionSet scs, List<ConvexPolygon2D> convexPolygonsToShrink, double... shrinkDistances)
   {
      ConvexPolygon2D originalPolygon = new ConvexPolygon2D();
      ConvexPolygon2D shrunkPolygon = new ConvexPolygon2D();
      FrameLine2D line = new FrameLine2D();

      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      
      for (ConvexPolygon2D polygonToShrink : convexPolygonsToShrink)
      {
         for (double shrink : shrinkDistances)
         {
            if (MathTools.epsilonEquals(shrink, 0.047, 1.0e-5))
               System.out.println();
            amountToShrink.set(shrink);
            originalPolygon.set(polygonToShrink);
            
            shrunkPolygon = new ConvexPolygon2D();
            shrunkPolygon.set(originalPolygon);
            
            shrinkSucceed.set(shrinker.scaleConvexPolygon(originalPolygon, shrink, shrunkPolygon));

            if(originalPolygon.getNumberOfVertices() > 1)
            {
               for (int i = 0; i < originalPolygon.getNumberOfVertices(); i++)
               {
                  FramePoint2D vertex = new FramePoint2D(worldFrame, originalPolygon.getVertex(i));
                  FramePoint2D nextVertex = new FramePoint2D(worldFrame, originalPolygon.getNextVertex(i));
                  line.set(vertex, nextVertex);
                  line.shiftToRight(shrink);
                  edgeLines[i].set(line);
               }
               
               for (int i = originalPolygon.getNumberOfVertices(); i < maxNumberOfEdges; i++)
               {
                  line.set(worldFrame, 10000.0, 10000.0, 10001.0, 10001.0);
                  edgeLines[i].set(line);
               }
            }

            yoOriginalPolygon.set(originalPolygon);
            yoShrunkPolygon.set(shrunkPolygon);

            scs.setTime(scs.getTime() + 0.01);
            scs.tickAndUpdate();
         }
      }
   }

   public static void main(String[] args)
   {
      new ConvexPolygon2dVisualizer();
   }
}
