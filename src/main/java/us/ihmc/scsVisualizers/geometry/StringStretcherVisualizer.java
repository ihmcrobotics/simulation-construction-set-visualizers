package us.ihmc.scsVisualizers.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class StringStretcherVisualizer implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final ArrayList<YoGraphicPosition> yoWaypoints = new ArrayList<>();

   private final YoGraphicPosition yoStartPoint = new YoGraphicPosition("startPosition", "", registry, 0.05, YoAppearance.Blue());
   private final YoGraphicPosition yoEndPoint = new YoGraphicPosition("endPosition", "", registry, 0.05, YoAppearance.Blue());
   private Point2D startPoint = new Point2D();
   private Point2D endPoint = new Point2D();

   private final ArrayList<YoFramePoint3D> minPoints = new ArrayList<>();
   private final ArrayList<YoFramePoint3D> maxPoints = new ArrayList<>();

   private final Point2D minPoint = new Point2D();
   private final Point2D maxPoint = new Point2D();
   private final List<Point2DBasics> newPoints = new ArrayList<>();
   private final YoBoolean problemOccured = new YoBoolean("problemOccured", registry);
   
   private final SliderBoardConfigurationManager sliderBoardConfigurationManager;

   public StringStretcherVisualizer()
   {

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      
      yoStartPoint.setPosition(-0.5, 0.5, 0.0);
      yoEndPoint.setPosition(1.0, 0.0, 0.0);
      
      graphicsListRegistry.registerArtifact("startPosition", yoStartPoint.createArtifact());
      graphicsListRegistry.registerArtifact("endPosition", yoEndPoint.createArtifact());
      
      double[] yMaxValues = {0.6, 0.6};
      double[] yMinValues = {0.0, 0.0};

      for (int i = 0; i < 2; i++)
      {
         YoFramePoint3D yoMinPoint = new YoFramePoint3D("minPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoMinPoint.set(0.25 * i + 0.1, yMinValues[i], 0.0);
         minPoints.add(yoMinPoint);
         YoGraphicPosition minPointGraphic = new YoGraphicPosition("minPoint" + i, yoMinPoint, 0.05, YoAppearance.Crimson());
         graphicsListRegistry.registerArtifact("minPoint" + i, minPointGraphic.createArtifact());

         YoFramePoint3D yoMaxPoint = new YoFramePoint3D("maxPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoMaxPoint.set(0.25 * i + 0.1, yMaxValues[i], 0.0);
         maxPoints.add(yoMaxPoint);
         YoGraphicPosition maxPointGraphic = new YoGraphicPosition("maxPoint" + i, yoMaxPoint, 0.05, YoAppearance.Red());
         graphicsListRegistry.registerArtifact("maxPoint" + i, maxPointGraphic.createArtifact());

      }

      for (int i = 0; i < 4; i++)
      {
         YoGraphicPosition position = new YoGraphicPosition("point_", i + "", registry, 0.01, YoAppearance.Black(), GraphicType.BALL);
         graphicsListRegistry.registerArtifact("points", position.createArtifact());
         yoWaypoints.add(position);
      }

      Robot robot = new Robot("ROBOT");
      robot.setController(this);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setSimulateNoFasterThanRealTime(true);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      
      
      sliderBoardConfigurationManager.setSlider(1, yoStartPoint.getYoX(), -2.0, 2.0);
      sliderBoardConfigurationManager.setSlider(2, yoStartPoint.getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(3, yoEndPoint.getYoX(), -2.0, 2.0);
      sliderBoardConfigurationManager.setSlider(4, yoEndPoint.getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(5, minPoints.get(0).getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(6, maxPoints.get(0).getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(7, minPoints.get(1).getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(8, maxPoints.get(1).getYoY(), -1.0, 1.0);
      
      
      
      scs.startOnAThread();
      scs.simulate();
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      problemOccured.set(false);
      stringStretcher.reset();
      
      startPoint.set(yoStartPoint.getX(), yoStartPoint.getY());
      endPoint.set(yoEndPoint.getX(), yoEndPoint.getY());
      
      stringStretcher.setStartPoint(startPoint);
      stringStretcher.setEndPoint(endPoint);

      try
      {
         for (int i = 0; i < minPoints.size(); i++)
         {
            minPoint.set(minPoints.get(i).getX(), minPoints.get(i).getY());
            maxPoint.set(maxPoints.get(i).getX(), maxPoints.get(i).getY());
            stringStretcher.addMinMaxPoints(minPoint, maxPoint);
         }
      }
      catch (Exception e)
      {
         problemOccured.set(true);
      }

      stringStretcher.stretchString(newPoints);
      for (int i = 0; i < newPoints.size(); i++)
      {
         Point2DBasics point2d = newPoints.get(i);
         yoWaypoints.get(i).setPosition(point2d.getX(), point2d.getY(), 0.0);
      }
   }

   public static void main(String[] args)
   {
      new StringStretcherVisualizer();
   }

}
