package us.ihmc.scsVisualizers.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class AlphaFilteredWrappingYoVariableVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double dt = 0.001;
   private final int recordFrequency = 5;
   int maxTimeStamp = 10000;
   private final int bufferSize = maxTimeStamp;

   private final YoLong timeStamp = new YoLong("timeStamp", registry);

   private final YoDouble unfilteredVar = new YoDouble("unfilteredVar", registry);
   private final YoDouble alphaVariable = new YoDouble("alpha", registry);

   private final AlphaFilteredWrappingYoVariable filteredVar = new AlphaFilteredWrappingYoVariable("filteredVar", "", registry, unfilteredVar, alphaVariable, -5.0, 5.0);

   public AlphaFilteredWrappingYoVariableVisualizer()
   {
      SimulationConstructionSet scs = setupSCS();

      alphaVariable.set(0.0);
      
      unfilteredVar.set(0.1);
      filteredVar.update();
      scs.tickAndUpdate();
      
      unfilteredVar.set(3.5);
      for(int i = 0; i<2000; i++)
      {
         filteredVar.update();
         scs.tickAndUpdate();
      }
      
      unfilteredVar.set(-4.1);
      for(int i = 0; i<2000; i++)
      {
         filteredVar.update();
         scs.tickAndUpdate();
      }
      
      unfilteredVar.set(4.0);
      for(int i = 0; i<2000; i++)
      {
         filteredVar.update();
         scs.tickAndUpdate();
      }
      
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private SimulationConstructionSet setupSCS()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.setGroundVisible(false);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(linkGraphics);
      return scs;
   }

   public static void main(String[] args)
   {
      new AlphaFilteredWrappingYoVariableVisualizer();
   }
}
