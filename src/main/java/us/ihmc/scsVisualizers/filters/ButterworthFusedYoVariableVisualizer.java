package us.ihmc.scsVisualizers.filters;

import us.ihmc.robotics.math.filters.AlphaFusedYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable.ButterworthFilterType;
import us.ihmc.robotics.math.filters.ButterworthFusedYoVariable;
import us.ihmc.simulationConstructionSet.util.RobotController;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class ButterworthFusedYoVariableVisualizer
{
   private static final double SIMULATION_DT = 0.001;
   private static final int RECORD_RATE = 1;

   private static final double MAX_FREQ_HERTZ = 1.0 / ((2.0 * Math.PI) * SIMULATION_DT);

   {
      System.out.println("MAX_FREQ_HERTZ = " + MAX_FREQ_HERTZ);
   }

   public static void main(String[] args)
   {
      new ButterworthFusedYoVariableVisualizer();
   }

   public ButterworthFusedYoVariableVisualizer()
   {
      Robot robot = new Robot("null")
      {
      } ;

      ButterworthFusedYoVariableController controller = new ButterworthFusedYoVariableController((YoDouble)robot.getVariable("t"));
      robot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.setDT(SIMULATION_DT, RECORD_RATE);

      GraphConfiguration autoscale = new GraphConfiguration("autoscale", GraphConfiguration.AUTO_SCALING);
      GraphConfiguration tipsOf3db = new GraphConfiguration("tipsOf3db", GraphConfiguration.MANUAL_SCALING, 0.70, 0.72);

      GraphConfiguration[] graphConfigurations = new GraphConfiguration[] {autoscale, tipsOf3db};
      scs.setupGraphConfigurations(graphConfigurations);

      scs.setupGraphGroup("testFused", new String[][][]
      {
         {
            {"original", "fused"}, {"autoscale"}
         },
         {
            {"original", "alphaFused"}, {"autoscale"}
         },
         {
            {"original", "lowPass"}, {"autoscale"}
         },
         {
            {"original", "highPass"}, {"autoscale"}
         },
         {
            {"original", "lowPassPlusHighPass"}, {"autoscale"}
         },
         {
            {"alpha", "cutoffFreq"}, {"autoscale"}
         }
      });

      scs.setupGraphGroup("lowAndHigh", new String[][][]
      {
         {
            {"lowPass", "highPass"}, {"tipsOf3db"}
         },
      });



      scs.setupEntryBoxGroup("testFused", new String[] {"alpha", "amp1", "freq1", "amp2", "freq2"});


      scs.setupConfiguration("testFused", "all", "testFused", "testFused");

      scs.selectConfiguration("testFused");

      scs.hideViewport();

//    scs.createNewGraphWindow("lowAndHigh", 1, true);

      MidiSliderBoard evolutionUC33E = new MidiSliderBoard(scs);
      evolutionUC33E.setSlider(1, "alpha", scs, 0.0, 1.0);
      evolutionUC33E.setSlider(2, "amp1", scs, 0.0, 1.0);
      evolutionUC33E.setSlider(3, "freq1", scs, 7.95, 9.0);
      evolutionUC33E.setSlider(4, "amp2", scs, 0.0, 1.0);
      evolutionUC33E.setSlider(5, "freq2", scs, 0, 100);


      Thread thread = new Thread(scs);
      thread.start();

      while (true)
      {
         try
         {
            Thread.sleep(4000);
         }
         catch (InterruptedException ex)
         {
         }
      }
   }


   private class ButterworthFusedYoVariableController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("ButterworthFusedYoVariableController");

      private final YoDouble dc = new YoDouble("dc", registry);

      private final YoDouble amp1 = new YoDouble("amp1", registry);
      private final YoDouble amp2 = new YoDouble("amp2", registry);
      private final YoDouble amp3 = new YoDouble("amp3", registry);

      private final YoDouble freq1 = new YoDouble("freq1", registry);
      private final YoDouble freq2 = new YoDouble("freq2", registry);
      private final YoDouble freq3 = new YoDouble("freq3", registry);

      private final YoDouble original = new YoDouble("original", registry);

      private final YoDouble highFreqNoise = new YoDouble("highFreqNoise", registry);
      private final YoDouble lowFreqNoise = new YoDouble("lowFreqNoise", registry);

      private final YoDouble originalPlusHighFreqNoise = new YoDouble("originalPlusHighFreqNoise", registry);
      private final YoDouble originalPlusLowFreqNoise = new YoDouble("originalPlusLowFreqNoise", registry);

      private final YoDouble lowPassPlusHighPass = new YoDouble("lowPassPlusHighPass", registry);

      private final YoDouble alpha = new YoDouble("alpha", registry);
      private final YoDouble cutoffFreq = new YoDouble("cutoffFreq", registry);

      private final ButterworthFilteredYoVariable lowPass, highPass;

      private final ButterworthFusedYoVariable fused;
      private final AlphaFusedYoVariable alphaFused;

      private final YoDouble t;

      public ButterworthFusedYoVariableController(YoDouble t)
      {
         this.t = t;

         fused = new ButterworthFusedYoVariable("fused", registry, alpha, originalPlusHighFreqNoise, originalPlusLowFreqNoise);
         alphaFused = new AlphaFusedYoVariable("alphaFused", registry, alpha, originalPlusHighFreqNoise, originalPlusLowFreqNoise);

         lowPass = new ButterworthFilteredYoVariable("lowPass", registry, alpha, originalPlusHighFreqNoise, ButterworthFilterType.LOW_PASS);
         highPass = new ButterworthFilteredYoVariable("highPass", registry, alpha, originalPlusLowFreqNoise, ButterworthFilterType.HIGH_PASS);

         amp1.set(1.0);
         freq1.set(10.0);

         amp2.set(0.1);
         freq2.set(100.0);

         amp3.set(0.2);
         freq3.set(3.0);

         dc.set(1.2);

         alpha.set(0.75);
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public void doControl()
      {
         cutoffFreq.set((1.0 - alpha.getDoubleValue()) / (2.0 * Math.PI * SIMULATION_DT));


         original.set(dc.getDoubleValue() + amp1.getDoubleValue() * Math.cos(2.0 * Math.PI * freq1.getDoubleValue() * t.getDoubleValue()) + amp2.getDoubleValue() * Math.cos(2.0 * Math.PI * freq2.getDoubleValue() * t.getDoubleValue())
                        + amp3.getDoubleValue() * Math.signum(Math.cos(2.0 * Math.PI * freq3.getDoubleValue() * t.getDoubleValue())));

         originalPlusHighFreqNoise.set(original.getDoubleValue() + highFreqNoise.getDoubleValue());
         originalPlusLowFreqNoise.set(original.getDoubleValue() + lowFreqNoise.getDoubleValue());

         lowPass.update();
         highPass.update();
         fused.update();
         alphaFused.update();

         lowPassPlusHighPass.set(lowPass.getDoubleValue() + highPass.getDoubleValue());
      }

      public String getName()
      {
         return "butterworthFusedYoVariableController";
      }
      
      public void initialize()
      {      
      }

      public String getDescription()
      {
         return getName();
      }

   }
}
