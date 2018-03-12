package us.ihmc.scsVisualizers.gpuProfiling;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class GPUZLogPlotter
{
   private static final Path DEFAULT_LOG_PATH = Paths.get("AtlasPointCloudDataReceiverTestWithGUI.csv");
   
   private static final int DATE = 0;
   private static final int MEMORY_USED = 6;
   private static final int GPU_LOAD = 7;
   private static final int MEMORY_CONTROLLER_LOAD = 8;
   
   private Path logFile;
   
   private List<Entry> entries = new ArrayList<>();
   
   public GPUZLogPlotter()
   {
      logFile = DEFAULT_LOG_PATH;
      
      System.out.println(logFile);
      
      formatLogFile();
//      loadEntries();
//      
//      plotGPUUsage();
   }
   
   public void plotGPUUsage()
   {
      Plotter plotter = new Plotter();
      
      for (int i = 1; i < entries.size(); i++)
      {
         Entry lastEntry = entries.get(i - 1);
         Entry entry = entries.get(i);
         
         plotter.addArtifact(new LineArtifact(String.valueOf(i), new Point2D(i - 1, lastEntry.gpuLoad), new Point2D(i, entry.gpuLoad)));
         plotter.addArtifact(new LineArtifact(String.valueOf(i), new Point2D(i - 1, lastEntry.memoryControllerLoad), new Point2D(i, entry.memoryControllerLoad)));
//         plotter.addArtifact(new LineArtifact(String.valueOf(i), new Point2d(i - 1, lastEntry.memoryUsed), new Point2d(i, entry.memoryUsed)));
      }
      
      plotter.setPreferredSize(1280, 720);
      plotter.setViewRange(entries.size() / 2);
      plotter.setFocusPointX((double) (entries.size() / 2));
      plotter.showInNewWindow();
      
   }
   
   private class Entry
   {
      double memoryUsed;
      double gpuLoad;
      double memoryControllerLoad;
   }
   
   public void loadEntries()
   {
      List<String> lines = FileTools.readAllLines(logFile, DefaultExceptionHandler.PRINT_STACKTRACE);
      
      entries.clear();
      
      for (int i = 1; i < lines.size(); i++)
      {  
         String string = lines.get(i);
         
         String[] split = string.split(",");
         
         Entry entry = new Entry();
         
         for (int j = 0; j < split.length; j++)
         {
            String entryString = split[j];
            
            switch (j)
            {
            case MEMORY_USED:
               entry.memoryUsed = Double.valueOf(entryString);
               break;
            case GPU_LOAD:
               entry.gpuLoad = Double.valueOf(entryString);
               break;
            case MEMORY_CONTROLLER_LOAD:
               entry.memoryControllerLoad = Double.valueOf(entryString);
               break;
            }
         }
         
         entries.add(entry);
      }
   }
   
   public void formatLogFile()
   {
      List<String> lines = FileTools.readAllLines(logFile, DefaultExceptionHandler.PRINT_STACKTRACE);
      
      List<String> outputLines = new ArrayList<>();
      
      for (String line : lines)
      {
         String outputLine = "";
         
         String[] split = line.split(",");
         
         for (String value : split)
         {
            value = value.trim();
            
            outputLine += value + ", ";
         }
         
         outputLine = outputLine.substring(0, outputLine.length() - 2);
         outputLines.add(outputLine);
      }
      
      FileTools.writeAllLines(outputLines, logFile, WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_STACKTRACE);
   }
   
   public static void main(String[] args)
   {
      new GPUZLogPlotter();
   }
}
