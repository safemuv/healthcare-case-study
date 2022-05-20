package main;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import op.CoordinateReader;
import op.FailureIntervalsReader;
import op.IntermediaryJava;
import py4j.GatewayServer;
/*This class allow calling JAVA FROM PYTHON*/
public class MainIntermediary {

  public static void main(String[] args) {
    IntermediaryJava intermediary =new IntermediaryJava();
    //#intermediary.getCoordinates("/tb3_0/");
    //List asd = new ArrayList();//test
    //asd=intermediary.getCoordinates();//test
    
    //intermediary is now the gateway.entry_point
    
    GatewayServer server = new GatewayServer(intermediary);
    server.start();
	
    //CoordinateReader cr=new CoordinateReader();
    //cr.setOfRooms();
    //String[] c=cr.giveCoordinate(32);
    //System.out.println(c[0]);
    //System.out.println(c[1]);
    //List<String> tb3_0 = new ArrayList<>();
    //tb3_0=intermediary.getFailureIntervals("/tb3_1/");
   //System.out.println(tb3_0);
  }
}