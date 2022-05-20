package main;
import java.util.List;

import op.MapNav;
import py4j.GatewayServer;

public class JavaCaller {
	public static void main(String[] args) {
        GatewayServer.turnLoggingOff();
        GatewayServer server = new GatewayServer();
        server.start();
        MapNav map = (MapNav) server.getPythonServerEntryPoint(new Class[] { MapNav.class });
        try {
        	
            map.sayHello();
            List a=map.goalStatus();
            System.out.println(a);
            //map.cancelGoal();
            
        } catch (Exception e) {
            e.printStackTrace();
        }
        server.shutdown();
  }
}
