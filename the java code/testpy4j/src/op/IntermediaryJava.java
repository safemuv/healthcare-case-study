package op;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.lang.model.element.Element;

public class IntermediaryJava {
	
	
	public void receiveRobotData(List data) {
		//System.out.println(data);//[robotName,[xPos,yPos],[speed],[isTrapped][piRetry]]
	}
	
	/*method that sends coordinates to the python script*/
	public List getCoordinates(String robotName) {
		List<String> rooms = new ArrayList<>();
		/*System.out.println("sending coordinate to python...");
		double cor[][]= {{4.61,-3.80},{3.51,-3.73},{2.66,-3.69}};//(x,y) coordinates of each room
		int randRoom = (int)(Math.random()*cor.length);//generates a random number, is the position of the room in "cor"
		List room= new ArrayList();//the coordinate of the selected room in List type
		room.add(cor[randRoom][0]);
		room.add(cor[randRoom][1]);
		return room;//returns type List because is the type in python*/
		CoordinateReader cr=new CoordinateReader();
		if(robotName.equals("/tb3_0/")) {
			rooms=cr.setOfRoomsR1();//send back a fixed set of rooms (3 rooms) for now is not sending back based on the names of the robots
		}
		else if(robotName.equals("/tb3_1/")) {
			rooms=cr.setOfRoomsR2();//send back a fixed set of rooms (3 rooms) for now is not sending back based on the names of the robots
		}
		
		else if(robotName.equals("/tb3_2/")) {
			rooms=cr.setOfRoomsR3();//send back a fixed set of rooms (3 rooms) for now is not sending back based on the names of the robots
		}
	    return rooms;
	  }
	
	
	public List getFailureIntervals(String robotName) {
		FailureIntervalsReader fir =new FailureIntervalsReader();
		return fir.readFromFile(robotName);
		
	  }
	
	
	
}
