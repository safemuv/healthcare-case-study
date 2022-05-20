package op;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class FailureIntervalsReader {
	private List<String> list;
	private String file;//attribute for the name of the file
	
	
	public FailureIntervalsReader() {
		this.file="/home/osboxes/configurationFiles/failureIntervals.txt";
		//this.readFromFile(robotName);
		
	}
	/*Reads from a file the intervals and save them in a list, we look for the name of the robot in the list and returns a list according  to the received name*/
	public List<String> readFromFile(String robotName) {
		robotName=robotName.substring(1,6);
		System.out.println(robotName);
		this.list = new ArrayList<>();
		List<String> tb3_0_temp = new ArrayList<>();
		List<String> tb3_1_temp = new ArrayList<>();
		List<String> tb3_2_temp = new ArrayList<>();
		List<String> returnList = new ArrayList<>();
		try (Stream<String> stream = Files.lines(Paths.get(this.file))) {
			this.list = stream.collect(Collectors.toList());
			for(int i=0;i<=list.size()-1;i++) {
				if(list.get(i).equals("tb3_0")) {
					tb3_0_temp.add(list.get(i+1));
					
				}else if(list.get(i).equals("tb3_1")) {
					tb3_1_temp.add(list.get(i+1));
					
				}else if(list.get(i).equals("tb3_2")) {
					tb3_2_temp.add(list.get(i+1));
				}
				
			}
			switch(robotName) {
				case "tb3_0":
					returnList=tb3_0_temp;
					break;
				case "tb3_1":
					returnList=tb3_1_temp;
					break;
				case "tb3_2":
					returnList=tb3_2_temp;
					break;
					
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return returnList;
		
	}
	
	
	
}
