package op;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class CoordinateReader {
	private List<String> list;
	private String file;//attribute for the name of the file
	
	public CoordinateReader() {
		this.file="/home/osboxes/configurationFiles/test(originales).txt";
		this.readFromFile();
	}
	
	public void readFromFile() {
		this.list = new ArrayList<>();
		try (Stream<String> stream = Files.lines(Paths.get(this.file))) {
			this.list = stream.collect(Collectors.toList());
			//System.out.println(list);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/*This method is useful if the approach is to request room by room*/
	public String[] giveCoordinate(int room) {
		//List<String> l = new ArrayList();
		String[] parts;
		//l=this.readFromFile();
		return parts=this.list.get(room).split(",");
		
		//for(int i=0;i<l.size();i++) {
			//parts = l.get(i).split(",");
			//System.out.println(parts);
			//}
	}
	
	public List setOfRoomsR1() {
		List<String> rooms = new ArrayList<>();
		for(int i=0;i<=20;i++) {
			rooms.add(this.list.get(i));
		}
		/*rooms.add(this.list.get(0));
		rooms.add(this.list.get(1));
		rooms.add(this.list.get(2));
		rooms.add(this.list.get(3));
		rooms.add(this.list.get(4));
		rooms.add(this.list.get(5));
		rooms.add(this.list.get(6));
		rooms.add(this.list.get(7));
		rooms.add(this.list.get(8));
		rooms.add(this.list.get(9));*/
		//rooms.add(this.list.get(9));
		//rooms.add(this.list.get(18));
		return rooms;
	}
	
	public List setOfRoomsR2() {
		List<String> rooms = new ArrayList<>();
		for(int i=39;i>20;i--) {
			rooms.add(this.list.get(i));
		}
		//rooms.add(this.list.get(20));
		/*rooms.add(this.list.get(19));
		rooms.add(this.list.get(18));
		rooms.add(this.list.get(17));
		rooms.add(this.list.get(16));
		rooms.add(this.list.get(15));
		rooms.add(this.list.get(22));*/
		
		return rooms;
	}
	
	public List setOfRoomsR3() {
		List<String> rooms = new ArrayList<>();
		for(int i=20;i<40;i++) {
			rooms.add(this.list.get(i));
		}
		/*rooms.add(this.list.get(20));
		rooms.add(this.list.get(21));
		rooms.add(this.list.get(22));
		rooms.add(this.list.get(23));
		rooms.add(this.list.get(24));
		rooms.add(this.list.get(25));*/
		//rooms.add(this.list.get(21));
		
		return rooms;
	}
	
	public int getSizeOfList() {//returns the size of list, useful to know how many rooms are in total
		return this.list.size();
	}
}
