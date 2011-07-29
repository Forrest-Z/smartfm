package fmui;

import java.io.IOException;
import java.net.Socket;

class UserData extends ClientData{
	private DataFromUserToServer dataToServer = new DataFromUserToServer();
	private DataFromServerToUser dataToUser = new DataFromServerToUser();
	private int userID = -1;
	
	public UserData(Socket socket) throws IOException {
		super(socket);
	}
	
	public boolean readUserData() throws IOException {
		String data = null;
		boolean isReady = false;
		
		data = super.readData();
		if (data != null) {
			if (data.startsWith(";")) {
            	int f = data.indexOf(":");
            	int s = data.indexOf(":", f+1);
            	int t = data.indexOf(":", s+1);
            	
            	userID = Integer.parseInt(data.substring(1,f));
            	dataToServer.taskID = Integer.parseInt(data.substring(f+1,s));
            	dataToServer.pickupOption = Integer.parseInt(data.substring(s+1,t));
            	dataToServer.dropoffOption = Integer.parseInt(data.substring(t+1));
			}
			isReady = true;
		}
		
		return isReady;
	}
	
	public void sendDataToUser(String dataSentToUser) {
		super.sendData(dataSentToUser);
	}
	
	public boolean isTaskRequestValid() {
		return ((dataToServer != null) && (dataToServer.pickupOption !=0) && (dataToServer.dropoffOption !=0) )? true : false;
	}
	
	public int getID() {
		return userID;
	}
	
	public int getTaskID() {
		return dataToServer.taskID;
	}
	
	public int getPickupOption() {
		return dataToServer.pickupOption;
	}
	
	public int getDropoffOption() {
		return dataToServer.dropoffOption;
	}
	
	public int getWaitTime() {
		return dataToUser.waitTime;
	}
	
	public int getCarID() {
		return dataToUser.carID;
	}
	
	public DataFromUserToServer getDataToServer() {
		return dataToServer;
	}
	
	public void setPickupOption(int pickupOption) {
		dataToServer.pickupOption = pickupOption;
	}
	
	public void setDropoffOption(int dropoffOption) {
		dataToServer.dropoffOption = dropoffOption;
	}
	
	public void setWaitTime(int waitTime) {
		dataToUser.waitTime = waitTime;
	}
	
	public void setCarID(int carID) {
		dataToUser.carID = carID;
	}
	
	public void clean(){
		dataToServer = null;
	}
}
