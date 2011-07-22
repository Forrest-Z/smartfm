package com.leon.golfcar;

public class Task {
	
	/*
	 * store Service information
	 */
	
	public String ID = "-1";
	public String taskID = "-1";
	public String pickUpOption = "0";
	public String dropOffOption = "0";
	public String waitTime = "0";
	public String carID = "0";
	public String dropOffLocation = "0";
	public String pickUpLocation = "0";
	
	public Task(){
		
	}
	
	public Task(String ID, String taskID, String pickUpOption, String dropOffOption, String waitTime, 
			String carID, String pickUpLocation, String dropOffLocation) {
		this.ID = ID;
		this.taskID = taskID;
		this.pickUpOption = pickUpOption;
		this.dropOffOption = dropOffOption;
		this.waitTime = waitTime;
		this.carID = carID;
		
		this.pickUpLocation = pickUpLocation;
		this.dropOffLocation = dropOffLocation;
	}	
	
	public String convertTaskInfoToString(){
		return ID + ":" + taskID + ":" + pickUpOption + ":" + dropOffOption + ":"
				+ waitTime + ":" + carID + ":" + pickUpLocation + ":" + dropOffLocation;
	}
}


