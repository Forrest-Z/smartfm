package com.steve.smartfm;

public class Task {
	
	/*
	 * store Service information
	 */
	
	public String ID = "-1";
	public String taskID = "-1";
	public String pickupOption = "0";
	public String dropoffOption = "0";
	public String waitTime = "0";
	public String carID = "0";
	public String dropoffLocation = "0";
	public String pickupLocation = "0";
	
	public Task(){
		
	}
	
	public Task(String ID, String taskID, String pickup_op, String dest_op, String wait, 
			String carID, String pickup_location, String dest_location) {
		this.ID = ID;
		this.taskID = taskID;
		this.pickupOption = pickup_op;
		this.dropoffOption = dest_op;
		this.waitTime = wait;
		this.carID = carID;
		
		this.pickupLocation = pickup_location;
		this.dropoffLocation = dest_location;
	}	
}