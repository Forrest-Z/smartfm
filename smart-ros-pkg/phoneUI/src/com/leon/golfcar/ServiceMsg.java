package com.leon.golfcar;

public class ServiceMsg {
	
	/*
	 * store Service information
	 */
	
	public String ID = "-1";
	public String taskID = "-1";
	public String pickup_op = "0";
	public String dest_op = "0";
	public String wait = "0";
	public String carID = "0";
	public String dest_location = "0";
	public String pickup_location = "0";
	
	public ServiceMsg(){
		
	}
	
	public ServiceMsg(String ID, String taskID, String pickup_op, String dest_op, String wait, 
			String carID, String pickup_location, String dest_location) {
		this.ID = ID;
		this.taskID = taskID;
		this.pickup_op = pickup_op;
		this.dest_op = dest_op;
		this.wait = wait;
		this.carID = carID;
		
		this.pickup_location = pickup_location;
		this.dest_location = dest_location;
	}	
}


