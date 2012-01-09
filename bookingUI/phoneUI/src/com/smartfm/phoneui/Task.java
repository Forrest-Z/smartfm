package com.smartfm.phoneui;

import org.apache.commons.lang3.builder.HashCodeBuilder;


public class Task {

	public int requestID = 0;
	public String customerID = "";
    public String status = "";
    public String pickup = "";
    public String dropoff = "";
    public String vehicleID = "";
	public double latitude = 0.0;
	public double longitude = 0.0;
	public int eta = 0;
		
	@Override 
	public boolean equals(Object aThat) {
	    if ( this == aThat ) return true;
	    if ( !(aThat instanceof Task) ) return false;
	    Task that = (Task)aThat;
	    boolean equal = this.requestID==that.requestID && 
	    	this.pickup.compareToIgnoreCase(that.pickup)==0 &&
	    	this.dropoff.compareToIgnoreCase(that.dropoff)==0 &&
	    	this.customerID.compareToIgnoreCase(that.customerID)==0;
	    return equal;
	}
	
	@Override
	public int hashCode() {
		int hashcode = new HashCodeBuilder(17, 31).
	        append(requestID).
	        append(pickup).
	        append(dropoff).
	        append(customerID).
	        toHashCode();
        return hashcode;
    }
	
	public String toString() {
		String ser = "" + requestID + "," + customerID;
		ser += "," + status + "," + pickup + "," + dropoff;
		ser += "," + vehicleID + "," + latitude + "," + longitude + "," + eta;
		return ser;
	}
	
	public static Task fromString(String ser) {
		Task task = new Task();
		String[] tokens = ser.split(",");
		task.requestID = Integer.parseInt(tokens[0]);
		task.customerID = tokens[1];
		task.status = tokens[2];
		task.pickup = tokens[3];
		task.dropoff = tokens[4];
		task.vehicleID = tokens[5];
		task.latitude = Double.parseDouble(tokens[6]);
		task.longitude = Double.parseDouble(tokens[7]);
		task.eta = Integer.parseInt(tokens[8]);
		return task;
	}
}
