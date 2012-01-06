package com.smartfm.phoneui;

import org.apache.commons.lang3.builder.HashCodeBuilder;


public class Task {

	public int requestID = 0;
	public String customerID = "";
    public String status = "";
    public String pickup = "";
    public String dropoff = "";
    public String vehicleID = "";
    public String vehicleStatus = "";
	public double latitude = 0.0;
	public double longitude = 0.0;
	public int eta = 0;
	public boolean custCancelled = false;
	
	public String formattedETA() {
		return "in about " + eta + " seconds";
	}
	
	public String formattedDescription() {
		String description = "";
		description += "From " + pickup + " To "
				+ dropoff + "\n";
		
		if( custCancelled ) {
			description += "We are cancelling your booking...\n";
			return description;
		}
		
		if( status.compareToIgnoreCase("Acknowledged") != 0 &&
				status.compareToIgnoreCase("Confirmed") != 0 &&
				status.compareToIgnoreCase("Processing") != 0 ) {
			description += "We are processing your booking...\n";
			return description;
		}
		
		if( status.compareToIgnoreCase("Processing")!=0 ) {
			if( eta==0 )
				description += "We are processing your booking...\n";
			else
				description += "Vehicle " + vehicleID + " will be at pickup location " + formattedETA() + ".\n";
			return description;
		}
			
		if( vehicleStatus.compareToIgnoreCase("GoingToPickupLocation")==0 ) {
            if( eta==0 )
            	description += "Vehicle $vehicleID on the way to pickup location.</p>";
            else
            	description += "Vehicle " + vehicleID + " will be at pickup location " + formattedETA() + ".\n";
        }
        else if( vehicleStatus.compareToIgnoreCase("GoingToPickupLocation")==0 ) {
        	description += "Vehicle " + vehicleID + " is at pickup location.\n";
        }
        else {
            if( eta==0 )
            	description += "Going to destination.";
            else
            	description += "Arriving to destination " + formattedETA() + ".\n";
        }
		return description;
	}
		
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
		ser += "," + vehicleID + "," + latitude + "," + longitude;
		ser += "," + vehicleStatus + "," + eta + "," + custCancelled;
		return ser;
	}
	
	public static Task fromString(String ser) {
		Task task = new Task();
		String[] tokens = ser.split(",");
		int i=0;
		task.requestID = Integer.parseInt(tokens[i++]);
		task.customerID = tokens[i++];
		task.status = tokens[i++];
		task.pickup = tokens[i++];
		task.dropoff = tokens[i++];
		task.vehicleID = tokens[i++];
		task.latitude = Double.parseDouble(tokens[i++]);
		task.longitude = Double.parseDouble(tokens[i++]);
		task.vehicleStatus = tokens[i++];
		task.eta = Integer.parseInt(tokens[i++]);
		task.custCancelled = Boolean.parseBoolean(tokens[i++]);
		return task;
	}
}
