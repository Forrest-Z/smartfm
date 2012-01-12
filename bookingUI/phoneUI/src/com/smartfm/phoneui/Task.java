package com.smartfm.phoneui;

import org.apache.commons.lang3.builder.HashCodeBuilder;


public class Task {

	public int requestID = 0;
	public String customerID = "";
    public String status = "";
    public String pickup = "";
    public String dropoff = "";
	public boolean custCancelled = false;
	public VehicleInfo vehicle = null;
	
	public String formattedETA() {
		return "in about " + vehicle.eta + " seconds";
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
			if( vehicle.eta==0 )
				description += "We are processing your booking...\n";
			else
				description += "Vehicle " + vehicle.vehicleID + " will be at pickup location " + formattedETA() + ".\n";
			return description;
		}
			
		if( vehicle.status.compareToIgnoreCase("GoingToPickupLocation")==0 ) {
            if( vehicle.eta==0 )
            	description += "Vehicle $vehicleID on the way to pickup location.</p>";
            else
            	description += "Vehicle " + vehicle.vehicleID + " will be at pickup location " + formattedETA() + ".\n";
        }
        else if( vehicle.status.compareToIgnoreCase("GoingToPickupLocation")==0 ) {
        	description += "Vehicle " + vehicle.vehicleID + " is at pickup location.\n";
        }
        else {
            if( vehicle.eta==0 )
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
		ser += "," + status + "," + pickup + "," + dropoff + "," + custCancelled;
		if( vehicle!=null ) {
			ser += "," + vehicle.vehicleID + "," + vehicle.latitude + "," + vehicle.longitude;
			ser += "," + vehicle.status + "," + vehicle.eta;
		}
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
		task.custCancelled = Boolean.parseBoolean(tokens[i++]);
		
		if( i<tokens.length ) {
			task.vehicle = new VehicleInfo();
			task.vehicle.vehicleID = tokens[i++];
			task.vehicle.latitude = Double.parseDouble(tokens[i++]);
			task.vehicle.longitude = Double.parseDouble(tokens[i++]);
			task.vehicle.status = tokens[i++];
			task.vehicle.eta = Integer.parseInt(tokens[i++]);
		}
		return task;
	}
}
