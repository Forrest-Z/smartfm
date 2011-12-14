package com.smartfm.phoneui;

import org.apache.commons.lang3.builder.HashCodeBuilder;


public class Task {
	public int requestID;
	public String customerID;
    public String status;
    public String pickup;
    public String dropoff;
    public String vehicleID;
	public double latitude;
	public double longitude;
	public int eta;
	
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

}
