package com.smartfm.phoneui;

import com.google.android.maps.GeoPoint;

public class VehicleInfo {

    public String vehicleID = "";
    public String status = "";
	public double latitude = 0.0;
	public double longitude = 0.0;
	public int eta = 0;
	public String currentLocation = "";
	public String tooltip = "";
	public int requestID = 0;
	
	public GeoPoint getLatLon() {
		return new GeoPoint((int) (latitude * 1E6), 
				(int) (longitude * 1E6));
	}
}
