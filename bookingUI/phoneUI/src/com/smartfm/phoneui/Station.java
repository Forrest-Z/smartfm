package com.smartfm.phoneui;

import com.google.android.maps.GeoPoint;

public class Station {

	public String name;
	public GeoPoint latlon;

	public Station(String name, double lat, double lon) {
		this.name = name;
		this.latlon = new GeoPoint((int) (lat * 1E6), (int) (lon * 1E6));
	}
}
