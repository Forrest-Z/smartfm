package com.smartfm.phoneui;

import java.util.Collections;
import java.util.ArrayList;
import java.util.List;

public class StationList {

	private ArrayList<Station> stations = null;
	private ArrayList<String> names = null;
	
	public StationList() {
		stations = new ArrayList<Station>();
		// This is identical to the entries in the database
		stations.add(new Station("E3A", 1.300500, 103.771592));
		stations.add(new Station("EA", 1.300730, 103.770844));
		stations.add(new Station("DCC Workshop", 1.299121, 103.770788));
		stations.add(new Station("McDonald", 1.298116, 103.771209));
	}
	
	public List<Station> getStations() {
		return Collections.unmodifiableList(stations);
	}
	
	public ArrayList<String> allNames() {
		if( names==null ) {
			names = new ArrayList<String>();
			for( Station s: stations )
				names.add(s.name);
		}
		return names;
	}
	
	public boolean exists(String stationName) {
		for( Station s: stations )
			if( s.name.compareTo(stationName)==0 )
				return true;
		return false;
	}
	
	public Station getStation(String stationName) {
		for( Station s: stations )
			if( s.name.compareTo(stationName)==0 )
				return s;
		throw new StationDoesNotExistException("Station " + stationName + " does not exist.");
	}
	
	public int getStationNumber(String stationName) {
		for( int i=0; i<stations.size(); i++ )
			if( stations.get(i).name.compareTo(stationName)==0 )
				return i;
		throw new StationDoesNotExistException("Station " + stationName + " does not exist.");
	}
}
