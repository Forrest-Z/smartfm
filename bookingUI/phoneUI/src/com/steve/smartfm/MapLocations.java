package com.steve.smartfm;

import android.os.Bundle;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.MyLocationOverlay;
import com.google.android.maps.Overlay;

public class MapLocations extends MapActivity {
	
	MyLocationOverlay userlocation;
	MapController mapcontroller;
	MapView mapview;
	
	@Override
	protected void onCreate(Bundle arg0) {
		// TODO Auto-generated method stub
		super.onCreate(arg0);
		setContentView(R.layout.mapview);
		
		mapview = (MapView) findViewById(R.id.mapview);
		mapview.setBuiltInZoomControls(true);
		mapcontroller = mapview.getController();
		mapcontroller.animateTo(new GeoPoint(1299631, 103771007));
		mapcontroller.setZoom(18);
		userlocation = new MyLocationOverlay(this, mapview);
		mapview.getOverlays().clear();
		userlocation.enableMyLocation();
		userlocation.enableCompass();
		mapview.getOverlays().add(userlocation);
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
		userlocation.disableCompass();
		userlocation.disableMyLocation();
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
		userlocation.enableCompass();
		userlocation.enableMyLocation();
	}

	@Override
	protected boolean isRouteDisplayed() {
		// TODO Auto-generated method stub
		return false;
	}
	
	// ADD CAR LOCATION OVERLAY - receive data from server
	private class CarLocationOverlay extends Overlay{
	
	}

}
