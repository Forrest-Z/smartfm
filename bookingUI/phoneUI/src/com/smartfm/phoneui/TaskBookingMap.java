package com.smartfm.phoneui;

import java.util.ArrayList;
import java.util.List;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.MyLocationOverlay;
import com.google.android.maps.Overlay;
import com.google.android.maps.OverlayItem;

import android.content.Intent;
import android.graphics.drawable.Drawable;
import android.os.Bundle;

public class TaskBookingMap extends MapActivity {
	
	//TODO add current location to the map and center it there (check that this does not violate the terms of use).
	
	MapController mc;
	MapView mapView;
	StationList stations;
	MyLocationOverlay userlocation;
	
	class StationOverlayItem extends OverlayItem {
		public Station mStation;
		StationOverlayItem(Station station) {
			super(station.latlon, station.name, station.name);
			mStation = station;
		}
	}
	
	class StationItemizedOverlay extends ItemizedOverlay<StationOverlayItem> {
		
		private ArrayList<StationOverlayItem> mOverlays = new ArrayList<StationOverlayItem>();
		
		public StationItemizedOverlay(Drawable defaultMarker) {
			super(boundCenterBottom(defaultMarker));
		}
		
		public void addOverlay(StationOverlayItem overlay) {
		    mOverlays.add(overlay);
		    populate();
		}
		
		@Override
		protected StationOverlayItem createItem(int i) {
		  return mOverlays.get(i);
		}
		
		@Override
		public int size() {
		  return mOverlays.size();
		}
		
		@Override
		protected boolean onTap(int index) {
			StationOverlayItem item = mOverlays.get(index);
		 	setResult(RESULT_OK, new Intent().
		 			putExtra("com.smartfm.phoneui.stationName", item.mStation.name));
		 	finish();
		 	return true;
		}
	}

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.mapview);
		
		stations = new StationList();

		mapView = (MapView) findViewById(R.id.mapview);
		mapView.setBuiltInZoomControls(true);
		mc = mapView.getController();
		mc.animateTo(new GeoPoint(1299631, 103771007));
		mc.setZoom(18);
		
		userlocation = new MyLocationOverlay(this, mapView);
		userlocation.enableMyLocation();
		userlocation.enableCompass();
		
		List<Overlay> mapOverlays = mapView.getOverlays();
		mapOverlays.clear();
		mapOverlays.add(userlocation);
		
		Drawable drawable = this.getResources().getDrawable(R.drawable.pin);
		StationItemizedOverlay itemizedoverlay = new StationItemizedOverlay(drawable);
		for( Station s: stations.getStations())
			itemizedoverlay.addOverlay(new StationOverlayItem(s));
		mapOverlays.add(itemizedoverlay);
				
		mapView.invalidate();
	}
	
	@Override
	protected void onPause() {
		super.onPause();
		userlocation.disableCompass();
		userlocation.disableMyLocation();
	}

	@Override
	protected void onResume() {
		super.onResume();
		userlocation.enableCompass();
		userlocation.enableMyLocation();
	}

	@Override
	protected boolean isRouteDisplayed() {
		return false;
	}
}