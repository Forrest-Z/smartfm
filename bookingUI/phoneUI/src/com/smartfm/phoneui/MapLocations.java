package com.smartfm.phoneui;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.MyLocationOverlay;
import com.google.android.maps.Overlay;
import com.google.android.maps.OverlayItem;


public class MapLocations extends MapActivity {	
	
	class StationOverlayItem extends OverlayItem {
		public Station mStation;
		StationOverlayItem(Station station) {
			super(station.latlon, station.name, station.name);
			mStation = station;
		}
	}	

	
	class StationItemizedOverlay extends ItemizedOverlay<StationOverlayItem> {
		
		ArrayList<StationOverlayItem> mOverlays = new ArrayList<StationOverlayItem>();
		
		public StationItemizedOverlay(Drawable defaultMarker) {
			super(boundCenterBottom(defaultMarker));
		}
		
		public void addStation(Station s) {
		    mOverlays.add(new StationOverlayItem(s));
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
	}
	
	
	class VehicleOverlayItem extends OverlayItem {
		public VehicleInfo vehicle = null;
		public VehicleOverlayItem(VehicleInfo v) {
			super(v.getLatLon(), v.vehicleID, "snippet");
			vehicle = v;
		}
	}
	
	
	class VehicleItemizedOverlay extends ItemizedOverlay<VehicleOverlayItem> {
		
		ArrayList<VehicleOverlayItem> mOverlays = new ArrayList<VehicleOverlayItem>();
		
		public VehicleItemizedOverlay(Drawable defaultMarker) {
			super(boundCenterBottom(defaultMarker));
		}
		
		public void addVehicle(VehicleInfo v) {
		    mOverlays.add(new VehicleOverlayItem(v));
		    populate();
		}
		
		public void update(List<VehicleInfo> vehicles) {
			mOverlays.clear();
			for(VehicleInfo v: vehicles)
				mOverlays.add(new VehicleOverlayItem(v));
			populate();				
		}
		
		@Override
		protected VehicleOverlayItem createItem(int i) {
		  return mOverlays.get(i);
		}
		
		@Override
		public int size() {
		  return mOverlays.size();
		}
	}

	MapController mc;
	MapView mapView;
	StationList stations;
	MyLocationOverlay userlocation;
	VehicleItemizedOverlay vItemizedoverlay;
	Handler handler = new Handler();


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
		
		Drawable redpin = this.getResources().getDrawable(R.drawable.pinred);
		StationItemizedOverlay itemizedoverlay = new StationItemizedOverlay(redpin);
		for( Station s: stations.getStations())
			itemizedoverlay.addStation(s);
		mapOverlays.add(itemizedoverlay);
				
		Drawable bluepin = this.getResources().getDrawable(R.drawable.pinblue);
		vItemizedoverlay = new VehicleItemizedOverlay(bluepin);
		updateVehicles();
		mapOverlays.add(vItemizedoverlay);
				
		startPeriodicUpdate();
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
	
	void updateVehicles() {
		try {
			LinkedList<VehicleInfo> vehicles = DBInterface.listVehicles();
			for(Iterator<VehicleInfo> it = vehicles.iterator(); it.hasNext(); ) {
				VehicleInfo v = it.next();
				if(v.latitude==0 || v.longitude==0)
					it.remove();
			}
			vItemizedoverlay.update(vehicles);
		} catch (Exception e) {
			// problem retrieving tasks... do nothing for now.
		}
		mapView.postInvalidate();
	}
	
	public void startPeriodicUpdate() {
		Runnable periodicUpdateProcess = new Runnable(){
			public void run() {
				updateVehicles();
				handler.postDelayed(this, 3000);
			}
		};
		
		handler.removeCallbacks(periodicUpdateProcess);
		handler.postDelayed(periodicUpdateProcess, 1000);
	}

}
