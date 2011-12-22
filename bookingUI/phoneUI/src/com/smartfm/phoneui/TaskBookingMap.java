package com.smartfm.phoneui;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.Overlay;

import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Point;
import android.graphics.RectF;
import android.os.Bundle;

public class TaskBookingMap extends MapActivity {
	
	//TODO add current location to the map and center it there (check that this does not violate the terms of use).
	
	//TODO it seems that the proper method of adding overlays is to use ItemizedOverlays. 
	// (see http://developer.android.com/resources/tutorials/views/hello-mapview.html)

	MapController mc;
	MapView mapView;
	StationList stations;
	Bitmap pin;
	
	class MapLocationOverlay extends Overlay {
		
		@Override
		public void draw(Canvas canvas, MapView mapView, boolean shadow) {
			super.draw(canvas, mapView, shadow);
			for( Station s: stations.getStations() ) {
				Point p = mapView.getProjection().toPixels(s.latlon, null);
				canvas.drawBitmap(pin, p.x - pin.getWidth() / 2,
						p.y - pin.getHeight(), null);
			}
		}
		
		boolean test(GeoPoint hit, Station s) {
			Point p_hit = mapView.getProjection().toPixels(hit, null);
			RectF r = new RectF(-pin.getWidth()/2, -pin.getHeight(), pin.getWidth()/2, 0);
			Point p_s = mapView.getProjection().toPixels(s.latlon, null);
			r.offset(p_s.x, p_s.y);
			return r.contains(p_hit.x, p_hit.y);
		}

		@Override
		public boolean onTap(GeoPoint hit, MapView mapView) {
			for( Station s: stations.getStations() ) {
				if ( test(hit, s) ) {
					setResult(RESULT_OK, new Intent().
							putExtra("com.smartfm.phoneui.stationName", s.name));
					finish();
					return true;
				}
			}
			return super.onTap(hit, mapView);
		}
	}

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.bookingmap);
		pin = BitmapFactory.decodeResource(getResources(), R.drawable.pin);
		
		stations = new StationList();

		mapView = (MapView) findViewById(R.id.mapView);
		mapView.setBuiltInZoomControls(true);
		mc = mapView.getController();
		mc.animateTo(new GeoPoint(1299631, 103771007));
		mc.setZoom(18);
		
		MapLocationOverlay mapLocationOverlay = new MapLocationOverlay();
		mapView.getOverlays().clear();
		mapView.getOverlays().add(mapLocationOverlay);
		mapView.invalidate();
	}

	@Override
	protected boolean isRouteDisplayed() {
		return false;
	}
}