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

	MapController mc;
	MapView mapView;
	StationList stations;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.bookingmap);

		initializeMapView();
		placePin();
	}

	public class MapLocationOverlay extends Overlay {
		
		Bitmap pin;

		@Override
		public void draw(Canvas canvas, MapView mapView, boolean shadow) {
			super.draw(canvas, mapView, shadow);
			pin = BitmapFactory.decodeResource(getResources(), R.drawable.pin);
			for( Station s: stations.getStations() ) {
				Point p = mapView.getProjection().toPixels(s.latlon, null);
				canvas.drawBitmap(pin, p.x - pin.getWidth() / 2,
						p.y - pin.getHeight(), null);
			}
		}

		@Override
		public boolean onTap(GeoPoint hit, MapView mapView) {
			Intent intent = new Intent(getApplicationContext(),
					TaskBooking.class);
			
			for( Station s: stations.getStations() ) {
				Point p = mapView.getProjection().toPixels(s.latlon, null);
				RectF hitTestRect = new RectF(p.x-pin.getWidth() / 2, 
						p.y-pin.getHeight(), p.y+pin.getWidth() / 2, p.y);
				Point pt = mapView.getProjection().toPixels(hit, null);
				if ( hitTestRect.contains(pt.x, pt.y) ) {
					intent.putExtra("com.smartfm.phoneui.stationName", s.name);
					finish();
				}
			}

			return super.onTap(hit, mapView);
		}
	}

	private void placePin() {
		MapLocationOverlay mapLocationOverlay = new MapLocationOverlay();
		mapView.getOverlays().clear();
		mapView.getOverlays().add(mapLocationOverlay);
		mapView.invalidate();
	}

	private void initializeMapView() {
		mapView = (MapView) findViewById(R.id.mapView);
		mapView.setBuiltInZoomControls(true);
		mc = mapView.getController();
		mc.animateTo(new GeoPoint(1299631, 103771007));
		mc.setZoom(18);
	}

	@Override
	protected boolean isRouteDisplayed() {
		return false;
	}
}