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
	Bitmap pin;
	MapLocation Macs, WS, EA, E3A;
	Point pointMacs = new Point(), pointWS = new Point(), pointEA = new Point(), pointE3A = new Point();

	/** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle bundle) {
        super.onCreate(bundle);
        setContentView(R.layout.bookingmap);

        initializeMapView();
        defineGeoPoints();
        placePin();
    }

    public class MapLocationOverlay extends Overlay{

		@Override
		public void draw(Canvas canvas, MapView mapView, boolean shadow) {
			// TODO Auto-generated method stub
			super.draw(canvas, mapView, shadow);
			pin = BitmapFactory.decodeResource(getResources(), R.drawable.pin);
			mapView.getProjection().toPixels(Macs.getLocation(), pointMacs);
			mapView.getProjection().toPixels(WS.getLocation(), pointWS);
			mapView.getProjection().toPixels(EA.getLocation(), pointEA);
			mapView.getProjection().toPixels(E3A.getLocation(), pointE3A);
			canvas.drawBitmap(pin, pointMacs.x-pin.getWidth()/2, pointMacs.y-pin.getHeight(), null);
			canvas.drawBitmap(pin, pointWS.x-pin.getWidth()/2, pointWS.y-pin.getHeight(), null);
			canvas.drawBitmap(pin, pointEA.x-pin.getWidth()/2, pointEA.y-pin.getHeight(), null);
			canvas.drawBitmap(pin, pointE3A.x-pin.getWidth()/2, pointE3A.y-pin.getHeight(), null);
		}


		@Override
		public boolean onTap(GeoPoint hit, MapView mapView) {
			// TODO Auto-generated method stub


			Intent intent = new Intent(getApplicationContext(), TaskBooking.class);
			switch(userHitLocation(mapView, hit)){
			case 0: intent.putExtra("location", 1); setResult(RESULT_OK, intent); finish(); break;
			case 1: intent.putExtra("location", 0); setResult(RESULT_OK, intent); finish(); break;
			case 2: intent.putExtra("location", 2); setResult(RESULT_OK, intent); finish(); break;
			case 3: intent.putExtra("location", 3); setResult(RESULT_OK, intent); finish(); break;
			default: break;
			}

			return super.onTap(hit, mapView);
		}


		public int userHitLocation(MapView mapView, GeoPoint hit){
			int hitLocation = 4;
			RectF hitTestRect = new RectF();
			mapView.getProjection().toPixels(Macs.getLocation(), pointMacs);
			mapView.getProjection().toPixels(WS.getLocation(), pointWS);
			mapView.getProjection().toPixels(EA.getLocation(), pointEA);
			mapView.getProjection().toPixels(E3A.getLocation(), pointE3A);
			Point[] point = {pointMacs, pointWS, pointEA, pointE3A,new Point(0,0)};
			int i;

			for (i=0; i<=3; i++){
				hitTestRect.set(-pin.getWidth()/2, -pin.getHeight(), pin.getWidth()/2, 0);
				hitTestRect.offset(point[i].x, point[i].y);
				mapView.getProjection().toPixels(hit, point[4]);
				if (hitTestRect.contains(point[4].x, point[4].y)){
					hitLocation = i;
				}
			}

			return hitLocation;
		}
    }

	private void placePin() {
		// TODO Auto-generated method stub
		MapLocationOverlay mapLocationOverlay = new MapLocationOverlay();
		mapView.getOverlays().clear();
		mapView.getOverlays().add(mapLocationOverlay);
		mapView.invalidate();
	}

	private void defineGeoPoints() {
		// TODO Auto-generated method stub
		Macs = new MapLocation("Macs", new GeoPoint(1298132,103771083));
		WS = new MapLocation("WS", new GeoPoint(1299076,103770815));
		EA = new MapLocation("EA", new GeoPoint(1300792,103770670));
		E3A = new MapLocation("E3A", new GeoPoint(1300524,103771459));
	}

	private void initializeMapView() {
		// TODO Auto-generated method stub
		mapView = (MapView) findViewById(R.id.mapView);
        mapView.setBuiltInZoomControls(true);
        mc = mapView.getController();
        mc.animateTo(new GeoPoint(1299631, 103771007));
        mc.setZoom(18);
	}

	@Override
	protected boolean isRouteDisplayed() {

		// TODO Auto-generated method stub
		return false;
	}

	public class MapLocation{
		String name = "";
		GeoPoint location = null;

		public MapLocation(String name, GeoPoint location){
			this.name = name;
			this.location = location;
		}

		public String getName(){
			return name;
		}

		public GeoPoint getLocation(){
			return location;
		}
	}
}