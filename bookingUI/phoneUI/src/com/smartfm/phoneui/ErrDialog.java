package com.smartfm.phoneui;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;

public class ErrDialog {

	public static AlertDialog show(Context context, String msg) {
		AlertDialog d = new AlertDialog.Builder(context)
			.setTitle("Error")
			.setMessage(msg)
			.setCancelable(false)
			.setNeutralButton("OK", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int id) {
					dialog.dismiss();
				}
			}).create();
		d.show();
		return d;
	}
}
