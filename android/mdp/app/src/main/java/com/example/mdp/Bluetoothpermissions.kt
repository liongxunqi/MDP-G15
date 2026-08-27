package com.example.mdp


import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.content.ContextCompat
import android.util.Log

/**
 * The Bluetooth permission model changed at Android 12 (API 31). Before you can
 * scan, connect, or even read device.name, these must be granted AT RUNTIME —
 * declaring them in the manifest is not enough. This is the #1 "why won't it
 * scan / why is device.name null" gotcha.
 */
object BluetoothPermissions {

    val required: Array<String>
        get() = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Android 12+
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN,
            )
        } else {
            // Android 11 and below: discovery needs location permission.
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION,
            )
        }

    fun allGranted(context: Context): Boolean =
        required.all { permission ->
            val granted = ContextCompat.checkSelfPermission(context, permission) == PackageManager.PERMISSION_GRANTED
            Log.d("BT", "$permission granted: $granted")

            // This MUST be the last line so the lambda returns this Boolean result
            granted
        }
}