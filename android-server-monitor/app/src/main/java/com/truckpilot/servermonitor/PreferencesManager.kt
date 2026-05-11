package com.truckpilot.servermonitor

import android.content.Context
import android.content.SharedPreferences
import androidx.core.content.edit

class PreferencesManager {

    private val prefs: SharedPreferences

    constructor(context: Context) {
        prefs = context.getSharedPreferences("server_monitor", Context.MODE_PRIVATE)
    }

    internal constructor(prefs: SharedPreferences) {
        this.prefs = prefs
    }

    var serverUrl: String
        get() = prefs.getString(KEY_URL, DEFAULT_URL)!!
        set(value) = prefs.edit { putString(KEY_URL, value) }

    var checkIntervalSeconds: Int
        get() = prefs.getInt(KEY_INTERVAL, DEFAULT_INTERVAL)
        set(value) {
            val snapped = (value.coerceIn(MIN_INTERVAL, MAX_INTERVAL) / 5) * 5
            prefs.edit { putInt(KEY_INTERVAL, snapped) }
        }

    var isServiceRunning: Boolean
        get() = prefs.getBoolean(KEY_SERVICE_RUNNING, false)
        set(value) = prefs.edit { putBoolean(KEY_SERVICE_RUNNING, value) }

    var lastCheckOnline: Boolean?
        get() = if (prefs.contains(KEY_LAST_ONLINE)) {
            prefs.getBoolean(KEY_LAST_ONLINE, false)
        } else null
        set(value) {
            if (value != null) {
                prefs.edit { putBoolean(KEY_LAST_ONLINE, value) }
            } else {
                prefs.edit { remove(KEY_LAST_ONLINE) }
            }
        }

    /**
     * Nach Prozess-Neustart: Aus SharedPreferences laden, ob der Server
     * zuletzt online war, damit der erste Check einen Zustandswechsel erkennen kann.
     */
    fun initLastStateFromSharedPrefs() {
        // No-Op: lastCheckOnline getter liest bereits aus SharedPreferences
    }

    companion object {
        private const val KEY_URL = "server_url"
        private const val KEY_INTERVAL = "check_interval"
        private const val KEY_SERVICE_RUNNING = "service_running"
        private const val KEY_LAST_ONLINE = "last_check_online"

        const val DEFAULT_URL = "http://100.103.91.121:8080"
        const val DEFAULT_INTERVAL = 60
        const val MIN_INTERVAL = 15
        const val MAX_INTERVAL = 600
    }
}
