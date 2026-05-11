package com.truckpilot.servermonitor

import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.os.IBinder
import android.os.PowerManager
import androidx.core.app.NotificationCompat
import androidx.lifecycle.LifecycleService
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

class MonitorService : LifecycleService() {

    private lateinit var prefs: PreferencesManager
    private lateinit var notif: NotificationHelper
    private var monitoringJob: Job? = null
    private var wakeLock: PowerManager.WakeLock? = null

    override fun onCreate() {
        super.onCreate()
        prefs = PreferencesManager(this)
        prefs.initLastStateFromSharedPrefs()
        notif = NotificationHelper(this)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // Bei System-Reset (intent==null) nicht automatisch neu starten,
        // wenn der Nutzer den Service zuvor explizit gestoppt hat.
        if (intent == null) {
            if (!prefs.isServiceRunning) {
                stopSelf()
                return START_NOT_STICKY
            }
            restartMonitoringAfterSystemKill()
            return START_STICKY
        }

        when (intent.action) {
            ACTION_START -> startMonitoring()
            ACTION_STOP -> stopMonitoring()
            ACTION_CHECK_NOW -> triggerImmediateCheck()
        }

        return super.onStartCommand(intent, flags, startId)
    }

    override fun onBind(intent: Intent): IBinder? = super.onBind(intent)

    private fun startMonitoring() {
        if (monitoringJob?.isActive == true) return

        val notification = buildForegroundNotification()
        startForeground(NOTIFICATION_ID, notification)

        acquireWakeLock()

        prefs.isServiceRunning = true

        monitoringJob = lifecycleScope.launch {
            while (isActive) {
                renewWakeLock()
                performCheck()
                delay(prefs.checkIntervalSeconds * 1000L)
            }
        }
    }

    private fun stopMonitoring() {
        monitoringJob?.cancel()
        monitoringJob = null
        releaseWakeLock()
        prefs.isServiceRunning = false
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    /**
     * Wenn der Service vom System gekillt und durch START_STICKY neu erstellt wurde,
     * nimm die Überwachung wieder auf.
     */
    private fun restartMonitoringAfterSystemKill() {
        lifecycleScope.launch {
            delay(500)
            startMonitoring()
        }
    }

    private suspend fun performCheck() {
        val url = prefs.serverUrl
        val result = ServerChecker.check(url)

        val isOnline = result is CheckResult.Online
        val lastState = prefs.lastCheckOnline

        if (lastState != null) {
            if (lastState && !isOnline) {
                notif.showOfflineNotification(url)
            } else if (!lastState && isOnline) {
                notif.showOnlineNotification(url)
            }
        }

        prefs.lastCheckOnline = isOnline
        updateForegroundNotification(url, isOnline)
    }

    private fun triggerImmediateCheck() {
        lifecycleScope.launch {
            performCheck()
        }
    }

    private fun buildForegroundNotification(
        serverUrl: String? = null,
        isOnline: Boolean? = null
    ): android.app.Notification {
        val title = serverUrl?.let {
            getString(R.string.service_monitoring, it)
        } ?: getString(R.string.service_starting)

        val contentText = when (isOnline) {
            true -> getString(R.string.status_online)
            false -> getString(R.string.status_offline)
            null -> getString(R.string.status_checking)
        }

        val intent = Intent(this, MainActivity::class.java)
        val pendingIntent = PendingIntent.getActivity(
            this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        return NotificationCompat.Builder(this, ServerMonitorApp.CHANNEL_SERVICE)
            .setSmallIcon(R.drawable.ic_server_up)
            .setContentTitle(title)
            .setContentText(contentText)
            .setOngoing(true)
            .setContentIntent(pendingIntent)
            .build()
    }

    private fun updateForegroundNotification(serverUrl: String, isOnline: Boolean) {
        val notification = buildForegroundNotification(serverUrl, isOnline)
        val manager = getSystemService(Context.NOTIFICATION_SERVICE) as android.app.NotificationManager
        manager.notify(NOTIFICATION_ID, notification)
    }

    private fun acquireWakeLock() {
        val powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = powerManager.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "ServerMonitor::MonitorWakeLock"
        ).apply {
            setReferenceCounted(false)
            // Timeout = 2x Intervall, wird vor jedem Check erneuert
            acquire((prefs.checkIntervalSeconds * 2 + 30) * 1000L)
        }
    }

    private fun renewWakeLock() {
        wakeLock?.let {
            if (it.isHeld) it.release()
        }
        wakeLock = null
        acquireWakeLock()
    }

    private fun releaseWakeLock() {
        wakeLock?.let {
            if (it.isHeld) it.release()
        }
        wakeLock = null
    }

    override fun onDestroy() {
        monitoringJob?.cancel()
        monitoringJob = null
        releaseWakeLock()
        super.onDestroy()
    }

    companion object {
        const val ACTION_START = "com.truckpilot.servermonitor.START"
        const val ACTION_STOP = "com.truckpilot.servermonitor.STOP"
        const val ACTION_CHECK_NOW = "com.truckpilot.servermonitor.CHECK_NOW"

        private const val NOTIFICATION_ID = 1001
    }
}
