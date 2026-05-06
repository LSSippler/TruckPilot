package com.truckpilot.servermonitor

import android.app.PendingIntent
import android.app.TaskStackBuilder
import android.content.Context
import android.content.Intent
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat

class NotificationHelper(private val context: Context) {

    fun showOfflineNotification(serverUrl: String) {
        val intent = Intent(context, MainActivity::class.java)
        val pendingIntent = TaskStackBuilder.create(context).run {
            addNextIntentWithParentStack(intent)
            getPendingIntent(0, PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE)
        }

        val notification = NotificationCompat.Builder(context, ServerMonitorApp.CHANNEL_STATUS)
            .setSmallIcon(R.drawable.ic_server_down)
            .setContentTitle(context.getString(R.string.notif_offline_title))
            .setContentText(context.getString(R.string.notif_offline_body, serverUrl))
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)
            .setContentIntent(pendingIntent)
            .build()

        NotificationManagerCompat.from(context).notify(NOTIF_ID_OFFLINE, notification)
    }

    fun showOnlineNotification(serverUrl: String) {
        val intent = Intent(context, MainActivity::class.java)
        val pendingIntent = TaskStackBuilder.create(context).run {
            addNextIntentWithParentStack(intent)
            getPendingIntent(1, PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE)
        }

        val notification = NotificationCompat.Builder(context, ServerMonitorApp.CHANNEL_STATUS)
            .setSmallIcon(R.drawable.ic_server_up)
            .setContentTitle(context.getString(R.string.notif_online_title))
            .setContentText(context.getString(R.string.notif_online_body, serverUrl))
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)
            .setContentIntent(pendingIntent)
            .build()

        NotificationManagerCompat.from(context).notify(NOTIF_ID_ONLINE, notification)
    }

    companion object {
        private const val NOTIF_ID_OFFLINE = 2001
        private const val NOTIF_ID_ONLINE = 2002
    }
}
