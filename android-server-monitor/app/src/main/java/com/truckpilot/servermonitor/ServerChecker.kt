package com.truckpilot.servermonitor

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.net.HttpURLConnection
import java.net.URL

object ServerChecker {

    /**
     * Prüft ob der Server erreichbar ist via HTTP HEAD request.
     * Timeout: 10s connect, 10s read.
     * Redirects werden manuell verfolgt (HEAD bleibt HEAD).
     */
    suspend fun check(urlString: String): CheckResult = withContext(Dispatchers.IO) {
        try {
            val url = URL(normalizeUrl(urlString))
            checkWithRedirects(url, 0)
        } catch (e: java.net.UnknownHostException) {
            CheckResult.Offline("Host nicht gefunden")
        } catch (e: java.net.SocketTimeoutException) {
            CheckResult.Offline("Timeout")
        } catch (e: java.io.IOException) {
            CheckResult.Offline(e.message ?: "Netzwerkfehler")
        } catch (e: Exception) {
            CheckResult.Offline(e.message ?: "Unbekannter Fehler")
        }
    }

    private fun checkWithRedirects(url: URL, redirectCount: Int): CheckResult {
        if (redirectCount > 5) return CheckResult.Offline("Zu viele Redirects")

        val connection = (url.openConnection() as HttpURLConnection).apply {
            connectTimeout = 10_000
            readTimeout = 10_000
            requestMethod = "HEAD"
            instanceFollowRedirects = false
        }

        return try {
            val responseCode = connection.responseCode

            if (responseCode in 300..399) {
                val location = connection.getHeaderField("Location")
                connection.disconnect()
                if (location != null) {
                    val redirectUrl = URL(url, location)
                    checkWithRedirects(redirectUrl, redirectCount + 1)
                } else {
                    CheckResult.Offline("Redirect ohne Location")
                }
            } else if (responseCode in 200..299) {
                CheckResult.Online(responseCode)
            } else {
                CheckResult.Offline("HTTP $responseCode")
            }
        } finally {
            try { connection.disconnect() } catch (_: Exception) {}
        }
    }

    internal fun normalizeUrl(url: String): String {
        return if (url.startsWith("http://") || url.startsWith("https://")) {
            url
        } else {
            "http://$url"
        }
    }
}

sealed class CheckResult {
    data class Online(val responseCode: Int) : CheckResult()
    data class Offline(val reason: String) : CheckResult()
}
