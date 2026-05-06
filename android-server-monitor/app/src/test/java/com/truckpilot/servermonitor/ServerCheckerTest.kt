package com.truckpilot.servermonitor

import org.junit.Assert.*
import org.junit.Test

class ServerCheckerTest {

    //region normalizeUrl

    @Test
    fun normalizeUrl_keepsHttpsPrefix() {
        assertEquals("https://example.com", ServerChecker.normalizeUrl("https://example.com"))
    }

    @Test
    fun normalizeUrl_keepsHttpPrefix() {
        assertEquals("http://example.com", ServerChecker.normalizeUrl("http://example.com"))
    }

    @Test
    fun normalizeUrl_addsHttpWhenMissing() {
        assertEquals("http://example.com", ServerChecker.normalizeUrl("example.com"))
    }

    @Test
    fun normalizeUrl_addsHttpToEmptyString() {
        assertEquals("http://", ServerChecker.normalizeUrl(""))
    }

    @Test
    fun normalizeUrl_preservesPathAndQuery() {
        val input = "example.com/api/v1?test=1"
        assertEquals("http://example.com/api/v1?test=1", ServerChecker.normalizeUrl(input))
    }

    //endregion

    //region CheckResult sealed class behaviour

    @Test
    fun onlineResultStoresResponseCode() {
        val result = CheckResult.Online(200)
        assertEquals(200, result.responseCode)
    }

    @Test
    fun offlineResultStoresReason() {
        val result = CheckResult.Offline("Timeout")
        assertEquals("Timeout", result.reason)
    }

    @Test
    fun onlineResultsAreEqualWithSameCode() {
        assertEquals(CheckResult.Online(200), CheckResult.Online(200))
    }

    @Test
    fun onlineResultsAreNotEqualWithDifferentCode() {
        assertNotEquals(CheckResult.Online(200), CheckResult.Online(404))
    }

    @Test
    fun offlineResultsAreEqualWithSameReason() {
        assertEquals(CheckResult.Offline("Timeout"), CheckResult.Offline("Timeout"))
    }

    @Test
    fun offlineResultsAreNotEqualWithDifferentReason() {
        assertNotEquals(CheckResult.Offline("Timeout"), CheckResult.Offline("DNS"))
    }

    @Test
    fun onlineAndOfflineAreDifferentTypes() {
        val online: CheckResult = CheckResult.Online(200)
        val offline: CheckResult = CheckResult.Offline("Timeout")

        assertTrue(online is CheckResult.Online)
        assertTrue(offline is CheckResult.Offline)
        assertNotEquals(online, offline)
    }

    @Test
    fun onlineCopyCreatesNewInstance() {
        val original = CheckResult.Online(200)
        val copy = original.copy(responseCode = 204)
        assertEquals(204, copy.responseCode)
    }

    @Test
    fun offlineCopyCreatesNewInstance() {
        val original = CheckResult.Offline("Timeout")
        val copy = original.copy(reason = "DNS error")
        assertEquals("DNS error", copy.reason)
    }

    //endregion
}
