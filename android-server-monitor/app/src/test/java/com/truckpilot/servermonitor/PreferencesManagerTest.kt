package com.truckpilot.servermonitor

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class PreferencesManagerTest {

    private lateinit var manager: PreferencesManager

    @Before
    fun setUp() {
        manager = PreferencesManager(InMemorySharedPreferences())
    }

    //region serverUrl

    @Test
    fun defaultServerUrl() {
        assertEquals(PreferencesManager.DEFAULT_URL, manager.serverUrl)
    }

    @Test
    fun setAndGetServerUrl() {
        manager.serverUrl = "https://myserver.example.com"
        assertEquals("https://myserver.example.com", manager.serverUrl)
    }

    //endregion

    //region checkIntervalSeconds – coercion & snapping

    @Test
    fun defaultCheckInterval() {
        assertEquals(PreferencesManager.DEFAULT_INTERVAL, manager.checkIntervalSeconds)
    }

    @Test
    fun checkIntervalCoercesBelowMin() {
        manager.checkIntervalSeconds = 5
        assertEquals(PreferencesManager.MIN_INTERVAL, manager.checkIntervalSeconds)
    }

    @Test
    fun checkIntervalCoercesAboveMax() {
        manager.checkIntervalSeconds = 1000
        assertEquals(PreferencesManager.MAX_INTERVAL, manager.checkIntervalSeconds)
    }

    @Test
    fun checkIntervalSnapsToFiveSecondGrid() {
        manager.checkIntervalSeconds = 17
        assertEquals(15, manager.checkIntervalSeconds)

        manager.checkIntervalSeconds = 22
        assertEquals(20, manager.checkIntervalSeconds)

        manager.checkIntervalSeconds = 33
        assertEquals(30, manager.checkIntervalSeconds)
    }

    //endregion

    //region isServiceRunning

    @Test
    fun isServiceRunningDefaultsToFalse() {
        assertFalse(manager.isServiceRunning)
    }

    @Test
    fun setAndGetIsServiceRunning() {
        manager.isServiceRunning = true
        assertTrue(manager.isServiceRunning)
    }

    //endregion

    //region lastCheckOnline (nullable)

    @Test
    fun lastCheckOnlineIsNullByDefault() {
        assertNull(manager.lastCheckOnline)
    }

    @Test
    fun setAndGetLastCheckOnlineTrue() {
        manager.lastCheckOnline = true
        assertEquals(true, manager.lastCheckOnline)
    }

    @Test
    fun setAndGetLastCheckOnlineFalse() {
        manager.lastCheckOnline = false
        assertEquals(false, manager.lastCheckOnline)
    }

    @Test
    fun lastCheckOnlineNullRemovesKey() {
        manager.lastCheckOnline = true
        assertNotNull(manager.lastCheckOnline)

        manager.lastCheckOnline = null
        assertNull(manager.lastCheckOnline)
    }

    //endregion

    //region initLastStateFromSharedPrefs

    @Test
    fun initLastStateFromSharedPrefsDoesNotCrash() {
        // Dokumentiert als No-Op – Getter liest bereits direkt aus SharedPreferences
        manager.initLastStateFromSharedPrefs()
        assertNull(manager.lastCheckOnline)
    }

    //endregion
}
