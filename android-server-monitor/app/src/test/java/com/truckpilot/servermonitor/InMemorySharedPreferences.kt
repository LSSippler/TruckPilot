package com.truckpilot.servermonitor

import android.content.SharedPreferences

/**
 * In-Memory-Implementierung von SharedPreferences für Unit-Tests.
 * Kein Android-Framework nötig – funktioniert mit der Standard-[SharedPreferences]-API.
 */
class InMemorySharedPreferences : SharedPreferences {

    private val map = mutableMapOf<String, Any?>()

    override fun getAll(): Map<String, *> = map.toMap()

    override fun getString(key: String, defValue: String?): String? =
        map[key] as? String ?: defValue

    override fun getStringSet(key: String, defValues: Set<String>?): Set<String>? =
        map[key] as? Set<String> ?: defValues

    override fun getInt(key: String, defValue: Int): Int =
        map[key] as? Int ?: defValue

    override fun getLong(key: String, defValue: Long): Long =
        map[key] as? Long ?: defValue

    override fun getFloat(key: String, defValue: Float): Float =
        map[key] as? Float ?: defValue

    override fun getBoolean(key: String, defValue: Boolean): Boolean =
        map[key] as? Boolean ?: defValue

    override fun contains(key: String): Boolean = map.containsKey(key)

    override fun edit(): SharedPreferences.Editor = InMemoryEditor(map)

    override fun registerOnSharedPreferenceChangeListener(
        listener: SharedPreferences.OnSharedPreferenceChangeListener?
    ) {
    }

    override fun unregisterOnSharedPreferenceChangeListener(
        listener: SharedPreferences.OnSharedPreferenceChangeListener?
    ) {
    }

    private class InMemoryEditor(
        private val map: MutableMap<String, Any?>
    ) : SharedPreferences.Editor {

        private val pending = mutableMapOf<String, Any?>()
        private var clearAll = false

        override fun putString(key: String, value: String?): SharedPreferences.Editor =
            apply { pending[key] = value }

        override fun putStringSet(key: String, values: Set<String>?): SharedPreferences.Editor =
            apply { pending[key] = values }

        override fun putInt(key: String, value: Int): SharedPreferences.Editor =
            apply { pending[key] = value }

        override fun putLong(key: String, value: Long): SharedPreferences.Editor =
            apply { pending[key] = value }

        override fun putFloat(key: String, value: Float): SharedPreferences.Editor =
            apply { pending[key] = value }

        override fun putBoolean(key: String, value: Boolean): SharedPreferences.Editor =
            apply { pending[key] = value }

        override fun remove(key: String): SharedPreferences.Editor =
            apply { pending[key] = null }

        override fun clear(): SharedPreferences.Editor =
            apply { clearAll = true }

        override fun commit(): Boolean {
            apply()
            return true
        }

        override fun apply() {
            if (clearAll) {
                map.clear()
            }
            pending.forEach { (key, value) ->
                if (value == null) {
                    map.remove(key)
                } else {
                    map[key] = value
                }
            }
            pending.clear()
            clearAll = false
        }
    }
}
