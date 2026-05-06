package com.truckpilot.servermonitor

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.google.android.material.slider.Slider
import com.truckpilot.servermonitor.databinding.ActivityMainBinding
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var prefs: PreferencesManager
    private var restartJob: Job? = null

    private val notificationPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) {
            doStartService()
        } else {
            Toast.makeText(this, R.string.error_notification_permission, Toast.LENGTH_LONG).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        prefs = PreferencesManager(this)

        setupUi()
        startStatusPoller()
    }

    private fun setupUi() {
        binding.urlInput.setText(prefs.serverUrl)
        binding.intervalSlider.value = prefs.checkIntervalSeconds.toFloat()
        binding.intervalLabel.text = getString(R.string.interval_seconds, prefs.checkIntervalSeconds)

        binding.intervalSlider.addOnChangeListener(Slider.OnChangeListener { _, value, fromUser ->
            if (fromUser) {
                val seconds = value.toInt()
                binding.intervalLabel.text = getString(R.string.interval_seconds, seconds)
                prefs.checkIntervalSeconds = seconds
                if (prefs.isServiceRunning) {
                    scheduleRestart()
                }
            }
        })

        binding.startButton.setOnClickListener {
            val url = binding.urlInput.text.toString().trim()
            if (url.isBlank()) {
                Toast.makeText(this, R.string.error_empty_url, Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            prefs.serverUrl = url

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS)
                    != PackageManager.PERMISSION_GRANTED
                ) {
                    notificationPermissionLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
                    return@setOnClickListener
                }
            }
            doStartService()
        }

        binding.stopButton.setOnClickListener {
            doStopService()
        }
    }

    private fun doStartService() {
        val intent = Intent(this, MonitorService::class.java).apply {
            action = MonitorService.ACTION_START
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            ContextCompat.startForegroundService(this, intent)
        } else {
            startService(intent)
        }
        prefs.isServiceRunning = true
        updateUiState(running = true)
    }

    private fun doStopService() {
        val intent = Intent(this, MonitorService::class.java).apply {
            action = MonitorService.ACTION_STOP
        }
        startService(intent)
        prefs.isServiceRunning = false
        updateUiState(running = false)
    }

    /** Debounced restart, verhindert Race-Conditions beim Slider-Schieben. */
    private fun scheduleRestart() {
        restartJob?.cancel()
        restartJob = lifecycleScope.launch {
            delay(800)
            if (!prefs.isServiceRunning) return@launch
            doStopService()
            delay(200)
            doStartService()
        }
    }

    private fun updateUiState(running: Boolean) {
        binding.startButton.isEnabled = !running
        binding.stopButton.isEnabled = running
        binding.urlInput.isEnabled = !running

        if (running) {
            binding.statusText.text = getString(R.string.status_checking)
        } else {
            binding.statusText.text = getString(R.string.status_stopped)
        }
    }

    private fun startStatusPoller() {
        lifecycleScope.launch {
            while (isActive) {
                val running = prefs.isServiceRunning
                updateUiState(running)
                delay(2000)
            }
        }
    }
}
