package com.vi.slam.android

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import com.vi.slam.android.ui.CalibrationScreen
import com.vi.slam.android.ui.HistoryScreen
import com.vi.slam.android.ui.MainScreen
import com.vi.slam.android.ui.SettingsScreen

class MainActivity : ComponentActivity() {

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.entries.all { it.value }
        if (!allGranted) {
            // Handle permission denied case
            // For now, the app will continue but camera may not work
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Request camera permissions if not granted
        val requiredPermissions = arrayOf(
            Manifest.permission.CAMERA
        )

        val permissionsToRequest = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (permissionsToRequest.isNotEmpty()) {
            requestPermissionLauncher.launch(permissionsToRequest.toTypedArray())
        }

        setContent {
            VISLAMTheme {
                VISLAMApp(context = applicationContext)
            }
        }
    }
}

@Composable
fun VISLAMApp(context: Context) {
    var currentScreen by remember { mutableStateOf<Screen>(Screen.Main) }

    when (currentScreen) {
        Screen.Main -> {
            MainScreen(
                context = context,
                onSettingsClick = {
                    currentScreen = Screen.Settings
                },
                onCalibrationClick = {
                    currentScreen = Screen.Calibration
                },
                onHistoryClick = {
                    currentScreen = Screen.History
                }
            )
        }
        Screen.Settings -> {
            SettingsScreen(
                onBackClick = {
                    currentScreen = Screen.Main
                }
            )
        }
        Screen.History -> {
            HistoryScreen(
                onBackClick = {
                    currentScreen = Screen.Main
                }
            )
        }
        Screen.Calibration -> {
            CalibrationScreen(
                onBackClick = {
                    currentScreen = Screen.Main
                }
            )
        }
    }
}

sealed class Screen {
    data object Main : Screen()
    data object Settings : Screen()
    data object History : Screen()
    data object Calibration : Screen()
}

@Composable
fun VISLAMTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        content = content
    )
}
