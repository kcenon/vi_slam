package com.vi.slam.android

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import com.vi.slam.android.ui.HistoryScreen
import com.vi.slam.android.ui.MainScreen
import com.vi.slam.android.ui.SettingsScreen

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            VISLAMTheme {
                VISLAMApp()
            }
        }
    }
}

@Composable
fun VISLAMApp() {
    var currentScreen by remember { mutableStateOf<Screen>(Screen.Main) }

    when (currentScreen) {
        Screen.Main -> {
            MainScreen(
                onSettingsClick = {
                    currentScreen = Screen.Settings
                },
                onCalibrationClick = {
                    // TODO: Navigate to Calibration screen
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
    }
}

sealed class Screen {
    data object Main : Screen()
    data object Settings : Screen()
    data object History : Screen()
}

@Composable
fun VISLAMTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        content = content
    )
}
