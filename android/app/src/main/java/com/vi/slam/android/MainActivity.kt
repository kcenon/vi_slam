package com.vi.slam.android

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import com.vi.slam.android.ui.MainScreen

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            VISLAMTheme {
                MainScreen(
                    onSettingsClick = {
                        // TODO: Navigate to Settings screen
                    },
                    onCalibrationClick = {
                        // TODO: Navigate to Calibration screen
                    }
                )
            }
        }
    }
}

@Composable
fun VISLAMTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        content = content
    )
}
