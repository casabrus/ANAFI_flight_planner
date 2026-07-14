package com.fotogrammetria.anafiplanner

import android.Manifest
import android.annotation.SuppressLint
import android.content.Intent
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.net.Uri
import android.os.Bundle
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.webkit.JavascriptInterface
import android.webkit.WebChromeClient
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.ArrayAdapter
import android.widget.TextView
import android.widget.Toast
import android.view.WindowManager
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.view.isVisible
import androidx.core.view.ViewCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.updateLayoutParams
import androidx.core.view.updatePadding
import androidx.documentfile.provider.DocumentFile
import com.fotogrammetria.anafiplanner.databinding.ActivityMainBinding
import com.google.android.material.slider.Slider
import com.google.android.material.textfield.TextInputEditText
import com.fotogrammetria.anafiplanner.planner.CameraProfile
import com.fotogrammetria.anafiplanner.planner.CameraProfiles
import com.fotogrammetria.anafiplanner.planner.CircleOptimizationMode
import com.fotogrammetria.anafiplanner.planner.CircleRotationStrategy
import com.fotogrammetria.anafiplanner.planner.CircleShotMode
import com.fotogrammetria.anafiplanner.planner.CirclegrammetryMath
import com.fotogrammetria.anafiplanner.planner.CirclegrammetryParameters
import com.fotogrammetria.anafiplanner.planner.CirclegrammetryPlan
import com.fotogrammetria.anafiplanner.planner.CirclegrammetryPlanner
import com.fotogrammetria.anafiplanner.planner.DemoPlanFactory
import com.fotogrammetria.anafiplanner.planner.ElevationSummary
import com.fotogrammetria.anafiplanner.planner.FlightWaypoint
import com.fotogrammetria.anafiplanner.planner.FocusTarget
import com.fotogrammetria.anafiplanner.planner.FocusTargetMode
import com.fotogrammetria.anafiplanner.planner.FreeFlightPlanExporter
import com.fotogrammetria.anafiplanner.planner.GeoPoint
import com.fotogrammetria.anafiplanner.planner.GridSurveyParameters
import com.fotogrammetria.anafiplanner.planner.GridSurveyPlan
import com.fotogrammetria.anafiplanner.planner.GridSurveyPlanner
import com.fotogrammetria.anafiplanner.planner.MissionMode
import com.fotogrammetria.anafiplanner.planner.PlannerMath
import com.fotogrammetria.anafiplanner.planner.RouteSamplePoint
import com.fotogrammetria.anafiplanner.planner.SpeedMode
import com.fotogrammetria.anafiplanner.planner.TakeoffPoint
import com.fotogrammetria.anafiplanner.planner.TerrainSampling
import com.fotogrammetria.anafiplanner.planner.WaypointSegmentType
import com.fotogrammetria.anafiplanner.terrain.CopernicusDemElevationService
import com.fotogrammetria.anafiplanner.terrain.ElevationService
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.round
import kotlin.math.tan
import org.json.JSONArray
import org.json.JSONObject

class MainActivity : AppCompatActivity() {
    private enum class CircleQuality(
        val targetOffNadirDeg: Double,
        val targetSegmentLengthM: Double,
    ) {
        FAST(30.0, 16.0),
        NORMAL(38.0, 12.0),
        HIGH(46.0, 8.0),
    }

    private lateinit var binding: ActivityMainBinding
    private val cameraProfiles = CameraProfiles.all
    private val exporter = FreeFlightPlanExporter()
    private val terrainService: ElevationService by lazy {
        CopernicusDemElevationService(applicationContext)
    }
    private val plannerExecutor = Executors.newSingleThreadExecutor()
    private val terrainExecutor = Executors.newSingleThreadExecutor()
    private val mainHandler = Handler(Looper.getMainLooper())
    private val plannerRequestSequence = AtomicInteger()
    private val jsonPreviewRequestSequence = AtomicInteger()
    private val terrainRequestSequence = AtomicInteger()
    private var currentPolygonPoints: List<GeoPoint> = emptyList()
    private var currentPolygon: List<GeoPoint> = emptyList()
    private var currentTakeoffPoint: TakeoffPoint? = null
    private var latestJson: String = ""
    private var latestMission: PlannedMission? = null
    private var preparedExport: PreparedExport? = null
    private var terrainSnapshot = TerrainSnapshot.empty()
    private var terrainLoading = false
    private var terrainErrorMessage: String? = null
    private var mapReady = false
    private var takeoffPlacementMode = false
    private var focusMapOnNextSync = true
    private var currentBaseMap = "imagery"
    private var suppressAutoGenerate = false
    private var pendingExportAfterFolderSelection = false
    private var manualSpeedOverrideMps: Double? = null
    private var currentSpeedLimitMps = 0.0
    private var flightPlanTitleDraft: String? = null
    private var latestMissionRevision = 0
    private var pendingGenerateRunnable: Runnable? = null
    private var userLocationCentered = false
    private var isLocationTrackingActive = false
    private val locationListener = LocationListener { location ->
        runOnUiThread {
            showUserLocationOnMap(location, focus = !userLocationCentered)
            userLocationCentered = true
        }
    }
    private val locationPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { grants ->
        if (grants.values.any { it }) {
            startLocationTracking()
        } else {
            Toast.makeText(this, R.string.location_permission_required, Toast.LENGTH_SHORT).show()
        }
    }
    private val freeFlightFolderLauncher = registerForActivityResult(
        ActivityResultContracts.OpenDocumentTree(),
    ) { uri ->
        if (uri == null) {
            pendingExportAfterFolderSelection = false
            Toast.makeText(this, R.string.json_export_cancelled, Toast.LENGTH_SHORT).show()
            return@registerForActivityResult
        }

        runCatching {
            contentResolver.takePersistableUriPermission(
                uri,
                Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_WRITE_URI_PERMISSION,
            )
            persistFreeFlightTreeUri(uri)
        }.onSuccess {
            if (pendingExportAfterFolderSelection) {
                pendingExportAfterFolderSelection = false
                writePreparedExport()
            }
        }.onFailure { error ->
            pendingExportAfterFolderSelection = false
            Toast.makeText(
                this,
                getString(R.string.json_export_failed, error.message ?: error.javaClass.simpleName),
                Toast.LENGTH_LONG,
            ).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdgeLandscape()
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        setupInputs()
        setupMap()
        setupButtons()
        applyWindowInsets()
        renderProjectState()
        generatePlan()
    }

    override fun onResume() {
        super.onResume()
        startLocationTracking()
    }

    override fun onPause() {
        stopLocationTracking()
        super.onPause()
    }

    private fun setupInputs() {
        suppressAutoGenerate = true
        binding.circleOptimizationSpinner.adapter = spinnerAdapter(
            listOf(
                getString(R.string.circle_optimization_fixed),
                getString(R.string.circle_optimization_fastest),
                getString(R.string.circle_optimization_balanced),
            ),
        )
        binding.circleFocusTargetSpinner.adapter = spinnerAdapter(
            listOf(
                getString(R.string.circle_focus_ground),
                getString(R.string.circle_focus_canopy),
                getString(R.string.circle_focus_object),
                getString(R.string.circle_focus_custom),
            ),
        )

        binding.menuTabs.addTab(binding.menuTabs.newTab().setText(R.string.tab_setup))
        binding.menuTabs.addTab(binding.menuTabs.newTab().setText(R.string.tab_geometry))
        binding.menuTabs.addTab(binding.menuTabs.newTab().setText(R.string.tab_review))
        binding.menuTabs.addOnTabSelectedListener(
            object : com.google.android.material.tabs.TabLayout.OnTabSelectedListener {
                override fun onTabSelected(tab: com.google.android.material.tabs.TabLayout.Tab) {
                    setMenuTab(tab.position)
                }

                override fun onTabUnselected(tab: com.google.android.material.tabs.TabLayout.Tab) = Unit

                override fun onTabReselected(tab: com.google.android.material.tabs.TabLayout.Tab) = Unit
            },
        )
        binding.missionModeGroup.check(R.id.missionGridButton)
        binding.cameraProfileGroup.check(R.id.cameraRawButton)
        setupSliderInputs()

        binding.missionModeGroup.addOnButtonCheckedListener { _, _, isChecked ->
            if (!isChecked) {
                return@addOnButtonCheckedListener
            }
            updateMissionInputVisibility()
            maybeGeneratePlan()
        }
        binding.cameraProfileGroup.addOnButtonCheckedListener { _, _, isChecked ->
            if (!isChecked) {
                return@addOnButtonCheckedListener
            }
            syncAltitudeFromGsd()
            syncCircleSuggestedControls()
            maybeGeneratePlan()
        }
        binding.circleOptimizationSpinner.onItemSelectedListener = SimpleItemSelectedListener {
            syncCircleSuggestedControls()
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleFocusTargetSpinner.onItemSelectedListener = SimpleItemSelectedListener {
            syncCircleSuggestedControls()
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleLockRadiusSwitch.setOnCheckedChangeListener { _, isChecked ->
            syncCircleSuggestedControls(forceRadius = isChecked)
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleLockAngleSwitch.setOnCheckedChangeListener { _, isChecked ->
            syncCircleSuggestedControls(forceTilt = isChecked)
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleManualOverlapSwitch.setOnCheckedChangeListener { _, _ ->
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleManualPointsSwitch.setOnCheckedChangeListener { _, isChecked ->
            syncCircleSuggestedControls(forcePoints = isChecked)
            updateCircleUiState()
            maybeGeneratePlan()
        }
        binding.circleOptimizationSpinner.setSelection(1, false)
        binding.circleFocusTargetSpinner.setSelection(0, false)
        setMenuTab(0)
        updateMissionInputVisibility()
        updateCircleUiState()
        updateSpeedInputState(0.0, null)
        suppressAutoGenerate = false
    }

    private fun setupSliderInputs() {
        bindSlider(
            slider = binding.altitudeSlider,
            labelView = binding.altitudeValueText,
            input = binding.altitudeInput,
            initialValue = 1.5,
            displayFormatter = { value -> formatGsdDisplay(value.toDouble(), selectedCameraProfile()) },
            inputFormatter = { value -> formatDerivedAltitudeFromGsd(value.toDouble(), selectedCameraProfile()) },
        )
        binding.altitudeSlider.addOnChangeListener { _, _, _ -> syncCircleSuggestedControls() }
        bindSlider(
            slider = binding.frontOverlapSlider,
            labelView = binding.frontOverlapValueText,
            input = binding.frontOverlapInput,
            initialValue = 80.0,
            displayFormatter = { value ->
                getString(R.string.front_overlap_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        bindSlider(
            slider = binding.sideOverlapSlider,
            labelView = binding.sideOverlapValueText,
            input = binding.sideOverlapInput,
            initialValue = 70.0,
            displayFormatter = { value ->
                getString(R.string.side_overlap_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        binding.gridAngleInput.setText("20")
        binding.gridAngleValueText.text = getString(R.string.grid_angle_display, 20)
        setupSpeedSlider()
        bindSlider(
            slider = binding.circleRadiusSlider,
            labelView = binding.circleRadiusValueText,
            input = binding.circleRadiusInput,
            initialValue = 30.0,
            displayFormatter = { value ->
                getString(R.string.circle_radius_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        bindSlider(
            slider = binding.circleOverlapSlider,
            labelView = binding.circleOverlapValueText,
            input = binding.circleOverlapInput,
            initialValue = 50.0,
            displayFormatter = { value ->
                getString(R.string.circle_overlap_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        bindSlider(
            slider = binding.circlePointsSlider,
            labelView = binding.circlePointsValueText,
            input = binding.circlePointsInput,
            initialValue = 24.0,
            displayFormatter = { value ->
                getString(R.string.circle_points_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        bindSlider(
            slider = binding.circleFocusHeightSlider,
            labelView = binding.circleFocusHeightValueText,
            input = binding.circleFocusHeightInput,
            initialValue = 3.0,
            displayFormatter = { value ->
                getString(R.string.circle_focus_height_display, formatSliderDecimal(value, 1))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 1) },
        )
        binding.circleFocusHeightSlider.addOnChangeListener { _, _, _ -> syncCircleSuggestedControls() }
        bindSlider(
            slider = binding.circleTiltSlider,
            labelView = binding.circleTiltValueText,
            input = binding.circleTiltInput,
            initialValue = 55.0,
            displayFormatter = { value ->
                getString(R.string.circle_tilt_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
        bindSlider(
            slider = binding.circleMaxExtensionSlider,
            labelView = binding.circleMaxExtensionValueText,
            input = binding.circleMaxExtensionInput,
            initialValue = 30.0,
            displayFormatter = { value ->
                getString(R.string.circle_max_extension_display, formatSliderDecimal(value, 0))
            },
            inputFormatter = { value -> formatSliderDecimal(value, 0) },
        )
    }

    private fun spinnerAdapter(items: List<String>): ArrayAdapter<String> {
        return ArrayAdapter(
            this,
            R.layout.item_spinner_selected,
            items,
        ).apply {
            setDropDownViewResource(R.layout.item_spinner_dropdown)
        }
    }

    private fun setupSpeedSlider() {
        val initialValue = 3.0f
        binding.speedValueText.text = getString(
            R.string.speed_display,
            formatSliderDecimal(initialValue, 1),
        )
        binding.speedInput.setText(formatSliderDecimal(initialValue, 1))
        binding.speedSlider.value = initialValue
        binding.speedSlider.addOnChangeListener { _, value, fromUser ->
            binding.speedValueText.text = getString(
                R.string.speed_display,
                formatSliderDecimal(value, 1),
            )
            binding.speedInput.setText(formatSliderDecimal(value, 1))
            if (fromUser) {
                manualSpeedOverrideMps = value.toDouble()
            }
        }
        binding.speedSlider.addOnSliderTouchListener(
            object : Slider.OnSliderTouchListener {
                override fun onStartTrackingTouch(slider: Slider) = Unit

                override fun onStopTrackingTouch(slider: Slider) {
                    maybeGeneratePlan()
                }
            },
        )
    }

    private fun bindSlider(
        slider: Slider,
        labelView: TextView,
        input: TextInputEditText,
        initialValue: Double,
        displayFormatter: (Float) -> String,
        inputFormatter: (Float) -> String,
    ) {
        val initialFloat = initialValue.toFloat()
        labelView.text = displayFormatter(initialFloat)
        input.setText(inputFormatter(initialFloat))
        slider.value = initialFloat
        slider.addOnChangeListener { _, value, _ ->
            labelView.text = displayFormatter(value)
            input.setText(inputFormatter(value))
        }
        slider.addOnSliderTouchListener(
            object : Slider.OnSliderTouchListener {
                override fun onStartTrackingTouch(slider: Slider) = Unit

                override fun onStopTrackingTouch(slider: Slider) {
                    maybeGeneratePlan()
                }
            },
        )
    }

    private fun maybeGeneratePlan() {
        if (!suppressAutoGenerate) {
            schedulePlanGeneration(immediate = false)
        }
    }

    private fun schedulePlanGeneration(immediate: Boolean) {
        pendingGenerateRunnable?.let(mainHandler::removeCallbacks)
        val runnable = Runnable {
            pendingGenerateRunnable = null
            dispatchPlanGeneration()
        }
        if (immediate) {
            runnable.run()
        } else {
            pendingGenerateRunnable = runnable
            mainHandler.postDelayed(runnable, PLAN_REGEN_DEBOUNCE_MS)
        }
    }

    private fun dispatchPlanGeneration() {
        val requestId = plannerRequestSequence.incrementAndGet()
        terrainRequestSequence.incrementAndGet()
        terrainLoading = false
        terrainErrorMessage = null
        terrainSnapshot = TerrainSnapshot.empty()
        renderProjectState()

        if (currentPolygon.size < 3) {
            latestMission = null
            clearPlanningOutputs()
            updateSpeedInputState(0.0, null)
            syncMapProject()
            return
        }

        val request = runCatching { capturePlanningRequest() }
            .getOrElse { error ->
                handlePlanGenerationFailure(error)
                return
            }

        plannerExecutor.execute {
            val result = runCatching { createMission(request) }
            runOnUiThread {
                if (requestId != plannerRequestSequence.get()) {
                    return@runOnUiThread
                }

                result.onSuccess { mission ->
                    latestMission = mission
                    latestMissionRevision += 1
                    preparedExport = null
                    invalidateJsonPreview(showPendingState = true)
                    binding.exportPathText.text = getString(R.string.export_path_placeholder)
                    updateSpeedInputState(mission.maxSupportedSpeedMps, mission.selectedSpeedMps)
                    renderProjectState()
                    syncMapProject()
                    if (binding.reviewTabContent.isVisible) {
                        ensureLatestJsonPreview()
                    }
                }.onFailure(::handlePlanGenerationFailure)
            }
        }
    }

    private fun handlePlanGenerationFailure(error: Throwable) {
        latestMission = null
        clearPlanningOutputs()
        updateSpeedInputState(0.0, null)
        Toast.makeText(
            this,
            getString(R.string.plan_generation_failed, error.message ?: error.javaClass.simpleName),
            Toast.LENGTH_LONG,
        ).show()
        renderProjectState()
        syncMapProject()
    }

    private fun syncCircleSuggestedControls(
        forceRadius: Boolean = false,
        forceTilt: Boolean = false,
        forcePoints: Boolean = false,
    ) {
        val altitudeM = parseOptionalDouble(binding.altitudeInput.text?.toString(), "Altitude") ?: return
        val quality = selectedCircleQuality()
        val focusTarget = runCatching { selectedCircleFocusTarget() }.getOrNull() ?: return
        val suggestedRadiusM = runCatching {
            CirclegrammetryMath.radiusFromOffNadirAngle(
                altitudeM,
                focusTarget.heightAboveTerrainM,
                quality.targetOffNadirDeg,
            )
        }.getOrNull() ?: return
        val suggestedTiltDeg = (90.0 - quality.targetOffNadirDeg).coerceIn(20.0, 85.0)
        val suggestedPoints = autoCirclePointsPerCircle(suggestedRadiusM, quality.targetSegmentLengthM)

        if (forceRadius || !binding.circleLockRadiusSwitch.isChecked) {
            syncSliderPresentation(
                slider = binding.circleRadiusSlider,
                labelView = binding.circleRadiusValueText,
                input = binding.circleRadiusInput,
                value = suggestedRadiusM,
                decimals = 0,
                title = getString(R.string.circle_radius_title),
                suffix = "m",
            )
        }
        if (forceTilt || !binding.circleLockAngleSwitch.isChecked) {
            syncSliderPresentation(
                slider = binding.circleTiltSlider,
                labelView = binding.circleTiltValueText,
                input = binding.circleTiltInput,
                value = suggestedTiltDeg,
                decimals = 0,
                title = getString(R.string.circle_tilt_title),
                suffix = "deg",
            )
        }
        if (forcePoints || !binding.circleManualPointsSwitch.isChecked) {
            syncSliderPresentation(
                slider = binding.circlePointsSlider,
                labelView = binding.circlePointsValueText,
                input = binding.circlePointsInput,
                value = suggestedPoints.toDouble(),
                decimals = 0,
                title = getString(R.string.circle_points_title),
                suffix = null,
            )
        }
    }

    private fun syncSliderPresentation(
        slider: Slider,
        labelView: TextView,
        input: TextInputEditText,
        value: Double,
        decimals: Int,
        title: String,
        suffix: String?,
    ) {
        val clampedValue = snapSliderValue(slider, value).toFloat()
        slider.value = clampedValue
        input.setText(formatSliderDecimal(clampedValue, decimals))
        val body = formatSliderDecimal(clampedValue, decimals)
        labelView.text = if (suffix != null) {
            "$title: $body $suffix"
        } else {
            "$title: $body"
        }
    }

    private fun snapSliderValue(
        slider: Slider,
        value: Double,
    ): Double {
        val valueFrom = slider.valueFrom.toDouble()
        val valueTo = slider.valueTo.toDouble()
        val clamped = value.coerceIn(valueFrom, valueTo)
        val step = slider.stepSize.toDouble()
        if (step <= 0.0) {
            return clamped
        }
        val steps = kotlin.math.round((clamped - valueFrom) / step)
        return (valueFrom + steps * step).coerceIn(valueFrom, valueTo)
    }

    private fun formatSliderDecimal(value: Float, decimals: Int): String {
        return String.format(Locale.US, "%.${decimals}f", value)
    }

    private fun formatGsdDisplay(
        gsdCmPerPixel: Double,
        cameraProfile: CameraProfile,
    ): String {
        val altitudeM = altitudeFromGsd(gsdCmPerPixel, cameraProfile)
        return getString(
            R.string.gsd_display,
            formatDecimal(gsdCmPerPixel),
            altitudeM.toInt(),
        )
    }

    private fun formatDerivedAltitudeFromGsd(
        gsdCmPerPixel: Double,
        cameraProfile: CameraProfile,
    ): String {
        return altitudeFromGsd(gsdCmPerPixel, cameraProfile).toInt().toString()
    }

    private fun altitudeFromGsd(
        gsdCmPerPixel: Double,
        cameraProfile: CameraProfile,
    ): Int {
        val footprintWidthM = (gsdCmPerPixel / 100.0) * cameraProfile.imageWidthPx
        val halfFovRad = Math.toRadians(cameraProfile.horizontalFovDeg / 2.0)
        return kotlin.math.round(footprintWidthM / (2.0 * tan(halfFovRad))).toInt().coerceAtLeast(1)
    }

    private fun syncAltitudeFromGsd() {
        val gsd = binding.altitudeSlider.value.toDouble()
        val altitudeM = altitudeFromGsd(gsd, selectedCameraProfile())
        binding.altitudeInput.setText(altitudeM.toString())
        binding.altitudeValueText.text = formatGsdDisplay(gsd, selectedCameraProfile())
    }

    private fun autoCirclePointsPerCircle(
        radiusM: Double,
        targetSegmentLengthM: Double,
    ): Int {
        val circumferenceM = 2.0 * PI * radiusM
        val rawPoints = kotlin.math.ceil(circumferenceM / targetSegmentLengthM).toInt()
        val snappedPoints = ((rawPoints + 3) / 4) * 4
        return snappedPoints.coerceIn(12, 96)
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupMap() {
        WebView.setWebContentsDebuggingEnabled(true)
        binding.mapWebView.settings.javaScriptEnabled = true
        binding.mapWebView.settings.domStorageEnabled = true
        binding.mapWebView.settings.allowFileAccess = true
        binding.mapWebView.overScrollMode = WebView.OVER_SCROLL_NEVER
        binding.mapWebView.webViewClient = WebViewClient()
        binding.mapWebView.webChromeClient = WebChromeClient()
        binding.mapWebView.addJavascriptInterface(MapBridge(), "AndroidBridge")
        binding.mapWebView.loadUrl("file:///android_asset/map/index.html")
    }

    private fun setupButtons() {
        binding.savePlanButton.setOnClickListener { prepareSaveDialog() }
        binding.resetPolygonMapButton.setOnClickListener { confirmPolygonReset() }
        binding.menuButton.setOnClickListener { setControlsOverlayVisible(true) }
        binding.closeMenuButton.setOnClickListener { setControlsOverlayVisible(false) }
        binding.saveDialogOverlay.setOnClickListener { setSaveDialogVisible(false) }
        binding.saveDialogCard.setOnClickListener { }
        binding.saveDialogCloseButton.setOnClickListener { setSaveDialogVisible(false) }
        binding.saveDialogExportButton.setOnClickListener {
            flightPlanTitleDraft = binding.savePlanNameInput.text?.toString()?.trim().orEmpty()
            requestExportFromSaveDialog()
        }
        binding.mapSourceButton.setOnClickListener {
            currentBaseMap = if (currentBaseMap == "imagery") "osm" else "imagery"
            updateMapSourceButton()
            applyMapBaseLayer()
        }
        binding.resetExportFolderButton.setOnClickListener {
            clearPersistedFreeFlightTreeUri()
            pendingExportAfterFolderSelection = false
            binding.exportPathText.text = getString(R.string.export_path_placeholder)
            Toast.makeText(this, R.string.json_export_folder_reset, Toast.LENGTH_SHORT).show()
        }
        binding.setTakeoffButton.setOnClickListener {
            takeoffPlacementMode = !takeoffPlacementMode
            updateTakeoffModeUi()
            setMapMode(if (takeoffPlacementMode) "takeoff" else "polygon")
        }
        updateMapSourceButton()
        updateTakeoffModeUi()
        syncAltitudeFromGsd()
    }

    @Suppress("DEPRECATION")
    private fun enableEdgeToEdgeLandscape() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        window.statusBarColor = android.graphics.Color.TRANSPARENT
        window.navigationBarColor = android.graphics.Color.TRANSPARENT
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            window.attributes = window.attributes.apply {
                layoutInDisplayCutoutMode =
                    WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
            }
        }
        applyImmersiveMode()
    }

    private fun applyWindowInsets() {
        ViewCompat.setOnApplyWindowInsetsListener(binding.rootContainer) { _, insets ->
            val systemBars = insets.getInsets(
                WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout(),
            )
            binding.menuButton.updateLayoutParams<androidx.coordinatorlayout.widget.CoordinatorLayout.LayoutParams> {
                topMargin = dp(18) + systemBars.top
                rightMargin = dp(16) + systemBars.right
            }
            binding.mapControlsGroup.updateLayoutParams<androidx.coordinatorlayout.widget.CoordinatorLayout.LayoutParams> {
                rightMargin = dp(16) + systemBars.right
                bottomMargin = dp(20) + systemBars.bottom
            }
            binding.mapStatusLine.updateLayoutParams<androidx.coordinatorlayout.widget.CoordinatorLayout.LayoutParams> {
                bottomMargin = dp(34) + systemBars.bottom
            }
            binding.controlsOverlay.updatePadding(
                left = dp(16) + systemBars.left,
                top = dp(16) + systemBars.top,
                right = dp(16) + systemBars.right,
                bottom = dp(16) + systemBars.bottom,
            )
            binding.saveDialogOverlay.updatePadding(
                left = dp(16) + systemBars.left,
                top = dp(16) + systemBars.top,
                right = dp(16) + systemBars.right,
                bottom = dp(16) + systemBars.bottom,
            )
            insets
        }
    }

    private fun dp(value: Int): Int {
        return (value * resources.displayMetrics.density).toInt()
    }

    private fun applyImmersiveMode() {
        val controller = WindowInsetsControllerCompat(window, window.decorView)
        controller.systemBarsBehavior =
            WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        controller.hide(WindowInsetsCompat.Type.systemBars())
    }

    private fun setControlsOverlayVisible(visible: Boolean) {
        binding.controlsOverlay.isVisible = visible
        binding.menuButton.isVisible = !visible
        binding.mapControlsGroup.isVisible = !visible
    }

    private fun setSaveDialogVisible(visible: Boolean) {
        binding.saveDialogOverlay.isVisible = visible
    }

    private fun updateMapSourceButton() {
        binding.mapSourceButton.setImageResource(R.drawable.ic_map_outline)
        binding.mapSourceButton.contentDescription = if (currentBaseMap == "imagery") {
            getString(R.string.map_source_sat)
        } else {
            getString(R.string.map_source_osm)
        }
    }

    private fun applyMapBaseLayer() {
        if (!mapReady) {
            return
        }
        evaluateJavascriptSafely("window.plannerMap.setBaseMap('$currentBaseMap');")
    }

    private fun prepareSaveDialog() {
        val mission = latestMission
        if (mission == null) {
            Toast.makeText(this, R.string.json_export_missing_plan, Toast.LENGTH_SHORT).show()
            return
        }
        val takeoff = currentTakeoffPoint
        if (takeoff == null) {
            Toast.makeText(this, R.string.json_export_home_required, Toast.LENGTH_LONG).show()
            return
        }

        val requestId = terrainRequestSequence.incrementAndGet()
        val polygon = currentPolygon.toList()
        val exportPlanTitle = selectedPlanTitle(selectedCameraProfile())
        val routeSamplePoints = buildRouteSamplePoints(mission, takeoff)
        terrainLoading = true
        terrainErrorMessage = null
        terrainSnapshot = TerrainSnapshot.empty()
        preparedExport = null
        binding.exportPathText.text = getString(R.string.json_export_preparing)
        renderProjectState()
        syncMapProject()

        terrainExecutor.execute {
            var snapshot = TerrainSnapshot.empty()
            val result = runCatching {
                snapshot = buildTerrainSnapshot(
                    polygon = polygon,
                    takeoff = takeoff,
                    waypoints = mission.waypoints,
                    routeSamplePoints = routeSamplePoints,
                )
                val adjustedMission = applyTerrainAdjustedMission(mission, takeoff, snapshot).copy(
                    title = exportPlanTitle,
                )
                val adjustedExportWaypoints = buildTerrainAdjustedExportWaypoints(
                    mission = adjustedMission,
                    takeoff = takeoff,
                    snapshot = snapshot,
                    routeSamplePoints = routeSamplePoints,
                )
                val adjustedJson = exporter.exportPrepared(
                    title = adjustedMission.title,
                    polygon = adjustedMission.polygon,
                    waypoints = adjustedExportWaypoints,
                    takeoffPoint = takeoff,
                )
                PreparedExport(
                    snapshot = snapshot,
                    mission = adjustedMission,
                    exportWaypoints = adjustedExportWaypoints,
                    jsonPayload = adjustedJson,
                )
            }

            runOnUiThread {
                if (requestId != terrainRequestSequence.get()) {
                    return@runOnUiThread
                }

                terrainLoading = false
                result.onSuccess { readyExport ->
                    preparedExport = readyExport
                    terrainSnapshot = readyExport.snapshot
                    terrainErrorMessage = null
                    latestMissionRevision += 1
                    jsonPreviewRequestSequence.incrementAndGet()
                    latestMission = readyExport.mission
                    latestJson = readyExport.jsonPayload
                    setJsonPreviewText(readyExport.jsonPayload)
                    binding.exportPathText.text = getString(R.string.export_path_placeholder)
                    renderProjectState()
                    syncMapProject()
                    openSaveDialog(readyExport.mission)
                }.onFailure { error ->
                    preparedExport = null
                    terrainSnapshot = snapshot
                    terrainErrorMessage = error.message ?: error.javaClass.simpleName
                    binding.exportPathText.text = getString(R.string.export_path_placeholder)
                    Toast.makeText(
                        this,
                        getString(R.string.json_export_failed, error.message ?: error.javaClass.simpleName),
                        Toast.LENGTH_LONG,
                    ).show()
                    renderProjectState()
                    syncMapProject()
                }
            }
        }
    }

    private fun openSaveDialog(mission: PlannedMission) {
        binding.savePlanSummaryText.text = buildSaveDialogSummary(mission)
        binding.savePlanNameInput.setText(selectedPlanTitle(selectedCameraProfile()))
        binding.savePlanNameInput.setSelection(binding.savePlanNameInput.text?.length ?: 0)
        binding.saveDialogExportButton.isEnabled = true
        binding.saveDialogExportButton.alpha = 1f
        setSaveDialogVisible(true)
    }

    private fun setMenuTab(position: Int) {
        binding.setupTabContent.isVisible = position == 0
        binding.geometryTabContent.isVisible = position == 1
        binding.reviewTabContent.isVisible = position == 2
        if (position == 2) {
            ensureLatestJsonPreview()
        }
    }

    private fun updateMissionInputVisibility() {
        val missionMode = selectedMissionMode()
        binding.gridInputsGroup.isVisible = missionMode != MissionMode.CIRCLEGRAMMETRY
        binding.circleInputsGroup.isVisible = missionMode != MissionMode.GRID_NADIR
        updateCircleUiState()
    }

    private fun updateCircleUiState() {
        syncCircleSuggestedControls()
        binding.circleFocusHeightBlock.isVisible =
            binding.circleFocusTargetSpinner.selectedItemPosition == 3
        binding.circleRadiusBlock.isVisible = binding.circleLockRadiusSwitch.isChecked
        binding.circleTiltBlock.isVisible = binding.circleLockAngleSwitch.isChecked
        binding.circleOverlapBlock.isVisible = binding.circleManualOverlapSwitch.isChecked
        binding.circlePointsBlock.isVisible = binding.circleManualPointsSwitch.isChecked
    }

    private fun confirmPolygonReset() {
        if (currentPolygonPoints.isEmpty()) {
            return
        }
        showNonDimmedDialog(
            AlertDialog.Builder(this)
            .setTitle(R.string.reset_polygon_title)
            .setMessage(R.string.reset_polygon_message)
            .setNegativeButton(R.string.close_button, null)
            .setPositiveButton(R.string.reset_polygon_confirm) { _, _ ->
                currentPolygonPoints = emptyList()
                currentPolygon = emptyList()
                focusMapOnNextSync = false
                generatePlan()
            },
        )
    }

    private fun requestExportFromSaveDialog() {
        if (currentTakeoffPoint == null) {
            Toast.makeText(this, R.string.json_export_home_required, Toast.LENGTH_LONG).show()
            return
        }
        val treeUri = getPersistedFreeFlightTreeUri()
        val planTitle = selectedPlanTitle(selectedCameraProfile())
        if (treeUri != null) {
            val existingPath = findExistingExportPath(treeUri, planTitle)
            if (existingPath != null) {
                showOverwriteExportDialog(existingPath)
                return
            }
        }
        val readyExport = preparedExport
        if (readyExport == null) {
            Toast.makeText(this, R.string.json_export_preparing, Toast.LENGTH_SHORT).show()
            return
        }
        setSaveDialogVisible(false)
        writePreparedExport(skipOverwriteCheck = true)
    }

    private fun updateSpeedInputState(
        maxSupportedSpeedMps: Double,
        selectedSpeedMps: Double?,
    ) {
        currentSpeedLimitMps = maxSupportedSpeedMps.coerceAtLeast(0.0)
        val showSlider = currentSpeedLimitMps > 5.0
        val effectiveSelectedSpeed = selectedSpeedMps ?: currentSpeedLimitMps
        val effectiveManual = effectiveManualSpeedOverride(currentSpeedLimitMps)
        if (!showSlider) {
            manualSpeedOverrideMps = null
        }

        binding.speedSliderBlock.isVisible = showSlider
        binding.speedSlider.isEnabled = showSlider

        val sliderMax = maxOf(1.0, roundUpToHalfStep(currentSpeedLimitMps))
        val sliderValue = when {
            showSlider && effectiveManual != null -> roundToHalfStep(effectiveManual)
            effectiveSelectedSpeed > 0.0 -> roundToHalfStep(effectiveSelectedSpeed.coerceAtMost(sliderMax))
            else -> 1.0
        }

        binding.speedSlider.valueFrom = 1f
        binding.speedSlider.valueTo = sliderMax.toFloat()
        binding.speedSlider.value = sliderValue.toFloat()
        binding.speedInput.setText(formatSliderDecimal(sliderValue.toFloat(), 1))
        binding.speedValueText.text = getString(
            R.string.speed_display,
            formatDecimal(sliderValue),
        )
        binding.speedAutoText.text = if (currentSpeedLimitMps <= 0.0) {
            getString(R.string.speed_auto_placeholder)
        } else if (showSlider) {
            getString(
                R.string.speed_auto_max,
                formatDecimal(currentSpeedLimitMps),
            )
        } else {
            getString(
                R.string.speed_auto_locked,
                formatDecimal(currentSpeedLimitMps),
            )
        }
    }

    private fun updateTakeoffModeUi() {
        if (takeoffPlacementMode) {
            binding.setTakeoffButton.setImageResource(R.drawable.ic_close_planner)
            binding.setTakeoffButton.contentDescription = getString(R.string.map_set_takeoff_cancel)
        } else {
            binding.setTakeoffButton.setImageResource(R.drawable.ic_takeoff_drone)
            binding.setTakeoffButton.contentDescription = getString(R.string.map_set_takeoff)
        }
    }

    private fun renderProjectState() {
        binding.mapStatusLine.text = buildMapStatusLine()
    }

    private fun generatePlan() {
        schedulePlanGeneration(immediate = true)
    }

    private fun capturePlanningRequest(): PlanningRequest {
        return PlanningRequest(
            polygon = currentPolygon.toList(),
            takeoffPoint = currentTakeoffPoint,
            missionMode = selectedMissionMode(),
            cameraProfile = selectedCameraProfile(),
            planTitleDraft = flightPlanTitleDraft?.trim().orEmpty(),
            altitudeM = parseDouble(binding.altitudeInput.text?.toString(), "Altitude"),
            frontOverlap = parsePercent(binding.frontOverlapInput.text?.toString(), "Front overlap"),
            sideOverlap = parsePercent(binding.sideOverlapInput.text?.toString(), "Side overlap"),
            gridAngleDeg = parseDouble(binding.gridAngleInput.text?.toString(), "Grid angle"),
            circleQuality = selectedCircleQuality(),
            circleFocusTarget = selectedCircleFocusTarget(),
            circleLockRadius = binding.circleLockRadiusSwitch.isChecked,
            circleLockAngle = binding.circleLockAngleSwitch.isChecked,
            circleManualOverlap = binding.circleManualOverlapSwitch.isChecked,
            circleManualPoints = binding.circleManualPointsSwitch.isChecked,
            circleMaxExtensionM = parseOptionalDouble(
                binding.circleMaxExtensionInput.text?.toString(),
                "Max extension",
            ),
            circleRequestedOverlap = parsePercent(binding.circleOverlapInput.text?.toString(), "Circle overlap"),
            circleLockedRadiusM = parseDouble(binding.circleRadiusInput.text?.toString(), "Circle radius"),
            circleLockedTiltDeg = parseDouble(binding.circleTiltInput.text?.toString(), "Circle tilt"),
            circleManualPointsValue = parseInt(binding.circlePointsInput.text?.toString(), "Points per circle"),
            manualSpeedOverrideMps = manualSpeedOverrideMps,
        )
    }

    private fun createMission(request: PlanningRequest): PlannedMission {
        return when (request.missionMode) {
            MissionMode.GRID_NADIR -> createGridMission(request)
            MissionMode.CIRCLEGRAMMETRY -> createCircleMission(request)
            MissionMode.HYBRID -> createHybridMission(request)
        }
    }

    private fun createGridMission(request: PlanningRequest): PlannedMission {
        val maxSupportedSpeedMps = computeGridMaxSupportedSpeed(
            cameraProfile = request.cameraProfile,
            altitudeM = request.altitudeM,
            frontOverlap = request.frontOverlap,
        )
        val speedMode = selectedSpeedMode(request.manualSpeedOverrideMps, maxSupportedSpeedMps)
        val requestedSpeedMps = selectedRequestedSpeed(request.manualSpeedOverrideMps, maxSupportedSpeedMps)
        val plan = GridSurveyPlanner().generatePlan(
            parameters = GridSurveyParameters(
                polygon = request.polygon,
                altitudeM = request.altitudeM,
                frontOverlap = request.frontOverlap,
                sideOverlap = request.sideOverlap,
                cameraProfile = request.cameraProfile,
                speedMode = speedMode,
                requestedSpeedMps = requestedSpeedMps,
                gridAngleDeg = request.gridAngleDeg,
                gimbalSettings = DemoPlanFactory.defaultGimbalSettings(),
            ),
            title = resolvePlanTitle(request.missionMode, request.cameraProfile, request.planTitleDraft),
            takeoffPoint = request.takeoffPoint,
        )

        return PlannedMission(
            title = plan.title,
            polygon = plan.polygon,
            waypoints = plan.waypoints,
            summary = buildGridSummary(plan, request.takeoffPoint),
            warning = plan.metrics.timing.warning,
            estimatedDistanceM = plan.estimatedDistanceM,
            estimatedDurationSec = plan.estimatedDurationSec,
            estimatedPhotoCount = plan.estimatedPhotoCount,
            baseAltitudeM = plan.parameters.altitudeM,
            maxSupportedSpeedMps = plan.metrics.timing.maxSupportedSpeedMps,
            selectedSpeedMps = plan.metrics.timing.selectedSpeedMps,
            selectedCapturePeriodSec = plan.metrics.timing.selectedPeriodSec,
        )
    }

    private fun createCircleMission(request: PlanningRequest): PlannedMission {
        val focusHeightM = request.circleFocusTarget.heightAboveTerrainM
        val derivedOffNadirDeg = when {
            request.circleLockRadius -> CirclegrammetryMath.offNadirAngleFromRadius(
                request.altitudeM,
                focusHeightM,
                request.circleLockedRadiusM,
            )

            request.circleLockAngle -> (90.0 - request.circleLockedTiltDeg).coerceIn(5.0, 75.0)
            else -> request.circleQuality.targetOffNadirDeg
        }
        val radiusM = if (request.circleLockRadius) {
            request.circleLockedRadiusM
        } else {
            CirclegrammetryMath.radiusFromOffNadirAngle(request.altitudeM, focusHeightM, derivedOffNadirDeg)
        }
        val pointsPerCircle = if (request.circleManualPoints) {
            request.circleManualPointsValue
        } else {
            autoCirclePointsPerCircle(radiusM, request.circleQuality.targetSegmentLengthM)
        }
        val cameraTiltDeg = if (request.circleLockAngle) {
            request.circleLockedTiltDeg
        } else {
            (90.0 - derivedOffNadirDeg).coerceIn(20.0, 85.0)
        }
        val maxSupportedSpeedMps = computeCircleMaxSupportedSpeed(
            radiusM = radiusM,
            pointsPerCircle = pointsPerCircle,
            cameraProfile = request.cameraProfile,
        )
        val speedMode = selectedSpeedMode(request.manualSpeedOverrideMps, maxSupportedSpeedMps)
        val requestedSpeedMps = selectedRequestedSpeed(request.manualSpeedOverrideMps, maxSupportedSpeedMps)
        val plan = CirclegrammetryPlanner().plan(
            parameters = CirclegrammetryParameters(
                polygon = request.polygon,
                altitudeM = request.altitudeM,
                radiusM = radiusM,
                radiusLocked = request.circleLockRadius,
                requestedOverlap = request.circleRequestedOverlap,
                optimizedOverlap = null,
                maxExtensionOutsideAreaM = request.circleMaxExtensionM,
                speedMode = speedMode,
                requestedSpeedMps = requestedSpeedMps,
                cameraTiltDeg = cameraTiltDeg,
                cameraAngleLocked = request.circleLockAngle,
                cameraTiltSpeedDegSec = 30.0,
                photoMode = request.cameraProfile.photoMode,
                cameraProfile = request.cameraProfile,
                pointsPerCircle = pointsPerCircle,
                pointsLocked = request.circleManualPoints,
                targetSegmentLengthM = if (request.circleManualPoints) null else request.circleQuality.targetSegmentLengthM,
                rotationStrategy = CircleRotationStrategy.ALTERNATING_ROWS,
                shotMode = CircleShotMode.AUTO,
                optimizationMode = if (request.circleManualOverlap) {
                    CircleOptimizationMode.USER_FIXED_OVERLAP
                } else {
                    CircleOptimizationMode.BALANCED_THRESHOLD
                },
                focusTarget = request.circleFocusTarget,
            ),
            title = resolvePlanTitle(request.missionMode, request.cameraProfile, request.planTitleDraft),
            takeoffPoint = request.takeoffPoint,
        )

        return PlannedMission(
            title = plan.title,
            polygon = request.polygon,
            waypoints = plan.waypoints,
            summary = buildCircleSummary(plan, request.takeoffPoint),
            warning = plan.estimate.warning,
            estimatedDistanceM = plan.estimatedDistanceM,
            estimatedDurationSec = plan.estimate.flightTimeSec,
            estimatedPhotoCount = plan.estimate.photos,
            baseAltitudeM = plan.parameters.altitudeM,
            maxSupportedSpeedMps = plan.estimate.maxSupportedSpeedMps,
            selectedSpeedMps = plan.estimate.selectedSpeedMps,
            selectedCapturePeriodSec = plan.estimate.selectedPeriodSec,
        )
    }

    private fun createHybridMission(request: PlanningRequest): PlannedMission {
        val gridMission = createGridMission(request.copy(missionMode = MissionMode.GRID_NADIR))
        val circleMission = createCircleMission(request.copy(missionMode = MissionMode.CIRCLEGRAMMETRY))
        val hybridWaypoints = gridMission.waypoints + circleMission.waypoints
        val totalDistanceM = computeRouteDistance(hybridWaypoints)
        val totalDurationSec = computeRouteDuration(hybridWaypoints)
        val warnings = listOfNotNull(gridMission.warning, circleMission.warning)
            .flatMap { it.split('\n') }
            .distinct()
            .joinToString(separator = "\n")
            .let { if (it.isBlank()) null else it }

        val summary = buildString {
            appendLine("mission: Hybrid")
            appendLine("grid photos: ${gridMission.estimatedPhotoCount}")
            appendLine("circle photos: ${circleMission.estimatedPhotoCount}")
            appendLine("total photos: ${gridMission.estimatedPhotoCount + circleMission.estimatedPhotoCount}")
            appendLine("total waypoints: ${hybridWaypoints.size}")
            appendLine("distance: ${formatMeters(totalDistanceM)}")
            appendLine("duration: ${formatDuration(totalDurationSec)}")
            appendLine(
                "takeoff: ${request.takeoffPoint?.let { formatLatLon(GeoPoint(it.lat, it.lon)) } ?: "not set"}",
            )
            append(
                if (request.takeoffPoint != null) {
                    getString(R.string.export_takeoff_yes)
                } else {
                    getString(R.string.export_takeoff_no)
                },
            )
        }

        return PlannedMission(
            title = resolvePlanTitle(request.missionMode, request.cameraProfile, request.planTitleDraft),
            polygon = request.polygon,
            waypoints = hybridWaypoints,
            summary = summary,
            warning = warnings,
            estimatedDistanceM = totalDistanceM,
            estimatedDurationSec = totalDurationSec,
            estimatedPhotoCount = gridMission.estimatedPhotoCount + circleMission.estimatedPhotoCount,
            baseAltitudeM = gridMission.baseAltitudeM,
            maxSupportedSpeedMps = minOf(gridMission.maxSupportedSpeedMps, circleMission.maxSupportedSpeedMps),
            selectedSpeedMps = minOf(gridMission.selectedSpeedMps, circleMission.selectedSpeedMps),
            selectedCapturePeriodSec = maxOf(
                gridMission.selectedCapturePeriodSec,
                circleMission.selectedCapturePeriodSec,
            ),
        )
    }

    private fun buildGridSummary(
        plan: GridSurveyPlan,
        takeoffPoint: TakeoffPoint?,
    ): String {
        return buildString {
            appendLine("mission: Grid Nadir")
            appendLine("camera: ${plan.cameraProfile.label}")
            appendLine("photo mode: ${plan.cameraProfile.photoMode.label}")
            appendLine("altitude: ${formatAltitudeMeters(plan.parameters.altitudeM)}")
            appendLine("gsd: ${formatDecimal(plan.metrics.footprint.gsdCmPerPixel)} cm/px")
            appendLine(
                "footprint: ${formatMeters(plan.metrics.footprint.footprintWidthM)} x " +
                    formatMeters(plan.metrics.footprint.footprintHeightM),
            )
            appendLine("line spacing: ${formatMeters(plan.metrics.lineSpacingM)}")
            appendLine("photo spacing: ${formatMeters(plan.metrics.photoSpacingM)}")
            appendLine("period: ${plan.metrics.timing.selectedPeriodSec}s")
            appendLine("speed: ${formatDecimal(plan.metrics.timing.selectedSpeedMps)} m/s")
            appendLine("max supported speed: ${formatDecimal(plan.metrics.timing.maxSupportedSpeedMps)} m/s")
            appendLine("waypoints: ${plan.waypoints.size}")
            appendLine("photos estimate: ${plan.estimatedPhotoCount}")
            appendLine("distance: ${formatMeters(plan.estimatedDistanceM)}")
            appendLine("duration: ${formatDuration(plan.estimatedDurationSec)}")
            appendLine(
                "takeoff: ${takeoffPoint?.let { formatLatLon(GeoPoint(it.lat, it.lon)) } ?: "not set"}",
            )
            append(
                if (takeoffPoint != null) {
                    getString(R.string.export_takeoff_yes)
                } else {
                    getString(R.string.export_takeoff_no)
                },
            )
        }
    }

    private fun buildCircleSummary(
        plan: CirclegrammetryPlan,
        takeoffPoint: TakeoffPoint?,
    ): String {
        val selectedOverlap = plan.layout.optimization?.overlap ?: plan.parameters.requestedOverlap
        return buildString {
            appendLine("mission: Assisted Circlegrammetry")
            appendLine("camera: ${plan.parameters.cameraProfile.label}")
            appendLine("photo mode: ${plan.parameters.photoMode.label}")
            appendLine("altitude: ${formatAltitudeMeters(plan.parameters.altitudeM)}")
            appendLine("radius: ${formatMeters(plan.geometry.radiusM)}")
            appendLine("focus height: ${formatMeters(plan.geometry.focusHeightM)}")
            appendLine("off-nadir: ${formatDecimal(plan.geometry.offNadirAngleDeg)} deg")
            appendLine("tilt: ${formatDecimal(plan.parameters.cameraTiltDeg)} deg")
            appendLine("overlap: ${formatDecimal(selectedOverlap * 100.0)} %")
            appendLine("grid: ${plan.layout.circlesX} x ${plan.layout.circlesY}")
            appendLine("outside ext X: ${formatMeters(plan.layout.footprint.extensionXEachSideM)}")
            appendLine("outside ext Y: ${formatMeters(plan.layout.footprint.extensionYEachSideM)}")
            appendLine("period: ${plan.estimate.selectedPeriodSec}s")
            appendLine("speed: ${formatDecimal(plan.estimate.selectedSpeedMps)} m/s")
            appendLine("max supported speed: ${formatDecimal(plan.estimate.maxSupportedSpeedMps)} m/s")
            appendLine("waypoints: ${plan.waypoints.size}")
            appendLine("photos estimate: ${plan.estimate.photos}")
            appendLine("distance: ${formatMeters(plan.estimatedDistanceM)}")
            appendLine("duration: ${formatDuration(plan.estimate.flightTimeSec)}")
            appendLine(
                "takeoff: ${takeoffPoint?.let { formatLatLon(GeoPoint(it.lat, it.lon)) } ?: "not set"}",
            )
            append(
                if (takeoffPoint != null) {
                    getString(R.string.export_takeoff_yes)
                } else {
                    getString(R.string.export_takeoff_no)
                },
            )
        }
    }

    private fun buildRouteSamplePoints(
        mission: PlannedMission,
        takeoffPoint: TakeoffPoint?,
        waypoints: List<FlightWaypoint> = mission.waypoints,
    ): List<RouteSamplePoint> {
        return TerrainSampling.sampleRouteSegmentsBySpacing(
            exporter.buildWaypointsForExport(
                waypoints,
                takeoffPoint,
                mission.baseAltitudeM,
            ),
            ROUTE_TERRAIN_SAMPLE_SPACING_M,
        )
    }

    private fun buildTerrainSnapshot(
        polygon: List<GeoPoint>,
        takeoff: TakeoffPoint?,
        waypoints: List<FlightWaypoint>,
        routeSamplePoints: List<RouteSamplePoint>,
    ): TerrainSnapshot {
        val polygonElevations = polygon.map(::fetchElevationSafely)
        val takeoffElevationM = takeoff?.let { fetchElevationSafely(GeoPoint(it.lat, it.lon)) }
        val waypointElevations = waypoints.map { waypoint ->
            fetchElevationSafely(GeoPoint(waypoint.latitude, waypoint.longitude))
        }
        val routeSampleElevations = routeSamplePoints.map { fetchElevationSafely(it.point) }
        val routeSummary = if (routeSamplePoints.isEmpty()) {
            null
        } else {
            TerrainSampling.summarizeElevations(routeSampleElevations)
        }

        return TerrainSnapshot(
            polygonElevations = polygonElevations,
            takeoffElevationM = takeoffElevationM,
            waypointElevations = waypointElevations,
            routeSampleElevations = routeSampleElevations,
            routeSummary = routeSummary,
        )
    }

    private fun applyTerrainAdjustedMission(
        mission: PlannedMission,
        takeoff: TakeoffPoint?,
        snapshot: TerrainSnapshot,
    ): PlannedMission {
        if (takeoff == null) {
            return mission
        }

        val takeoffElevationM = snapshot.takeoffElevationM ?: throw ExportValidationException(
            getString(R.string.export_validation_takeoff_elevation_missing),
        )

        if (snapshot.waypointElevations.isEmpty()) {
            throw ExportValidationException(
                getString(R.string.export_validation_waypoints_unavailable),
            )
        }

        var lowestAbsoluteAltitudeM = Double.POSITIVE_INFINITY
        var missingCount = 0
        val adjustedWaypoints = mission.waypoints.mapIndexed { index, waypoint ->
            val waypointGround = snapshot.waypointElevations.getOrNull(index)
            if (waypointGround == null) {
                missingCount += 1
                return@mapIndexed waypoint
            }

            val correctedAltitude = mission.baseAltitudeM + (waypointGround - takeoffElevationM)
            val absoluteAltitudeM = takeoffElevationM + correctedAltitude
            lowestAbsoluteAltitudeM = minOf(lowestAbsoluteAltitudeM, absoluteAltitudeM)
            waypoint.copy(altitudeM = correctedAltitude)
        }

        if (missingCount > 0) {
            throw ExportValidationException(
                getString(R.string.export_validation_waypoints_missing, missingCount),
            )
        }

        var routeMissingCount = 0
        var unsafeRouteCount = 0
        snapshot.routeSampleElevations.forEach { routeGround ->
            if (routeGround == null) {
                routeMissingCount += 1
                return@forEach
            }

            val correctedAltitude = mission.baseAltitudeM + (routeGround - takeoffElevationM)
            val absoluteAltitudeM = takeoffElevationM + correctedAltitude
            lowestAbsoluteAltitudeM = minOf(lowestAbsoluteAltitudeM, absoluteAltitudeM)
            if (correctedAltitude < MIN_TERRAIN_ADJUSTED_ALTITUDE_M) {
                unsafeRouteCount += 1
            }
        }

        if (routeMissingCount > 0) {
            throw ExportValidationException(
                getString(R.string.export_validation_route_missing, routeMissingCount),
            )
        }

        val unsafeWaypointCount = adjustedWaypoints.count { it.altitudeM < MIN_TERRAIN_ADJUSTED_ALTITUDE_M }
        if (unsafeWaypointCount > 0 || unsafeRouteCount > 0) {
            val maxTakeoffElevationM = lowestAbsoluteAltitudeM - MIN_TERRAIN_ADJUSTED_ALTITUDE_M
            throw ExportValidationException(
                getString(R.string.export_validation_lower_takeoff, formatMeters(maxTakeoffElevationM)),
            )
        }

        return mission.copy(
            waypoints = adjustedWaypoints,
            warning = mission.warning,
        )
    }

    private fun buildTerrainAdjustedExportWaypoints(
        mission: PlannedMission,
        takeoff: TakeoffPoint?,
        snapshot: TerrainSnapshot,
        routeSamplePoints: List<RouteSamplePoint>,
    ): List<FlightWaypoint> {
        val exportWaypoints = exporter.buildWaypointsForExport(
            mission.waypoints,
            takeoff,
            mission.baseAltitudeM,
        )
        if (takeoff == null || routeSamplePoints.isEmpty()) {
            return exportWaypoints
        }

        val takeoffElevationM = snapshot.takeoffElevationM ?: return exportWaypoints
        val samplesBySegment = routeSamplePoints.withIndex().groupBy { it.value.segmentStartIndex }

        return buildList {
            add(exportWaypoints.first())
            for (segmentIndex in 0 until exportWaypoints.lastIndex) {
                val startWaypoint = exportWaypoints[segmentIndex]
                val endWaypoint = exportWaypoints[segmentIndex + 1]

                if (shouldDensifyTerrainSegment(startWaypoint)) {
                    samplesBySegment[segmentIndex]
                        .orEmpty()
                        .sortedBy { it.value.ratio }
                        .forEach { indexedSample ->
                            val terrainElevation = snapshot.routeSampleElevations.getOrNull(indexedSample.index)
                                ?: return@forEach
                            val correctedAltitude = mission.baseAltitudeM + (terrainElevation - takeoffElevationM)
                            add(
                                FlightWaypoint(
                                    latitude = indexedSample.value.point.lat,
                                    longitude = indexedSample.value.point.lon,
                                    altitudeM = correctedAltitude,
                                    yawDeg = startWaypoint.yawDeg,
                                    speedMps = startWaypoint.speedMps,
                                    actions = emptyList(),
                                ),
                            )
                        }
                }

                add(endWaypoint)
            }
        }
    }

    private fun shouldDensifyTerrainSegment(startWaypoint: FlightWaypoint): Boolean {
        return startWaypoint.segmentTypeToNext == WaypointSegmentType.LINEAR
    }

    private fun mergeWarnings(
        currentWarning: String?,
        additions: List<String>,
    ): String? {
        val merged = listOfNotNull(currentWarning)
            .flatMap { it.split('\n') }
            .map(String::trim)
            .filter(String::isNotBlank) +
            additions.filter(String::isNotBlank)
        val distinct = merged.distinct()
        return if (distinct.isEmpty()) null else distinct.joinToString(separator = "\n")
    }

    private fun fetchElevationSafely(point: GeoPoint): Double? {
        return runCatching {
            terrainService.fetchElevation(point.lat, point.lon)
        }.getOrNull()
    }

    private fun clearPlanningOutputs() {
        latestMissionRevision += 1
        invalidateJsonPreview(showPendingState = false)
        preparedExport = null
        binding.exportPathText.text = getString(R.string.export_path_placeholder)
    }

    private fun invalidateJsonPreview(showPendingState: Boolean) {
        latestJson = ""
        jsonPreviewRequestSequence.incrementAndGet()
        binding.jsonPreviewText.text = getString(
            if (showPendingState) {
                R.string.json_preview_pending
            } else {
                R.string.json_placeholder
            },
        )
    }

    private fun setJsonPreviewText(jsonPayload: String) {
        binding.jsonPreviewText.text = if (jsonPayload.length <= JSON_PREVIEW_MAX_CHARS) {
            jsonPayload
        } else {
            buildString {
                append(jsonPayload.take(JSON_PREVIEW_MAX_CHARS))
                appendLine()
                appendLine()
                append(
                    getString(
                        R.string.json_preview_truncated,
                        jsonPayload.length - JSON_PREVIEW_MAX_CHARS,
                    ),
                )
            }
        }
    }

    private fun ensureLatestJsonPreview(force: Boolean = false) {
        val mission = latestMission ?: run {
            binding.jsonPreviewText.text = getString(R.string.json_placeholder)
            return
        }
        if (!force && latestJson.isNotBlank()) {
            return
        }

        val missionRevision = latestMissionRevision
        val requestId = jsonPreviewRequestSequence.incrementAndGet()
        val takeoffPoint = currentTakeoffPoint
        binding.jsonPreviewText.text = getString(R.string.json_preview_pending)
        plannerExecutor.execute {
            val result = runCatching {
                exporter.export(
                    title = mission.title,
                    polygon = mission.polygon,
                    waypoints = mission.waypoints,
                    takeoffPoint = takeoffPoint,
                    transferAltitudeM = mission.baseAltitudeM,
                )
            }
            runOnUiThread {
                if (requestId != jsonPreviewRequestSequence.get() || missionRevision != latestMissionRevision) {
                    return@runOnUiThread
                }
                result.onSuccess { jsonPayload ->
                    latestJson = jsonPayload
                    setJsonPreviewText(jsonPayload)
                }.onFailure { error ->
                    binding.jsonPreviewText.text = getString(R.string.json_placeholder)
                    Toast.makeText(
                        this,
                        getString(R.string.plan_generation_failed, error.message ?: error.javaClass.simpleName),
                        Toast.LENGTH_SHORT,
                    ).show()
                }
            }
        }
    }

    private fun writePreparedExport(skipOverwriteCheck: Boolean = false) {
        val readyExport = preparedExport
        if (readyExport == null) {
            Toast.makeText(this, R.string.json_export_missing_plan, Toast.LENGTH_SHORT).show()
            return
        }
        if (currentTakeoffPoint == null) {
            Toast.makeText(this, R.string.json_export_home_required, Toast.LENGTH_LONG).show()
            return
        }

        val treeUri = getPersistedFreeFlightTreeUri()
        if (treeUri == null) {
            pendingExportAfterFolderSelection = true
            Toast.makeText(this, R.string.json_export_select_folder, Toast.LENGTH_LONG).show()
            freeFlightFolderLauncher.launch(null)
            return
        }

        val exportPlanTitle = selectedPlanTitle(selectedCameraProfile())
        if (!skipOverwriteCheck) {
            val existingPath = findExistingExportPath(treeUri, exportPlanTitle)
            if (existingPath != null) {
                showOverwriteExportDialog(existingPath)
                return
            }
        }

        val requestId = terrainRequestSequence.incrementAndGet()
        terrainLoading = true
        terrainErrorMessage = null
        binding.exportPathText.text = getString(R.string.json_export_preparing)
        renderProjectState()
        syncMapProject()

        terrainExecutor.execute {
            val takeoff = currentTakeoffPoint ?: return@execute
            val result = runCatching<Pair<PreparedExport, String>> {
                val missionToWrite = readyExport.mission.copy(title = exportPlanTitle)
                val jsonToWrite = exporter.exportPrepared(
                    title = missionToWrite.title,
                    polygon = missionToWrite.polygon,
                    waypoints = readyExport.exportWaypoints,
                    takeoffPoint = takeoff,
                )
                val exportPath = exportToFreeFlightFolder(
                    treeUri = treeUri,
                    planTitle = missionToWrite.title,
                    jsonPayload = jsonToWrite,
                )
                Pair(
                    PreparedExport(
                        snapshot = readyExport.snapshot,
                        mission = missionToWrite,
                        exportWaypoints = readyExport.exportWaypoints,
                        jsonPayload = jsonToWrite,
                    ),
                    exportPath,
                )
            }

            runOnUiThread {
                if (requestId != terrainRequestSequence.get()) {
                    return@runOnUiThread
                }

                terrainLoading = false
                result.onSuccess { prepared ->
                    val writtenExport = prepared.first
                    val exportPath = prepared.second
                    preparedExport = writtenExport
                    terrainSnapshot = writtenExport.snapshot
                    terrainErrorMessage = null
                    latestMissionRevision += 1
                    jsonPreviewRequestSequence.incrementAndGet()
                    latestMission = writtenExport.mission
                    latestJson = writtenExport.jsonPayload
                    setJsonPreviewText(writtenExport.jsonPayload)
                    binding.exportPathText.text = exportPath
                    Toast.makeText(
                        this,
                        getString(R.string.json_exported, exportPath),
                        Toast.LENGTH_LONG,
                    ).show()
                }.onFailure { error ->
                    terrainErrorMessage = error.message ?: error.javaClass.simpleName
                    binding.exportPathText.text = getString(R.string.export_path_placeholder)
                    Toast.makeText(
                        this,
                        getString(R.string.json_export_failed, error.message ?: error.javaClass.simpleName),
                        Toast.LENGTH_LONG,
                    ).show()
                }
                renderProjectState()
                syncMapProject()
            }
        }
    }

    private fun showOverwriteExportDialog(existingPath: String) {
        showNonDimmedDialog(
            AlertDialog.Builder(this)
            .setTitle(R.string.json_export_overwrite_title)
            .setMessage(getString(R.string.json_export_overwrite_message, existingPath))
            .setNegativeButton(R.string.close_button, null)
            .setPositiveButton(R.string.json_export_overwrite_confirm) { _, _ ->
                setSaveDialogVisible(false)
                writePreparedExport(skipOverwriteCheck = true)
            },
        )
    }

    private fun showNonDimmedDialog(builder: AlertDialog.Builder) {
        val dialog = builder.create()
        dialog.setOnShowListener {
            dialog.window?.setDimAmount(0f)
            dialog.window?.clearFlags(WindowManager.LayoutParams.FLAG_DIM_BEHIND)
        }
        dialog.show()
    }

    private fun syncMapProject() {
        if (!mapReady) {
            return
        }
        val focusProject = focusMapOnNextSync
        focusMapOnNextSync = false
        val projectJson = buildMapProjectJson(
            rawPolygonPoints = currentPolygonPoints,
            polygon = currentPolygon,
            polygonElevations = terrainSnapshot.polygonElevations,
            takeoff = currentTakeoffPoint,
            takeoffElevationM = terrainSnapshot.takeoffElevationM,
            waypoints = latestMission?.let {
                reducePreviewWaypoints(
                    exporter.buildWaypointsForExport(it.waypoints, currentTakeoffPoint, it.baseAltitudeM),
                )
            }.orEmpty(),
            focusProject = focusProject,
            gridAngleDeg = parseOptionalDouble(binding.gridAngleInput.text?.toString(), "Grid angle") ?: 0.0,
            missionMode = selectedMissionMode(),
        )
        evaluateJavascriptSafely("window.plannerMap.renderProject($projectJson);")
    }

    @SuppressLint("MissingPermission")
    private fun startLocationTracking() {
        if (!mapReady) {
            return
        }
        if (!hasLocationPermission()) {
            locationPermissionLauncher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                ),
            )
            return
        }
        if (isLocationTrackingActive) {
            return
        }

        val locationManager = getSystemService(LocationManager::class.java)
        if (locationManager == null) {
            Toast.makeText(this, R.string.location_provider_unavailable, Toast.LENGTH_SHORT).show()
            return
        }

        val enabledProviders = runCatching { locationManager.getProviders(true) }.getOrDefault(emptyList())
        val providers = buildList {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                add(LocationManager.GPS_PROVIDER)
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                add(LocationManager.NETWORK_PROVIDER)
            }
            enabledProviders.forEach { provider ->
                if (!contains(provider)) {
                    add(provider)
                }
            }
        }

        if (providers.isEmpty()) {
            findBestLastKnownLocation(locationManager)?.let {
                showUserLocationOnMap(it, focus = !userLocationCentered)
                userLocationCentered = true
            }
                ?: Toast.makeText(this, R.string.location_provider_unavailable, Toast.LENGTH_SHORT).show()
            return
        }

        providers.forEach { provider ->
            runCatching {
                locationManager.requestLocationUpdates(
                    provider,
                    LOCATION_UPDATE_MIN_TIME_MS,
                    LOCATION_UPDATE_MIN_DISTANCE_M,
                    locationListener,
                    mainLooper,
                )
            }
        }

        findBestLastKnownLocation(locationManager)?.let {
            showUserLocationOnMap(it, focus = !userLocationCentered)
            userLocationCentered = true
        }
        isLocationTrackingActive = true
    }

    private fun stopLocationTracking() {
        if (!isLocationTrackingActive) {
            return
        }
        val locationManager = getSystemService(LocationManager::class.java) ?: return
        runCatching {
            locationManager.removeUpdates(locationListener)
        }
        isLocationTrackingActive = false
    }

    private fun setMapMode(mode: String) {
        if (!mapReady) {
            return
        }
        evaluateJavascriptSafely("window.plannerMap.setMode('$mode');")
    }

    private fun evaluateJavascriptSafely(script: String) {
        runCatching {
            binding.mapWebView.evaluateJavascript(script, null)
        }.onFailure { error ->
            Toast.makeText(
                this,
                getString(R.string.map_bridge_error, error.message ?: error.javaClass.simpleName),
                Toast.LENGTH_SHORT,
            ).show()
        }
    }

    private fun buildMapProjectJson(
        rawPolygonPoints: List<GeoPoint>,
        polygon: List<GeoPoint>,
        polygonElevations: List<Double?>,
        takeoff: TakeoffPoint?,
        takeoffElevationM: Double?,
        waypoints: List<FlightWaypoint>,
        focusProject: Boolean,
        gridAngleDeg: Double,
        missionMode: MissionMode,
    ): String {
        val root = JSONObject()
        root.put("focusProject", focusProject)
        root.put("gridAngleDeg", ((gridAngleDeg % 360.0) + 360.0) % 360.0)
        root.put("showGridRotationHandle", polygon.size >= 3 && missionMode != MissionMode.CIRCLEGRAMMETRY)
        root.put("rawPolygonPoints", JSONArray().apply {
            rawPolygonPoints.forEach { point ->
                put(
                    JSONObject().apply {
                        put("lat", point.lat)
                        put("lon", point.lon)
                    },
                )
            }
        })
        root.put("polygon", JSONArray().apply {
            polygon.forEachIndexed { index, point ->
                put(
                    JSONObject().apply {
                        put("lat", point.lat)
                        put("lon", point.lon)
                        polygonElevations.getOrNull(index)?.let { put("elevationM", it) }
                    },
                )
            }
        })
        root.put(
            "takeoff",
            takeoff?.let {
                JSONObject().apply {
                    put("lat", it.lat)
                    put("lon", it.lon)
                    put("isUserConfirmed", it.isUserConfirmed)
                    takeoffElevationM?.let { elevation -> put("elevationM", elevation) }
                }
            } ?: JSONObject.NULL,
        )
        root.put("waypoints", JSONArray().apply {
            waypoints.forEach { waypoint ->
                put(
                    JSONObject().apply {
                        put("lat", waypoint.latitude)
                        put("lon", waypoint.longitude)
                    },
                )
            }
        })
        root.put(
            "center",
            JSONObject().apply {
                val center = polygon.firstOrNull()
                    ?: takeoff?.let { GeoPoint(it.lat, it.lon) }
                    ?: GeoPoint(45.0703, 7.6869)
                put("lat", center.lat)
                put("lon", center.lon)
            },
        )
        return root.toString()
    }

    private fun reducePreviewWaypoints(waypoints: List<FlightWaypoint>): List<FlightWaypoint> {
        if (waypoints.size <= MAP_PREVIEW_MAX_WAYPOINTS) {
            return waypoints
        }
        val interiorMax = (MAP_PREVIEW_MAX_WAYPOINTS - 2).coerceAtLeast(1)
        val stride = kotlin.math.ceil((waypoints.size - 2).toDouble() / interiorMax.toDouble())
            .toInt()
            .coerceAtLeast(1)
        return buildList {
            add(waypoints.first())
            var index = 1
            while (index < waypoints.lastIndex) {
                add(waypoints[index])
                index += stride
            }
            val lastWaypoint = waypoints.last()
            if (lastOrNull()?.sameCoordinates(lastWaypoint) != true) {
                add(lastWaypoint)
            }
        }
    }

    private fun buildMapStatusLine(): String {
        val mission = latestMission
        if (mission == null) {
            return if (currentPolygon.size >= 3) {
                getString(R.string.map_status_ready, currentPolygon.size)
            } else {
                getString(R.string.map_status_idle)
            }
        }

        val modeLabel = selectedMissionMode().label
        val altitude = formatAltitudeMeters(mission.baseAltitudeM)
        val photos = getString(R.string.map_status_photos, mission.estimatedPhotoCount)
        val duration = formatDuration(mission.estimatedDurationSec)
        return getString(R.string.map_status_compact, modeLabel, altitude, photos, duration)
    }

    private fun buildSaveDialogSummary(mission: PlannedMission): String {
        val modeLabel = selectedMissionMode().label
        val altitude = formatAltitudeMeters(mission.baseAltitudeM)
        val distance = formatMeters(mission.estimatedDistanceM)
        val duration = formatDuration(mission.estimatedDurationSec)
        val photoCount = mission.estimatedPhotoCount
        val waypointCount = mission.waypoints.size
        return buildString {
            appendLine(getString(R.string.save_summary_mode, modeLabel))
            appendLine(getString(R.string.save_summary_height, altitude))
            appendLine(getString(R.string.save_summary_duration, duration))
            appendLine(getString(R.string.save_summary_distance, distance))
            appendLine(getString(R.string.save_summary_photos, photoCount))
            appendLine(getString(R.string.save_summary_waypoints, waypointCount))
            append(
                if (currentTakeoffPoint != null) {
                    getString(R.string.save_summary_home_set)
                } else {
                    getString(R.string.save_summary_home_missing)
                },
            )
            mission.warning?.takeIf { it.isNotBlank() }?.let { warning ->
                appendLine()
                appendLine()
                appendLine(getString(R.string.save_dialog_warning_prefix))
                append(warning)
            }
        }
    }

    private fun exportToFreeFlightFolder(
        treeUri: Uri,
        planTitle: String,
        jsonPayload: String,
    ): String {
        val selectedRoot = DocumentFile.fromTreeUri(this, treeUri)
            ?: error(getString(R.string.export_folder_inaccessible))
        require(isValidFreeFlightSelection(selectedRoot)) {
            getString(R.string.json_export_wrong_folder)
        }
        val flightPlanRoot = resolveFlightPlanRoot(selectedRoot)
        val planFolderName = buildPlanFolderName(planTitle)
        val planFolder = flightPlanRoot.findFile(planFolderName)?.takeIf { it.isDirectory }
            ?: flightPlanRoot.createDirectory(planFolderName)
            ?: error(getString(R.string.export_folder_create_failed))

        val savedPlanFile = planFolder.findFile("savedPlan.json")?.takeIf { it.isFile }
            ?: planFolder.createFile("application/json", "savedPlan.json")
            ?: error(getString(R.string.export_saved_plan_create_failed))

        contentResolver.openOutputStream(savedPlanFile.uri, "wt")?.bufferedWriter().use { writer ->
            checkNotNull(writer) { getString(R.string.export_saved_plan_open_failed) }
            writer.write(jsonPayload)
        }

        val rootName = selectedRoot.name ?: "selected-root"
        val exportRootLabel = if (selectedRoot.name.equals("flightPlan", ignoreCase = true)) {
            rootName
        } else {
            "$rootName/flightPlan"
        }
        return "$exportRootLabel/$planFolderName/savedPlan.json"
    }

    private fun findExistingExportPath(
        treeUri: Uri,
        planTitle: String,
    ): String? {
        val selectedRoot = DocumentFile.fromTreeUri(this, treeUri) ?: return null
        if (!isValidFreeFlightSelection(selectedRoot)) {
            return null
        }
        val flightPlanRoot = resolveFlightPlanRoot(selectedRoot)
        val planFolderName = buildPlanFolderName(planTitle)
        val planFolder = flightPlanRoot.findFile(planFolderName)?.takeIf { it.isDirectory } ?: return null
        val savedPlanFile = planFolder.findFile("savedPlan.json")?.takeIf { it.isFile } ?: return null
        val rootName = selectedRoot.name ?: "selected-root"
        val exportRootLabel = if (selectedRoot.name.equals("flightPlan", ignoreCase = true)) {
            rootName
        } else {
            "$rootName/flightPlan"
        }
        return "$exportRootLabel/$planFolderName/${savedPlanFile.name ?: "savedPlan.json"}"
    }

    private fun resolveFlightPlanRoot(selectedRoot: DocumentFile): DocumentFile {
        if (selectedRoot.name.equals("flightPlan", ignoreCase = true)) {
            return selectedRoot
        }

        return selectedRoot.findFile("flightPlan")?.takeIf { it.isDirectory }
            ?: if (
                selectedRoot.name.equals("FreeFlight 3", ignoreCase = true) ||
                selectedRoot.name.equals("FreeFlight 6", ignoreCase = true)
            ) {
                selectedRoot.createDirectory("flightPlan")
            } else {
                null
            }
            ?: error(getString(R.string.json_export_wrong_folder))
    }

    private fun buildPlanFolderName(title: String): String {
        val sanitized = title
            .replace(invalidFolderCharsRegex, "_")
            .replace(whitespaceRegex, " ")
            .trim()
            .trim('.')
        return sanitized.ifBlank { "ANAFI_Plan_${System.currentTimeMillis()}" }
    }

    private fun persistFreeFlightTreeUri(uri: Uri) {
        getSharedPreferences(exportPrefsName, MODE_PRIVATE)
            .edit()
            .putString(prefFreeFlightTreeUri, uri.toString())
            .apply()
    }

    private fun getPersistedFreeFlightTreeUri(): Uri? {
        val uriString = getSharedPreferences(exportPrefsName, MODE_PRIVATE)
            .getString(prefFreeFlightTreeUri, null)
            ?: return null
        return Uri.parse(uriString)
    }

    private fun clearPersistedFreeFlightTreeUri() {
        getSharedPreferences(exportPrefsName, MODE_PRIVATE)
            .edit()
            .remove(prefFreeFlightTreeUri)
            .apply()
    }

    private fun hasLocationPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION,
        ) == PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_COARSE_LOCATION,
            ) == PackageManager.PERMISSION_GRANTED
    }

    private fun findBestLastKnownLocation(locationManager: LocationManager): Location? {
        return runCatching { locationManager.getProviders(true) }
            .getOrDefault(emptyList())
            .mapNotNull { provider ->
                runCatching { locationManager.getLastKnownLocation(provider) }.getOrNull()
            }
            .maxByOrNull { it.time }
    }

    private fun showUserLocationOnMap(location: Location, focus: Boolean) {
        val accuracyM = if (location.hasAccuracy()) location.accuracy.toDouble() else 0.0
        evaluateJavascriptSafely(
            "window.plannerMap.showUserLocation(${location.latitude},${location.longitude},$accuracyM,${focus.toString().lowercase(Locale.US)});",
        )
    }

    private fun selectedMissionMode(): MissionMode {
        return when (binding.missionModeGroup.checkedButtonId) {
            R.id.missionCircleButton -> MissionMode.CIRCLEGRAMMETRY
            R.id.missionHybridButton -> MissionMode.HYBRID
            else -> MissionMode.GRID_NADIR
        }
    }

    private fun selectedCameraProfile(): CameraProfile {
        return when (binding.cameraProfileGroup.checkedButtonId) {
            R.id.cameraWideJpgButton -> cameraProfiles.first { it.label == "WIDE JPG" }
            R.id.cameraJpgButton -> cameraProfiles.first { it.label == "JPG" }
            else -> cameraProfiles.first { it.label == "RAW" }
        }
    }

    private fun selectedCircleQuality(): CircleQuality {
        return when (binding.circleOptimizationSpinner.selectedItemPosition) {
            0 -> CircleQuality.FAST
            2 -> CircleQuality.HIGH
            else -> CircleQuality.NORMAL
        }
    }

    private fun selectedCircleFocusTarget(): FocusTarget {
        return when (binding.circleFocusTargetSpinner.selectedItemPosition) {
            1 -> FocusTarget(FocusTargetMode.CANOPY_CENTER, 3.0)
            2 -> FocusTarget(FocusTargetMode.OBJECT_CENTER, 1.5)
            3 -> FocusTarget(
                FocusTargetMode.CUSTOM,
                parseDouble(binding.circleFocusHeightInput.text?.toString(), "Custom focus height"),
            )

            else -> FocusTarget(FocusTargetMode.GROUND, 0.0)
        }
    }

    private fun selectedPlanTitle(cameraProfile: CameraProfile): String {
        return resolvePlanTitle(selectedMissionMode(), cameraProfile, flightPlanTitleDraft)
    }

    private fun resolvePlanTitle(
        missionMode: MissionMode,
        cameraProfile: CameraProfile,
        titleDraft: String?,
    ): String {
        val rawTitle = titleDraft?.trim().orEmpty()
        if (rawTitle.isNotEmpty()) {
            return rawTitle
        }
        return when (missionMode) {
            MissionMode.GRID_NADIR -> "Grid ${cameraProfile.label}"
            MissionMode.CIRCLEGRAMMETRY -> "Circle ${cameraProfile.label}"
            MissionMode.HYBRID -> "Hybrid ${cameraProfile.label}"
        }
    }

    private fun isValidFreeFlightSelection(selectedRoot: DocumentFile): Boolean {
        val rootName = selectedRoot.name.orEmpty()
        return rootName.equals("FreeFlight 3", ignoreCase = true) ||
            rootName.equals("FreeFlight 6", ignoreCase = true) ||
            rootName.equals("flightPlan", ignoreCase = true) ||
            selectedRoot.findFile("flightPlan")?.isDirectory == true
    }

    private fun selectedSpeedMode(maxSupportedSpeedMps: Double): SpeedMode {
        return selectedSpeedMode(manualSpeedOverrideMps, maxSupportedSpeedMps)
    }

    private fun selectedSpeedMode(
        requestedManualSpeedMps: Double?,
        maxSupportedSpeedMps: Double,
    ): SpeedMode {
        return if (effectiveManualSpeedOverride(requestedManualSpeedMps, maxSupportedSpeedMps) != null) {
            SpeedMode.CUSTOM
        } else {
            SpeedMode.RECOMMENDED
        }
    }

    private fun selectedRequestedSpeed(maxSupportedSpeedMps: Double): Double? {
        return selectedRequestedSpeed(manualSpeedOverrideMps, maxSupportedSpeedMps)
    }

    private fun effectiveManualSpeedOverride(maxSupportedSpeedMps: Double): Double? {
        return effectiveManualSpeedOverride(manualSpeedOverrideMps, maxSupportedSpeedMps)
    }

    private fun selectedRequestedSpeed(
        requestedManualSpeedMps: Double?,
        maxSupportedSpeedMps: Double,
    ): Double? {
        return effectiveManualSpeedOverride(requestedManualSpeedMps, maxSupportedSpeedMps)
    }

    private fun effectiveManualSpeedOverride(
        requestedManualSpeedMps: Double?,
        maxSupportedSpeedMps: Double,
    ): Double? {
        if (maxSupportedSpeedMps <= 5.0) {
            return null
        }
        val override = requestedManualSpeedMps?.coerceIn(1.0, maxSupportedSpeedMps) ?: return null
        return override.takeIf { it < maxSupportedSpeedMps - 0.24 }
    }

    private fun computeGridMaxSupportedSpeed(
        cameraProfile: CameraProfile,
        altitudeM: Double,
        frontOverlap: Double,
    ): Double {
        val footprint = PlannerMath.footprint(cameraProfile, altitudeM)
        val photoSpacingM = footprint.footprintHeightM * (1.0 - frontOverlap)
        return photoSpacingM / cameraProfile.photoMode.minPeriodSec
    }

    private fun computeCircleMaxSupportedSpeed(
        radiusM: Double,
        pointsPerCircle: Int,
        cameraProfile: CameraProfile,
    ): Double {
        val circumferenceM = 2.0 * PI * radiusM
        val photoSpacingM = circumferenceM / pointsPerCircle
        return photoSpacingM / cameraProfile.photoMode.minPeriodSec
    }

    private fun roundToHalfStep(value: Double): Double {
        return round(value * 2.0) / 2.0
    }

    private fun roundUpToHalfStep(value: Double): Double {
        return kotlin.math.ceil(value * 2.0) / 2.0
    }

    private fun normalizeGridAngle(value: Double): Int {
        val normalized = ((value % 360.0) + 360.0) % 360.0
        return normalized.toInt().mod(360)
    }

    private fun parseDouble(value: String?, label: String): Double {
        return value?.toDoubleOrNull()
            ?: throw IllegalArgumentException("$label is invalid.")
    }

    private fun parseOptionalDouble(value: String?, label: String): Double? {
        val trimmed = value?.trim().orEmpty()
        if (trimmed.isEmpty()) {
            return null
        }
        return trimmed.toDoubleOrNull()
            ?: throw IllegalArgumentException("$label is invalid.")
    }

    private fun parseInt(value: String?, label: String): Int {
        return value?.toIntOrNull()
            ?: throw IllegalArgumentException("$label is invalid.")
    }

    private fun parsePercent(value: String?, label: String): Double {
        val percent = parseDouble(value, label)
        require(percent in 0.0..99.0) { "$label must be within [0, 99]." }
        return percent / 100.0
    }

    private fun computeRouteDistance(waypoints: List<FlightWaypoint>): Double {
        return waypoints.zipWithNext().sumOf { (start, end) ->
            distanceMeters(
                GeoPoint(start.latitude, start.longitude),
                GeoPoint(end.latitude, end.longitude),
            )
        }
    }

    private fun computeRouteDuration(waypoints: List<FlightWaypoint>): Double {
        return waypoints.zipWithNext().sumOf { (start, end) ->
            val distanceM = distanceMeters(
                GeoPoint(start.latitude, start.longitude),
                GeoPoint(end.latitude, end.longitude),
            )
            distanceM / start.speedMps.coerceAtLeast(0.1)
        }
    }

    private fun distanceMeters(start: GeoPoint, end: GeoPoint): Double {
        val projection = com.fotogrammetria.anafiplanner.planner.GeoProjection(start)
        val a = projection.toLocalMeters(start)
        val b = projection.toLocalMeters(end)
        return a.distanceTo(b)
    }

    private fun formatLatLon(point: GeoPoint): String {
        return String.format(Locale.US, "%.6f, %.6f", point.lat, point.lon)
    }

    private fun formatAltitudeMeters(value: Double): String {
        return "${kotlin.math.round(value).toInt()} m"
    }

    private fun formatMeters(value: Double): String {
        return String.format(Locale.US, "%.1f m", value)
    }

    private fun formatDecimal(value: Double): String {
        return String.format(Locale.US, "%.2f", value)
    }

    private fun formatGroundElevation(value: Double?): String {
        return when {
            value != null -> String.format(Locale.US, "%.1f m", value)
            terrainLoading -> "loading..."
            else -> "n/a"
        }
    }

    private fun formatDuration(seconds: Double): String {
        val rounded = seconds.toInt()
        val minutes = rounded / 60
        val remainingSeconds = rounded % 60
        return String.format(Locale.US, "%02d:%02d", minutes, remainingSeconds)
    }

    override fun onDestroy() {
        stopLocationTracking()
        pendingGenerateRunnable?.let(mainHandler::removeCallbacks)
        pendingGenerateRunnable = null
        binding.mapWebView.removeJavascriptInterface("AndroidBridge")
        binding.mapWebView.destroy()
        plannerExecutor.shutdownNow()
        terrainExecutor.shutdownNow()
        super.onDestroy()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            applyImmersiveMode()
        }
    }

    inner class MapBridge {
        @JavascriptInterface
        fun onPolygonChanged(json: String) {
            val rawPoints = parsePolygon(json)
            runOnUiThread {
                val orderedPolygon = normalizeOrderedPolygon(rawPoints)
                if (hasSelfIntersection(orderedPolygon)) {
                    Toast.makeText(this@MainActivity, R.string.polygon_self_intersection, Toast.LENGTH_SHORT).show()
                    syncMapProject()
                    return@runOnUiThread
                }
                currentPolygonPoints = orderedPolygon
                currentPolygon = orderedPolygon
                generatePlan()
            }
        }

        @JavascriptInterface
        fun onPolygonInvalid() {
            runOnUiThread {
                Toast.makeText(this@MainActivity, R.string.polygon_self_intersection, Toast.LENGTH_SHORT).show()
                syncMapProject()
            }
        }

        @JavascriptInterface
        fun onTakeoffPointChanged(json: String) {
            val point = parseTakeoffPoint(json)
            runOnUiThread {
                currentTakeoffPoint = point
                takeoffPlacementMode = false
                updateTakeoffModeUi()
                generatePlan()
            }
        }

        @JavascriptInterface
        fun onMapReady() {
            runOnUiThread {
                mapReady = true
                syncMapProject()
                applyMapBaseLayer()
                setMapMode("polygon")
                startLocationTracking()
            }
        }

        @JavascriptInterface
        fun requestUserLocation() {
            runOnUiThread {
                startLocationTracking()
            }
        }

        @JavascriptInterface
        fun onGridAngleChanged(angleDeg: Double) {
            runOnUiThread {
                val normalizedAngle = normalizeGridAngle(angleDeg)
                if (binding.gridAngleInput.text?.toString() == normalizedAngle.toString()) {
                    return@runOnUiThread
                }
                binding.gridAngleInput.setText(normalizedAngle.toString())
                binding.gridAngleValueText.text = getString(R.string.grid_angle_display, normalizedAngle)
                maybeGeneratePlan()
            }
        }
    }

    private fun parsePolygon(json: String): List<GeoPoint> {
        val jsonArray = JSONArray(json)
        return buildList {
            for (index in 0 until jsonArray.length()) {
                val obj = jsonArray.getJSONObject(index)
                add(
                    GeoPoint(
                        lat = obj.getDouble("lat"),
                        lon = obj.getDouble("lon"),
                    ),
                )
            }
        }
    }

    private fun normalizeOrderedPolygon(points: List<GeoPoint>): List<GeoPoint> {
        val normalized = mutableListOf<GeoPoint>()
        points.forEach { point ->
            if (normalized.lastOrNull()?.isNear(point) != true) {
                normalized += point
            }
        }
        if (normalized.size >= 2 && normalized.first().isNear(normalized.last())) {
            normalized.removeAt(normalized.lastIndex)
        }
        return normalized
    }

    private fun hasSelfIntersection(points: List<GeoPoint>): Boolean {
        if (points.size < 4) {
            return false
        }

        val edgeCount = points.size
        for (firstIndex in 0 until edgeCount) {
            val firstNext = (firstIndex + 1) % edgeCount
            for (secondIndex in firstIndex + 1 until edgeCount) {
                val secondNext = (secondIndex + 1) % edgeCount
                val sharesVertex =
                    firstIndex == secondIndex ||
                        firstIndex == secondNext ||
                        firstNext == secondIndex ||
                        firstNext == secondNext
                if (sharesVertex) {
                    continue
                }

                if (
                    segmentsIntersect(
                        points[firstIndex],
                        points[firstNext],
                        points[secondIndex],
                        points[secondNext],
                    )
                ) {
                    return true
                }
            }
        }

        return false
    }

    private fun segmentsIntersect(
        aStart: GeoPoint,
        aEnd: GeoPoint,
        bStart: GeoPoint,
        bEnd: GeoPoint,
    ): Boolean {
        val o1 = orientation(aStart, aEnd, bStart)
        val o2 = orientation(aStart, aEnd, bEnd)
        val o3 = orientation(bStart, bEnd, aStart)
        val o4 = orientation(bStart, bEnd, aEnd)

        if (o1 != o2 && o3 != o4) {
            return true
        }

        if (o1 == 0 && onSegment(aStart, aEnd, bStart)) return true
        if (o2 == 0 && onSegment(aStart, aEnd, bEnd)) return true
        if (o3 == 0 && onSegment(bStart, bEnd, aStart)) return true
        if (o4 == 0 && onSegment(bStart, bEnd, aEnd)) return true
        return false
    }

    private fun orientation(a: GeoPoint, b: GeoPoint, c: GeoPoint): Int {
        val cross = (b.lon - a.lon) * (c.lat - a.lat) - (b.lat - a.lat) * (c.lon - a.lon)
        return when {
            abs(cross) <= 1e-10 -> 0
            cross > 0.0 -> 1
            else -> -1
        }
    }

    private fun onSegment(start: GeoPoint, end: GeoPoint, point: GeoPoint): Boolean {
        return point.lon <= maxOf(start.lon, end.lon) + 1e-10 &&
            point.lon >= minOf(start.lon, end.lon) - 1e-10 &&
            point.lat <= maxOf(start.lat, end.lat) + 1e-10 &&
            point.lat >= minOf(start.lat, end.lat) - 1e-10
    }

    private fun GeoPoint.isNear(other: GeoPoint): Boolean {
        return abs(lat - other.lat) < 1e-7 && abs(lon - other.lon) < 1e-7
    }

    private fun FlightWaypoint.sameCoordinates(other: FlightWaypoint): Boolean {
        return abs(latitude - other.latitude) < 1e-7 && abs(longitude - other.longitude) < 1e-7
    }

    private fun parseTakeoffPoint(json: String): TakeoffPoint? {
        val obj = JSONObject(json)
        if (obj.optBoolean("clear", false)) {
            return null
        }
        return TakeoffPoint(
            lat = obj.getDouble("lat"),
            lon = obj.getDouble("lon"),
            isUserConfirmed = obj.optBoolean("isUserConfirmed", true),
        )
    }

    private data class TerrainSnapshot(
        val polygonElevations: List<Double?>,
        val takeoffElevationM: Double?,
        val waypointElevations: List<Double?>,
        val routeSampleElevations: List<Double?>,
        val routeSummary: ElevationSummary?,
    ) {
        fun isEmpty(): Boolean {
            return polygonElevations.isEmpty() &&
                takeoffElevationM == null &&
                waypointElevations.isEmpty() &&
                routeSampleElevations.isEmpty() &&
                routeSummary == null
        }

        companion object {
            fun empty(): TerrainSnapshot {
                return TerrainSnapshot(
                    polygonElevations = emptyList(),
                    takeoffElevationM = null,
                    waypointElevations = emptyList(),
                    routeSampleElevations = emptyList(),
                    routeSummary = null,
                )
            }
        }
    }

    private data class PlannedMission(
        val title: String,
        val polygon: List<GeoPoint>,
        val waypoints: List<FlightWaypoint>,
        val summary: String,
        val warning: String?,
        val estimatedDistanceM: Double,
        val estimatedDurationSec: Double,
        val estimatedPhotoCount: Int,
        val baseAltitudeM: Double,
        val maxSupportedSpeedMps: Double,
        val selectedSpeedMps: Double,
        val selectedCapturePeriodSec: Int,
    )

    private data class PlanningRequest(
        val polygon: List<GeoPoint>,
        val takeoffPoint: TakeoffPoint?,
        val missionMode: MissionMode,
        val cameraProfile: CameraProfile,
        val planTitleDraft: String,
        val altitudeM: Double,
        val frontOverlap: Double,
        val sideOverlap: Double,
        val gridAngleDeg: Double,
        val circleQuality: CircleQuality,
        val circleFocusTarget: FocusTarget,
        val circleLockRadius: Boolean,
        val circleLockAngle: Boolean,
        val circleManualOverlap: Boolean,
        val circleManualPoints: Boolean,
        val circleMaxExtensionM: Double?,
        val circleRequestedOverlap: Double,
        val circleLockedRadiusM: Double,
        val circleLockedTiltDeg: Double,
        val circleManualPointsValue: Int,
        val manualSpeedOverrideMps: Double?,
    )

    private data class PreparedExport(
        val snapshot: TerrainSnapshot,
        val mission: PlannedMission,
        val exportWaypoints: List<FlightWaypoint>,
        val jsonPayload: String,
    )

    private class ExportValidationException(message: String) : IllegalStateException(message)

    private companion object {
        private const val JSON_PREVIEW_MAX_CHARS = 24000
        private const val PLAN_REGEN_DEBOUNCE_MS = 220L
        private const val MAP_PREVIEW_MAX_WAYPOINTS = 1200
        private const val ROUTE_TERRAIN_SAMPLE_SPACING_M = 10.0
        private const val MIN_TERRAIN_ADJUSTED_ALTITUDE_M = 3.0
        private const val LOCATION_UPDATE_MIN_TIME_MS = 1500L
        private const val LOCATION_UPDATE_MIN_DISTANCE_M = 1f
        private const val exportPrefsName = "freeflight_export"
        private const val prefFreeFlightTreeUri = "freeflight_tree_uri"
        private val invalidFolderCharsRegex = Regex("""[\\/:*?"<>|]""")
        private val whitespaceRegex = Regex("""\s+""")
    }
}
