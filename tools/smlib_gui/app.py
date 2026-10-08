# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import os
import sys

from .hud import HudStats, ViewportHud
from .history import SourceHistory
from .inspection import object_summary_text
from .model import ActiveObject, legacy_parts_from_objects
from .operations import test_runner
from .operations.assert_valid import (
    object_can_assert_valid,
    run_assert_valid,
    topology_source_expr,
)
from .operations.dump import object_can_dump, run_dump
from .operations.booleans import apply_boolean_operation
from .operations.file_io import (
    LOAD_FILE_FILTER,
    SAVE_FILE_FILTER,
    ensure_extension_for_filter,
    export_native_brep,
    export_occt_brep,
    export_usd_objects,
    file_kind_for_path,
    import_native_brep_object,
    import_occt_brep_objects,
    import_usd_objects,
)
from .operations.modify import apply_modify_operation
from .operations.overlays import (
    OVERLAY_KINDS,
    BoxOverlay,
    CurveSampleOverlay,
    DrawBatchOverlay,
    LineOverlay,
    PointOverlay,
    PolylineOverlay,
    SectionOverlay,
    SurfaceSampleOverlay,
    TopologyOverlay,
    brep_edge_highlight_from_edge,
    brep_edgeuse_highlight,
    brep_face_highlight_from_face,
    brep_face_surface_overlay,
    brep_loopuse_highlight,
    is_overlay,
    kernel_draw_overlay,
    make_overlay_object,
    object_bounds,
    object_center,
    sample_curve_overlay,
    section_overlay,
)
from .operations.picking import TopologyPick, topology_hits_for_ray
from .runtime import QtCore, QtGui, QtInteractor, QtWidgets, pv, smdev
from .scripting import execute_script
from .settings import (
    CHORD_TOLERANCE_FRACS,
    COLOR_MODE_BREP_ORIENTATION,
    COLOR_MODE_CYCLE,
    COLOR_MODE_OPTIONS,
    DISPLAY_MODE_OPTIONS,
    DISPLAY_MODE_SHADED_WIREFRAME,
    DISPLAY_MODES_WITH_BREP_WIREFRAME,
    DISPLAY_MODES_WITH_SHADED_SURFACE,
    MAX_ASPECT_RATIO_PRESETS,
    MAX_EDGE_LENGTH_FRACS,
    NORMALS_MODE_CYCLE,
    NORMALS_MODE_OFF,
    NORMALS_MODE_OPTIONS,
    TOPOLOGY_COUNTS_BREP,
    TOPOLOGY_COUNTS_CYCLE,
    TOPOLOGY_COUNTS_OPTIONS,
    UP_AXIS_OPTIONS,
    DisplaySettings,
    TessellationSettings,
    color_mode_orients_to_brep_normals,
)
from .tess_presets import (
    bbox_diagonal_from_meshes,
    chord_tolerance_for_index,
    cycle_index,
    max_aspect_ratio_for_index,
    max_edge_length_for_index,
)
from .tessellation import mesh_error_summary, objects_to_pyvista_batches, tessellate_objects
from .topology_stats import topology_count_rows
from . import user_test_catalog
from .viewport import ViewportController
from .widgets import PasteablePlainTextEdit, VerticalLabel

LEFT_PANEL_EXPANDED_WIDTH = 360
LEFT_PANEL_COLLAPSED_WIDTH = 40


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SMLib 3D Modeler")
        self.resize(1600, 900)

        self._objects: list[ActiveObject] = []
        self._parts: list[dict] = []
        self._current_source = ""
        self._selected_object_id: int | None = None
        self._first_display = True
        self._display_settings = DisplaySettings()
        self._tessellation_settings = TessellationSettings()
        self._topology_selection: TopologyPick | None = None
        self._topology_pick_last_screen: tuple[float, float] | None = None
        self._topology_pick_cycle_index = 0
        self._topology_pick_pixel_tolerance = 5.0
        self._topology_draw_cycle_key: tuple[object, ...] | None = None
        self._topology_draw_cycle_indices: dict[str, int] = {}
        self._topology_transient_overlay_ids: dict[str, int] = {}
        self._test_is_running = False
        self._bbox_diag = 1.0
        self._chord_preset_index = 0
        self._max_edge_preset_index = 0
        self._max_aspect_preset_index = 0
        self._source_history = SourceHistory(max_entries=50)
        self._user_test_path = user_test_catalog.DEFAULT_USER_TEST_PATH
        self._user_test_cases: list[user_test_catalog.UserTestCase] = []
        self._user_test_overrides: dict[int, str] = {}
        self._user_test_loading = False
        self._user_test_error = ""
        self._left_panel_collapsed = False

        self._setup_ui()
        self._apply_styles()
        self._reload_user_tests(restore_number=True)

    def _setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self._status_bar = QtWidgets.QLabel("")
        self._status_bar.setStyleSheet("color: #a0a8c0; font-size: 11px; padding: 4px 0;")
        self._status_bar.setWordWrap(True)
        self.statusBar().addWidget(self._status_bar, stretch=1)

        self._left_panel = QtWidgets.QWidget()
        self._left_panel.setObjectName("leftPanel")
        self._left_panel.setFixedWidth(LEFT_PANEL_EXPANDED_WIDTH)
        left_layout = QtWidgets.QVBoxLayout(self._left_panel)
        left_layout.setContentsMargins(12, 12, 12, 12)
        left_layout.setSpacing(8)

        header_row = QtWidgets.QHBoxLayout()
        header_row.setSpacing(6)
        self._user_test_title = QtWidgets.QLabel("User Tests")
        self._user_test_title.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: #7ec8e3; padding: 4px 0;"
        )
        header_row.addWidget(self._user_test_title, stretch=1)
        self._left_panel_toggle = QtWidgets.QToolButton()
        self._left_panel_toggle.setObjectName("leftPanelToggle")
        self._left_panel_toggle.setAutoRaise(True)
        self._left_panel_toggle.setCursor(QtCore.Qt.PointingHandCursor)
        self._left_panel_toggle.setFixedSize(24, 24)
        self._left_panel_toggle.setText("‹")
        self._left_panel_toggle.setToolTip("Collapse user tests")
        self._left_panel_toggle.setStyleSheet(
            "QToolButton { background: #2f334a; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 4px; font-size: 14px; } "
            "QToolButton:hover { background: #3a3f5c; }"
        )
        self._left_panel_toggle.clicked.connect(self._on_toggle_left_panel)
        header_row.addWidget(self._left_panel_toggle)
        left_layout.addLayout(header_row)

        self._left_panel_spine_label = VerticalLabel("User Tests")
        self._left_panel_spine_label.setObjectName("leftPanelSpine")
        spine_font = QtGui.QFont(self._user_test_title.font())
        spine_font.setPointSize(12)
        spine_font.setBold(True)
        self._left_panel_spine_label.setFont(spine_font)
        self._left_panel_spine_label.setToolTip("Expand user tests")
        self._left_panel_spine_label.hide()
        self._left_panel_spine_label.clicked.connect(self._on_toggle_left_panel)
        left_layout.addWidget(self._left_panel_spine_label, stretch=1)

        self._left_panel_body = QtWidgets.QWidget()
        self._left_panel_body.setObjectName("leftPanelBody")
        body_layout = QtWidgets.QVBoxLayout(self._left_panel_body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(8)

        user_test_hint = QtWidgets.QLabel(
            "Numbered sm / smdev scripts. "
            "Edit a case and press Run to play it back."
        )
        user_test_hint.setWordWrap(True)
        user_test_hint.setStyleSheet("color: #a0a8c0; font-size: 11px; background: transparent; border: none;")
        body_layout.addWidget(user_test_hint)

        picker_row = QtWidgets.QHBoxLayout()
        picker_row.setSpacing(6)
        self._user_test_combo = QtWidgets.QComboBox()
        combo_style = (
            "QComboBox { background: #1e2030; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 3px; padding: 3px 6px; "
            "font-size: 12px; } "
            "QComboBox::drop-down { border: none; } "
            "QComboBox QAbstractItemView { background: #1e2030; color: #e0e4f0; "
            "selection-background-color: #4c6ef5; }"
        )
        self._user_test_combo.setStyleSheet(combo_style)
        self._user_test_combo.currentIndexChanged.connect(self._on_user_test_combo_changed)
        picker_row.addWidget(self._user_test_combo, stretch=1)
        self._user_test_number = QtWidgets.QSpinBox()
        self._user_test_number.setRange(0, 999)
        self._user_test_number.setFixedWidth(72)
        self._user_test_number.setStyleSheet(
            "background: #1e2030; color: #e0e4f0; border: 1px solid #3d4059; "
            "border-radius: 3px; padding: 2px 4px; font-size: 12px;"
        )
        self._user_test_number.valueChanged.connect(self._on_user_test_number_changed)
        picker_row.addWidget(self._user_test_number)
        body_layout.addLayout(picker_row)

        self._source_edit = PasteablePlainTextEdit()
        self._source_edit.setPlaceholderText(
            "Paste or type sm.* / smdev.* commands, then press Run.\n\n"
            "Example:\n"
            "  box = sm.create_box((0, 0, 0), 10, 10, 10)\n"
            "  sm.fillet_edges(box, box.edges(), radius=1.0)\n"
            "  smdev.dump(box)"
        )
        font = QtGui.QFont("Courier New", 11)
        font.setStyleHint(QtGui.QFont.Monospace)
        self._source_edit.setFont(font)
        self._source_edit.setStyleSheet(
            "background: #1a1c2e; color: #a8d8ea; border: 1px solid #3d4059; "
            "border-radius: 6px; padding: 6px;"
        )
        self._source_edit.textChanged.connect(self._on_user_test_source_changed)
        body_layout.addWidget(self._source_edit, stretch=1)

        run_row = QtWidgets.QHBoxLayout()
        self._run_user_test_btn = QtWidgets.QPushButton("Run")
        self._run_user_test_btn.setStyleSheet(
            "QPushButton { background: #37b24d; color: white; font-weight: bold; "
            "border: none; border-radius: 6px; padding: 8px 16px; font-size: 13px; } "
            "QPushButton:hover { background: #40c057; } "
            "QPushButton:pressed { background: #2f9e44; }"
        )
        self._run_user_test_btn.clicked.connect(self._on_run_user_test)
        run_row.addWidget(self._run_user_test_btn)
        self._reload_user_test_btn = self._make_panel_button("Reload")
        self._save_user_test_btn = self._make_panel_button("Save")
        self._open_user_test_btn = self._make_panel_button("Open")
        self._reload_user_test_btn.clicked.connect(self._on_reload_user_tests)
        self._save_user_test_btn.clicked.connect(self._on_save_user_test)
        self._open_user_test_btn.clicked.connect(self._on_open_user_test_file)
        for btn in (self._reload_user_test_btn, self._save_user_test_btn, self._open_user_test_btn):
            run_row.addWidget(btn)
        run_row.addStretch()
        body_layout.addLayout(run_row)

        history_row = QtWidgets.QHBoxLayout()
        self._replay_source_btn = self._make_panel_button("Replay")
        self._history_prev_btn = self._make_panel_button("Prev")
        self._history_next_btn = self._make_panel_button("Next")
        self._replay_source_btn.clicked.connect(self._on_replay_source_history)
        self._history_prev_btn.clicked.connect(self._on_previous_source_history)
        self._history_next_btn.clicked.connect(self._on_next_source_history)
        for btn in (self._replay_source_btn, self._history_prev_btn, self._history_next_btn):
            btn.setEnabled(False)
            history_row.addWidget(btn)
        history_row.addStretch()
        body_layout.addLayout(history_row)

        self._user_test_path_label = QtWidgets.QLabel("")
        self._user_test_path_label.setWordWrap(True)
        self._user_test_path_label.setStyleSheet(
            "color: #687089; font-size: 10px; background: transparent; border: none;"
        )
        body_layout.addWidget(self._user_test_path_label)
        left_layout.addWidget(self._left_panel_body, stretch=1)

        run_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Return"), self._source_edit)
        run_shortcut.setContext(QtCore.Qt.WidgetShortcut)
        run_shortcut.activated.connect(self._on_run_user_test)
        save_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+S"), self._source_edit)
        save_shortcut.setContext(QtCore.Qt.WidgetShortcut)
        save_shortcut.activated.connect(self._on_save_user_test)

        self._plotter = QtInteractor(central)
        self._plotter.set_background("#1e2030", top="#2a2d3e")
        self._plotter.add_axes()
        self._viewport = ViewportController(self._plotter)
        self._hud = ViewportHud(self._plotter)
        self._default_tessellation_settings = TessellationSettings()
        self._plotter.installEventFilter(self)
        self._setup_hotkeys()
        main_layout.addWidget(self._left_panel)
        main_layout.addWidget(self._plotter, stretch=1)

        right_scroll = QtWidgets.QScrollArea()
        right_scroll.setObjectName("rightPanelScroll")
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        right_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        right_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        right_scroll.setFixedWidth(340)

        right_panel = QtWidgets.QWidget()
        right_panel.setObjectName("rightPanel")
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.setSpacing(8)

        file_group = self._make_collapsible_group("File")
        file_layout = QtWidgets.QGridLayout(file_group)
        file_layout.setSpacing(6)
        file_buttons = [
            ("Load", self._on_load_file),
            ("Save", self._on_save_file),
        ]
        for i, (label, callback) in enumerate(file_buttons):
            btn = self._make_panel_button(label)
            btn.clicked.connect(callback)
            file_layout.addWidget(btn, 0, i)
        right_layout.addWidget(file_group)

        view_group = self._make_collapsible_group("View")
        view_layout = QtWidgets.QVBoxLayout(view_group)
        view_layout.setSpacing(6)
        display_layout = QtWidgets.QFormLayout()
        display_layout.setSpacing(6)

        self._display_mode_combo = QtWidgets.QComboBox()
        for label, mode in DISPLAY_MODE_OPTIONS:
            self._display_mode_combo.addItem(label, mode)
        self._display_mode_combo.setStyleSheet(
            "QComboBox { background: #1e2030; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 3px; padding: 3px 6px; "
            "font-size: 12px; } "
            "QComboBox::drop-down { border: none; } "
            "QComboBox QAbstractItemView { background: #1e2030; color: #e0e4f0; "
            "selection-background-color: #4c6ef5; }"
        )
        for i in range(self._display_mode_combo.count()):
            if self._display_mode_combo.itemData(i) == self._display_settings.display_mode:
                self._display_mode_combo.setCurrentIndex(i)
                break
        self._display_mode_combo.currentIndexChanged.connect(self._on_display_mode_changed)
        display_layout.addRow("Mode:", self._display_mode_combo)

        self._color_mode_combo = QtWidgets.QComboBox()
        for label, mode in COLOR_MODE_OPTIONS:
            self._color_mode_combo.addItem(label, mode)
        self._color_mode_combo.setStyleSheet(self._display_mode_combo.styleSheet())
        for i in range(self._color_mode_combo.count()):
            if self._color_mode_combo.itemData(i) == self._display_settings.color_mode:
                self._color_mode_combo.setCurrentIndex(i)
                break
        self._color_mode_combo.currentIndexChanged.connect(self._on_color_mode_changed)
        display_layout.addRow("Color:", self._color_mode_combo)

        self._up_axis_combo = QtWidgets.QComboBox()
        for label, axis in UP_AXIS_OPTIONS:
            self._up_axis_combo.addItem(label, axis)
        self._up_axis_combo.setStyleSheet(self._display_mode_combo.styleSheet())
        for i in range(self._up_axis_combo.count()):
            if self._up_axis_combo.itemData(i) == self._display_settings.up_axis:
                self._up_axis_combo.setCurrentIndex(i)
                break
        self._up_axis_combo.currentIndexChanged.connect(self._on_up_axis_changed)
        display_layout.addRow("Up axis:", self._up_axis_combo)

        self._normals_mode_combo = QtWidgets.QComboBox()
        for label, mode in NORMALS_MODE_OPTIONS:
            self._normals_mode_combo.addItem(label, mode)
        self._normals_mode_combo.setStyleSheet(self._display_mode_combo.styleSheet())
        for i in range(self._normals_mode_combo.count()):
            if self._normals_mode_combo.itemData(i) == self._display_settings.normals_mode:
                self._normals_mode_combo.setCurrentIndex(i)
                break
        self._normals_mode_combo.currentIndexChanged.connect(self._on_normals_mode_changed)
        display_layout.addRow("Normals:", self._normals_mode_combo)

        self._edge_vertices_check = QtWidgets.QCheckBox("Show vertices")
        self._edge_vertices_check.setStyleSheet(
            "QCheckBox { color: #c0c8e0; background: transparent; " "border: none; font-size: 12px; }"
        )
        self._edge_vertices_check.setChecked(self._display_settings.show_vertices)
        self._edge_vertices_check.toggled.connect(self._on_edge_vertices_changed)
        display_layout.addRow("", self._edge_vertices_check)

        self._topology_counts_combo = QtWidgets.QComboBox()
        for label, mode in TOPOLOGY_COUNTS_OPTIONS:
            self._topology_counts_combo.addItem(label, mode)
        self._topology_counts_combo.setStyleSheet(self._display_mode_combo.styleSheet())
        for i in range(self._topology_counts_combo.count()):
            if self._topology_counts_combo.itemData(i) == self._display_settings.topology_counts_mode:
                self._topology_counts_combo.setCurrentIndex(i)
                break
        self._topology_counts_combo.currentIndexChanged.connect(self._on_topology_counts_mode_changed)
        display_layout.addRow("Counts:", self._topology_counts_combo)
        view_layout.addLayout(display_layout)

        view_grid = QtWidgets.QGridLayout()
        view_grid.setSpacing(6)

        view_buttons = [
            ("Front", self._on_view_front),
            ("Back", self._on_view_back),
            ("Left", self._on_view_left),
            ("Right", self._on_view_right),
            ("Top", self._on_view_top),
            ("Bottom", self._on_view_bottom),
            ("Iso", self._on_view_isometric),
            ("F-Iso", self._on_view_front_iso),
            ("Fit", self._on_fit_view),
            ("Save", self._on_save_view),
            ("Load", self._on_restore_view),
        ]
        for i, (label, callback) in enumerate(view_buttons):
            btn = self._make_panel_button(label)
            btn.clicked.connect(callback)
            view_grid.addWidget(btn, i // 4, i % 4)

        self._parallel_projection_check = QtWidgets.QCheckBox("Parallel")
        self._parallel_projection_check.setStyleSheet(
            "QCheckBox { color: #c0c8e0; background: transparent; " "border: none; font-size: 12px; }"
        )
        self._parallel_projection_check.toggled.connect(self._on_parallel_projection_changed)
        view_grid.addWidget(self._parallel_projection_check, 2, 3)
        view_layout.addLayout(view_grid)
        right_layout.addWidget(view_group)

        tess_group = self._make_collapsible_group("Tessellation", expanded=False)
        tess_vbox = QtWidgets.QVBoxLayout(tess_group)
        tess_vbox.setSpacing(4)

        self._chord_spin = self._make_tess_slider(tess_vbox, "Chord tol:", 0.001, 1.0, 0.001, 3, 0.05)
        self._chord_spin.valueChanged.connect(self._on_tess_changed)

        self._angle_spin = self._make_tess_slider(tess_vbox, "Angle (deg):", 1.0, 60.0, 0.5, 1, 25.0)
        self._angle_spin.valueChanged.connect(self._on_tess_changed)

        self._max_edge_spin = self._make_tess_slider(tess_vbox, "Max edge:", 0.0, 100.0, 0.1, 1, 0.0)
        self._max_edge_spin.valueChanged.connect(self._on_tess_changed)

        self._max_aspect_spin = self._make_tess_slider(tess_vbox, "Aspect:", 0.0, 20.0, 0.1, 1, 0.0)
        self._max_aspect_spin.valueChanged.connect(self._on_tess_changed)

        right_layout.addWidget(tess_group)

        object_group = self._make_collapsible_group("Objects", expanded=False)
        object_layout = QtWidgets.QVBoxLayout(object_group)
        object_layout.setSpacing(6)

        self._object_list = QtWidgets.QListWidget()
        self._object_list.setMaximumHeight(120)
        self._object_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self._object_list.setStyleSheet(
            "QListWidget { background: #1e2030; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 3px; font-size: 12px; } "
            "QListWidget::item { padding: 4px 6px; } "
            "QListWidget::item:selected { background: #4c6ef5; color: white; }"
        )
        self._object_list.currentRowChanged.connect(self._on_object_selection_changed)
        object_layout.addWidget(self._object_list)

        object_btn_row = QtWidgets.QHBoxLayout()
        self._delete_object_btn = QtWidgets.QPushButton("Delete")
        self._dump_object_btn = QtWidgets.QPushButton("Dump")
        self._assert_valid_object_btn = QtWidgets.QPushButton("AssertValid")
        for btn in (self._delete_object_btn, self._dump_object_btn, self._assert_valid_object_btn):
            btn.setEnabled(False)
            btn.setStyleSheet(
                "QPushButton { background: #2f334a; color: #e0e4f0; "
                "border: 1px solid #3d4059; border-radius: 4px; "
                "padding: 5px 10px; font-size: 12px; } "
                "QPushButton:hover { background: #3a3f5c; } "
                "QPushButton:disabled { color: #687089; background: #24283b; }"
            )
            object_btn_row.addWidget(btn)
        self._delete_object_btn.clicked.connect(self._on_delete_object)
        self._dump_object_btn.clicked.connect(self._on_dump_object)
        self._assert_valid_object_btn.clicked.connect(self._on_assert_valid_object)
        object_btn_row.addStretch()
        object_layout.addLayout(object_btn_row)

        self._object_info = QtWidgets.QPlainTextEdit()
        self._object_info.setReadOnly(True)
        self._object_info.setMaximumHeight(150)
        self._object_info.setStyleSheet(
            "background: #1a1c2e; color: #cdd6f4; border: 1px solid #3d4059; "
            "border-radius: 4px; padding: 5px; font-size: 11px;"
        )
        object_layout.addWidget(self._object_info)

        right_layout.addWidget(object_group)

        boolean_group = self._make_collapsible_group("Boolean", expanded=False)
        boolean_grid = QtWidgets.QGridLayout(boolean_group)
        boolean_grid.setSpacing(6)
        boolean_buttons = [
            ("Union", "union"),
            ("Diff", "difference"),
            ("Intersect", "intersection"),
            ("Merge", "merge"),
            ("Cookie", "cookie"),
            ("Imprint", "imprint"),
        ]
        for i, (label, operation) in enumerate(boolean_buttons):
            btn = self._make_panel_button(label)
            btn.clicked.connect(lambda _checked=False, op=operation: self._apply_boolean_operation(op))
            boolean_grid.addWidget(btn, i // 3, i % 3)
        right_layout.addWidget(boolean_group)

        modify_group = self._make_collapsible_group("Modify", expanded=False)
        modify_grid = QtWidgets.QGridLayout(modify_group)
        modify_grid.setSpacing(6)
        modify_buttons = [
            ("Heal BRep", "heal"),
            ("Stitch Solid", "stitch_solid"),
            ("Stitch Shell", "stitch_shell"),
            ("Normals", "unify_normals"),
            ("Mass", "mass_properties"),
        ]
        for i, (label, operation) in enumerate(modify_buttons):
            btn = self._make_panel_button(label)
            btn.clicked.connect(lambda _checked=False, op=operation: self._apply_modify_operation(op))
            modify_grid.addWidget(btn, i // 2, i % 2)
        right_layout.addWidget(modify_group)

        tests_group = self._make_collapsible_group("Tests", expanded=False)
        tests_layout = QtWidgets.QVBoxLayout(tests_group)
        tests_layout.setSpacing(6)

        self._test_suite_combo = QtWidgets.QComboBox()
        self._test_suite_combo.setStyleSheet(
            "QComboBox { background: #1e2030; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 3px; padding: 3px 6px; "
            "font-size: 12px; } "
            "QComboBox::drop-down { border: none; } "
            "QComboBox QAbstractItemView { background: #1e2030; color: #e0e4f0; "
            "selection-background-color: #4c6ef5; }"
        )
        for suite in test_runner.list_prog_test_suites():
            self._test_suite_combo.addItem(suite, suite)
        tests_layout.addWidget(self._test_suite_combo)

        test_btn_row = QtWidgets.QHBoxLayout()
        self._run_test_btn = self._make_panel_button("Run In GUI")
        self._run_test_btn.clicked.connect(self._on_run_test_suite)
        test_btn_row.addWidget(self._run_test_btn)
        tests_layout.addLayout(test_btn_row)

        self._test_log = QtWidgets.QPlainTextEdit()
        self._test_log.setReadOnly(True)
        self._test_log.setMaximumHeight(120)
        self._test_log.setStyleSheet(
            "background: #1a1c2e; color: #cdd6f4; border: 1px solid #3d4059; "
            "border-radius: 4px; padding: 5px; font-size: 10px;"
        )
        tests_layout.addWidget(self._test_log)
        self._test_suite_combo.currentIndexChanged.connect(self._on_test_suite_changed)
        right_layout.addWidget(tests_group)

        topology_group = self._make_collapsible_group("Topology Selection", expanded=False)
        topology_layout = QtWidgets.QVBoxLayout(topology_group)
        topology_layout.setSpacing(6)
        self._topology_summary = QtWidgets.QLabel("No topology selected.")
        self._topology_summary.setWordWrap(True)
        self._topology_summary.setStyleSheet("color: #c0c8e0; font-size: 11px; background: transparent; border: none;")
        topology_layout.addWidget(self._topology_summary)
        topology_grid = QtWidgets.QGridLayout()
        topology_grid.setSpacing(6)
        self._topology_action_buttons: dict[str, QtWidgets.QPushButton] = {}
        topology_buttons = [
            ("Draw Face", "face", self._add_picked_face_overlay),
            ("Draw Surface", "surface", self._add_picked_surface_overlay),
            ("Draw Edge", "edge", self._add_picked_edge_overlay),
            ("Draw Curve", "curve", self._add_picked_curve_overlay),
            ("Draw Edgeuse", "edgeuse", self._add_picked_edgeuse_overlay),
            ("Draw Loopuse", "loopuse", self._add_picked_loopuse_overlay),
            ("Dump", "dump", self._on_dump_topology),
            ("AssertValid", "assert_valid", self._on_assert_valid_topology),
        ]
        for i, (label, key, callback) in enumerate(topology_buttons):
            btn = self._make_panel_button(label)
            btn.clicked.connect(callback)
            btn.setEnabled(False)
            self._topology_action_buttons[key] = btn
            topology_grid.addWidget(btn, i // 2, i % 2)
        topology_layout.addLayout(topology_grid)
        right_layout.addWidget(topology_group)

        debug_group = self._make_collapsible_group("Debug", expanded=False)
        debug_grid = QtWidgets.QGridLayout(debug_group)
        debug_grid.setSpacing(6)
        debug_buttons = [
            ("Box", self._add_selected_box_overlay, False),
            ("Center", self._add_selected_center_overlay, False),
            ("Section", self._add_selected_section_overlay, False),
            ("Clear", self._clear_debug_overlays, False),
        ]
        for i, (label, callback, checkable) in enumerate(debug_buttons):
            btn = self._make_panel_button(label)
            if checkable:
                btn.setCheckable(True)
                btn.toggled.connect(callback)
            else:
                btn.clicked.connect(callback)
            debug_grid.addWidget(btn, i // 5, i % 5)
        right_layout.addWidget(debug_group)
        right_layout.addStretch(1)

        self._info_label = QtWidgets.QLabel("")
        self._info_label.setStyleSheet("color: #a0a8c0; font-size: 11px; padding: 4px 0;")
        self._info_label.setWordWrap(True)
        right_layout.addWidget(self._info_label)

        right_scroll.setWidget(right_panel)
        main_layout.addWidget(right_scroll)
        self._apply_initial_group_expansion(right_panel)
        self._refresh_test_panel()
        self._set_left_panel_collapsed(self._saved_left_panel_collapsed(), persist=False)

    def _make_collapsible_group(self, title: str, *, expanded: bool = True) -> QtWidgets.QGroupBox:
        """Create a right-panel group box that can collapse its contents."""
        group = QtWidgets.QGroupBox(title)
        group.setCheckable(True)
        # Keep expanded while children are added, then apply the startup state.
        group.setChecked(True)
        group.setProperty("smlib_start_expanded", expanded)
        # Cap growth so a collapsed neighbour's slack does not stretch this group.
        group.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        group.setStyleSheet(
            "QGroupBox { color: #c0c8e0; font-size: 12px; font-weight: bold; "
            "border: 1px solid #3d4059; border-radius: 6px; margin-top: 8px; "
            "padding-top: 16px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; "
            "padding: 0 4px; } "
            "QGroupBox::indicator { width: 12px; height: 12px; } "
            "QGroupBox::indicator:unchecked { "
            "border: 1px solid #687089; background: #1e2030; border-radius: 2px; } "
            "QGroupBox::indicator:checked { "
            "border: 1px solid #4c6ef5; background: #4c6ef5; border-radius: 2px; }"
        )
        group.toggled.connect(lambda checked, g=group: self._set_group_expanded(g, checked))
        return group

    def _apply_initial_group_expansion(self, panel: QtWidgets.QWidget) -> None:
        for group in panel.findChildren(QtWidgets.QGroupBox):
            if not group.isCheckable():
                continue
            expanded = bool(group.property("smlib_start_expanded"))
            if group.isChecked() != expanded:
                group.setChecked(expanded)
            else:
                self._set_group_expanded(group, expanded)

    @staticmethod
    def _set_layout_items_visible(layout: QtWidgets.QLayout | None, visible: bool) -> None:
        if layout is None:
            return
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            if widget is not None:
                widget.setVisible(visible)
            MainWindow._set_layout_items_visible(item.layout(), visible)

    def _set_group_expanded(self, group: QtWidgets.QGroupBox, expanded: bool) -> None:
        self._set_layout_items_visible(group.layout(), expanded)
        if expanded:
            group.setMaximumHeight(16777215)
            # Checkable QGroupBox re-enables children on expand; restore panel state.
            if hasattr(self, "_delete_object_btn"):
                has_selection = self._selected_object() is not None
                self._delete_object_btn.setEnabled(has_selection)
                self._dump_object_btn.setEnabled(object_can_dump(self._selected_object()))
                if hasattr(self, "_assert_valid_object_btn"):
                    self._assert_valid_object_btn.setEnabled(
                        object_can_assert_valid(self._selected_object())
                    )
            if hasattr(self, "_topology_summary"):
                self._update_topology_selection_panel()
            if hasattr(self, "_run_test_btn"):
                self._refresh_test_panel(reset_log=False)
        else:
            # Leave room for the title/checkbox only.
            group.setMaximumHeight(max(28, group.fontMetrics().height() + 18))
        group.updateGeometry()

    def _make_panel_button(self, label: str) -> QtWidgets.QPushButton:
        btn = QtWidgets.QPushButton(label)
        btn.setStyleSheet(
            "QPushButton { background: #2f334a; color: #e0e4f0; "
            "border: 1px solid #3d4059; border-radius: 4px; "
            "padding: 4px 8px; font-size: 12px; } "
            "QPushButton:hover { background: #3a3f5c; } "
            "QPushButton:pressed { background: #25293d; } "
            "QPushButton:checked { background: #4c6ef5; color: white; }"
        )
        return btn

    def _make_tess_slider(self, layout, label, lo, hi, step, decimals, default):
        slider_style = (
            "QSlider { background: transparent; border: none; } "
            "QSlider::groove:horizontal { background: #1e2030; height: 4px; "
            "border-radius: 2px; } "
            "QSlider::handle:horizontal { background: #4c6ef5; width: 12px; "
            "margin: -4px 0; border-radius: 6px; }"
        )
        spin_style = (
            "background: #1e2030; color: #e0e4f0; border: 1px solid #3d4059; "
            "border-radius: 3px; padding: 1px 2px; font-size: 11px;"
        )

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        lbl = QtWidgets.QLabel(label)
        lbl.setFixedWidth(70)
        lbl.setStyleSheet("color: #c0c8e0; font-size: 12px; background: transparent; border: none;")
        row.addWidget(lbl)
        steps = max(1, int(round((hi - lo) / step)))
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, steps)
        slider.setValue(int(round((default - lo) / step)))
        slider.setStyleSheet(slider_style)
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setValue(default)
        spin.setFixedWidth(70)
        spin.setStyleSheet(spin_style)
        updating = {"lock": False}

        def _slider_cb(pos, s=spin, lo_=lo, step_=step, dec_=decimals):
            if updating["lock"]:
                return
            updating["lock"] = True
            s.setValue(round(lo_ + pos * step_, dec_))
            updating["lock"] = False

        def _spin_cb(v, sl=slider, lo_=lo, step_=step):
            if updating["lock"]:
                return
            updating["lock"] = True
            sl.setValue(int(round((v - lo_) / step_)))
            updating["lock"] = False

        slider.valueChanged.connect(_slider_cb)
        spin.valueChanged.connect(_spin_cb)
        row.addWidget(slider, stretch=1)
        row.addWidget(spin)
        layout.addLayout(row)
        return spin

    def _setup_hotkeys(self) -> None:
        context = QtCore.Qt.WidgetWithChildrenShortcut
        # Bind to the viewport so number/letter keys still type in the user-test editor.
        host = self._plotter

        def _shortcut(key, handler):
            # Modifiers belong in the key sequence itself; QShortcut has no
            # setModifiers. Pass QKeySequence(key | modifier) when one is needed.
            shortcut = QtGui.QShortcut(key, host)
            shortcut.setContext(context)
            shortcut.activated.connect(handler)
            return shortcut

        _shortcut(QtCore.Qt.Key_W, self._cycle_display_mode_hotkey)
        _shortcut(QtCore.Qt.Key_M, self._toggle_color_mode_hotkey)
        _shortcut(QtCore.Qt.Key_F, self._on_fit_view)
        _shortcut(QtCore.Qt.Key_Z, self._fly_to_cursor)
        _shortcut(QtCore.Qt.Key_1, lambda: self._set_named_view("front"))
        _shortcut(QtCore.Qt.Key_2, lambda: self._set_named_view("back"))
        _shortcut(QtCore.Qt.Key_3, lambda: self._set_named_view("left"))
        _shortcut(QtCore.Qt.Key_4, lambda: self._set_named_view("right"))
        _shortcut(QtCore.Qt.Key_5, lambda: self._set_named_view("top"))
        _shortcut(QtCore.Qt.Key_6, lambda: self._set_named_view("bottom"))
        _shortcut(QtCore.Qt.Key_I, lambda: self._set_named_view("isometric"))
        _shortcut(QtCore.Qt.Key_7, lambda: self._set_named_view("front-iso"))
        _shortcut(QtCore.Qt.Key_N, self._cycle_normals_hotkey)
        _shortcut(QtCore.Qt.Key_E, self._toggle_edge_vertices_hotkey)
        _shortcut(QtCore.Qt.Key_T, self._cycle_topology_counts_hotkey)
        _shortcut(QtCore.Qt.Key_C, self._cycle_chord_tolerance_hotkey)
        _shortcut(QtCore.Qt.Key_L, self._cycle_max_edge_hotkey)
        _shortcut(QtCore.Qt.Key_R, self._cycle_max_aspect_hotkey)
        _shortcut(QtCore.Qt.Key_Plus, self._increase_angle_hotkey)
        _shortcut(QtCore.Qt.Key_Equal, self._increase_angle_hotkey)
        _shortcut(QtCore.Qt.Key_Minus, self._decrease_angle_hotkey)

    def _cycle_display_mode_hotkey(self) -> None:
        current_index = self._display_mode_combo.currentIndex()
        next_index = (current_index + 1) % self._display_mode_combo.count()
        self._display_mode_combo.setCurrentIndex(next_index)

    def _toggle_color_mode_hotkey(self) -> None:
        cycle = list(COLOR_MODE_CYCLE)
        current = self._display_settings.color_mode
        next_mode = cycle[(cycle.index(current) + 1) % len(cycle)] if current in cycle else COLOR_MODE_BREP_ORIENTATION
        for index in range(self._color_mode_combo.count()):
            if self._color_mode_combo.itemData(index) == next_mode:
                self._color_mode_combo.setCurrentIndex(index)
                return

    def _cycle_normals_hotkey(self) -> None:
        cycle = list(NORMALS_MODE_CYCLE)
        current = self._display_settings.normals_mode
        next_mode = cycle[(cycle.index(current) + 1) % len(cycle)] if current in cycle else NORMALS_MODE_OFF
        for index in range(self._normals_mode_combo.count()):
            if self._normals_mode_combo.itemData(index) == next_mode:
                self._normals_mode_combo.setCurrentIndex(index)
                return

    def _toggle_edge_vertices_hotkey(self) -> None:
        self._edge_vertices_check.setChecked(not self._edge_vertices_check.isChecked())

    def _cycle_topology_counts_hotkey(self) -> None:
        cycle = list(TOPOLOGY_COUNTS_CYCLE)
        current = self._display_settings.topology_counts_mode
        next_mode = cycle[(cycle.index(current) + 1) % len(cycle)] if current in cycle else TOPOLOGY_COUNTS_BREP
        for index in range(self._topology_counts_combo.count()):
            if self._topology_counts_combo.itemData(index) == next_mode:
                self._topology_counts_combo.setCurrentIndex(index)
                return

    def _apply_chord_preset(self, index: int) -> None:
        self._chord_preset_index = index % len(CHORD_TOLERANCE_FRACS)
        value = chord_tolerance_for_index(self._chord_preset_index, self._bbox_diag)
        self._chord_spin.setValue(value)

    def _apply_max_edge_preset(self, index: int) -> None:
        self._max_edge_preset_index = index % len(MAX_EDGE_LENGTH_FRACS)
        value = max_edge_length_for_index(self._max_edge_preset_index, self._bbox_diag)
        self._max_edge_spin.setValue(value)

    def _apply_max_aspect_preset(self, index: int) -> None:
        self._max_aspect_preset_index = index % len(MAX_ASPECT_RATIO_PRESETS)
        self._max_aspect_spin.setValue(max_aspect_ratio_for_index(self._max_aspect_preset_index))

    def _cycle_chord_tolerance_hotkey(self) -> None:
        self._apply_chord_preset(cycle_index(self._chord_preset_index, len(CHORD_TOLERANCE_FRACS)))

    def _cycle_max_edge_hotkey(self) -> None:
        self._apply_max_edge_preset(cycle_index(self._max_edge_preset_index, len(MAX_EDGE_LENGTH_FRACS)))

    def _cycle_max_aspect_hotkey(self) -> None:
        self._apply_max_aspect_preset(cycle_index(self._max_aspect_preset_index, len(MAX_ASPECT_RATIO_PRESETS)))

    def _increase_angle_hotkey(self) -> None:
        self._angle_spin.setValue(max(1.0, self._angle_spin.value() / 2.0))

    def _decrease_angle_hotkey(self) -> None:
        self._angle_spin.setValue(min(60.0, self._angle_spin.value() * 2.0))

    def _on_normals_mode_changed(self) -> None:
        self._display_settings.normals_mode = self._normals_mode_combo.currentData() or NORMALS_MODE_OFF
        if self._objects:
            self._display_meshes(self._pyvista_batches())

    def _on_edge_vertices_changed(self, checked: bool) -> None:
        self._display_settings.show_vertices = bool(checked)
        if self._objects:
            self._display_meshes(self._pyvista_batches())

    def _on_topology_counts_mode_changed(self) -> None:
        self._display_settings.topology_counts_mode = self._topology_counts_combo.currentData() or TOPOLOGY_COUNTS_BREP
        if self._objects:
            meshes = self._pyvista_batches()
            self._update_viewport_hud(meshes)
            self._plotter.render()

    def _fly_to_cursor(self) -> None:
        fly = getattr(self._plotter, "fly_to_mouse_position", None)
        if fly is None:
            self._status_bar.setText("Fly-to is unavailable in this PyVista build.")
            return
        try:
            fly()
            self._status_bar.setText("Flew to cursor position.")
        except Exception as exc:
            self._status_bar.setText(f"Fly-to failed: {exc}")

    def _update_viewport_hud(self, meshes: list[pv.PolyData]) -> None:
        self._hud.update(
            display_mode=self._display_settings.display_mode,
            color_mode=self._display_settings.color_mode,
            tessellation=self._tessellation_settings,
            stats=HudStats(
                rows=topology_count_rows(
                    self._objects,
                    meshes,
                    self._display_settings.topology_counts_mode,
                ),
                topology_counts_mode=self._display_settings.topology_counts_mode,
            ),
            defaults=self._default_tessellation_settings,
            normals_mode=self._display_settings.normals_mode,
            show_vertices=self._display_settings.show_vertices,
            topology_counts_mode=self._display_settings.topology_counts_mode,
        )

    def _apply_styles(self):
        self.setStyleSheet("""
            QMainWindow { background: #181a2a; }
            #leftPanel, #rightPanel, #rightPanelScroll { background: #21243a; }
            #rightPanelScroll { border: none; }
            QLabel { color: #c0c8e0; }
            QGroupBox QLabel { background: transparent; border: none; }
            QDoubleSpinBox { font-size: 12px; }
        """)

    def _on_tess_changed(self):
        self._sync_tessellation_settings()
        if self._objects:
            self._retessellate_and_display()

    def _on_display_mode_changed(self):
        self._display_settings.display_mode = self._display_mode_combo.currentData() or DISPLAY_MODE_SHADED_WIREFRAME
        if self._objects:
            self._display_meshes(self._pyvista_batches())

    def _on_color_mode_changed(self):
        self._display_settings.color_mode = self._color_mode_combo.currentData() or COLOR_MODE_BREP_ORIENTATION
        if self._objects:
            self._display_meshes(self._pyvista_batches())

    def _on_up_axis_changed(self):
        from .settings import UP_AXIS_Z

        self._display_settings.up_axis = self._up_axis_combo.currentData() or UP_AXIS_Z

    def _on_view_front(self):
        self._set_named_view("front")

    def _on_view_back(self):
        self._set_named_view("back")

    def _on_view_left(self):
        self._set_named_view("left")

    def _on_view_top(self):
        self._set_named_view("top")

    def _on_view_bottom(self):
        self._set_named_view("bottom")

    def _on_view_right(self):
        self._set_named_view("right")

    def _on_view_isometric(self):
        self._set_named_view("isometric")

    def _on_view_front_iso(self):
        self._set_named_view("front-iso")

    def _set_named_view(self, view_name: str):
        self._viewport.set_named_view(view_name, up_axis=self._display_settings.up_axis)
        self._status_bar.setText(f"View: {view_name}.")

    def _on_fit_view(self):
        self._viewport.fit_view()
        self._status_bar.setText("View fit to visible objects.")

    def _on_save_view(self):
        self._viewport.save_camera()
        self._status_bar.setText("Saved current view.")

    def _on_restore_view(self):
        if self._viewport.restore_saved_camera():
            self._parallel_projection_check.blockSignals(True)
            self._parallel_projection_check.setChecked(self._viewport.is_parallel_projection())
            self._parallel_projection_check.blockSignals(False)
            self._display_settings.parallel_projection = self._viewport.is_parallel_projection()
            self._status_bar.setText("Restored saved view.")
        else:
            self._status_bar.setText("No saved view.")

    def _on_parallel_projection_changed(self, checked: bool):
        self._display_settings.parallel_projection = bool(checked)
        self._viewport.set_parallel_projection(checked)

    def _selected_test_suite(self) -> str:
        suite = self._test_suite_combo.currentData()
        return str(suite or self._test_suite_combo.currentText())

    def _refresh_test_panel(self, reset_log: bool = True):
        available = test_runner.tests_available()
        self._run_test_btn.setEnabled(available and not self._test_is_running)
        self._test_suite_combo.setEnabled(not self._test_is_running)
        self._run_test_btn.setToolTip(test_runner.DEBUG_GUIDANCE if available else test_runner.unavailable_reason())
        if self._test_is_running or not reset_log:
            return
        if available:
            self._test_log.setPlainText(test_runner.DEBUG_GUIDANCE)
        else:
            self._test_log.setPlainText(test_runner.unavailable_reason())

    def _on_test_suite_changed(self):
        self._refresh_test_panel()

    def _workbench_settings(self) -> QtCore.QSettings:
        return QtCore.QSettings(user_test_catalog.SETTINGS_ORG, user_test_catalog.SETTINGS_APP)

    @staticmethod
    def _settings_bool(value, default: bool = False) -> bool:
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes"}
        if isinstance(value, (int, bool)):
            return bool(value)
        return default

    def _saved_left_panel_collapsed(self) -> bool:
        settings = self._workbench_settings()
        return self._settings_bool(
            settings.value(user_test_catalog.SETTINGS_KEY_USER_TEST_PANEL_COLLAPSED, False),
            False,
        )

    def _persist_left_panel_collapsed(self, collapsed: bool) -> None:
        settings = self._workbench_settings()
        settings.setValue(user_test_catalog.SETTINGS_KEY_USER_TEST_PANEL_COLLAPSED, bool(collapsed))

    def _on_toggle_left_panel(self) -> None:
        self._set_left_panel_collapsed(not self._left_panel_collapsed)

    def _set_left_panel_collapsed(self, collapsed: bool, persist: bool = True) -> None:
        collapsed = bool(collapsed)
        self._left_panel_collapsed = collapsed
        self._left_panel_body.setVisible(not collapsed)
        self._user_test_title.setVisible(not collapsed)
        self._left_panel_spine_label.setVisible(collapsed)
        left_layout = self._left_panel.layout()
        if collapsed:
            self._left_panel.setFixedWidth(LEFT_PANEL_COLLAPSED_WIDTH)
            if left_layout is not None:
                left_layout.setContentsMargins(8, 8, 8, 8)
            self._left_panel_toggle.setText("›")
            self._left_panel_toggle.setToolTip("Expand user tests")
        else:
            self._left_panel.setFixedWidth(LEFT_PANEL_EXPANDED_WIDTH)
            if left_layout is not None:
                left_layout.setContentsMargins(12, 12, 12, 12)
            self._left_panel_toggle.setText("‹")
            self._left_panel_toggle.setToolTip("Collapse user tests")
        if persist:
            self._persist_left_panel_collapsed(collapsed)

    def _saved_user_test_number(self) -> int:
        settings = self._workbench_settings()
        value = settings.value(user_test_catalog.SETTINGS_KEY_USER_TEST_NUMBER, 1)
        try:
            return int(value)
        except (TypeError, ValueError):
            return 1

    def _persist_user_test_number(self, number: int) -> None:
        settings = self._workbench_settings()
        settings.setValue(user_test_catalog.SETTINGS_KEY_USER_TEST_NUMBER, int(number))

    def _current_user_test_number(self) -> int:
        return int(self._user_test_number.value())

    def _reload_user_tests(self, restore_number: bool = False) -> list[user_test_catalog.UserTestCase]:
        current = self._current_user_test_number() if hasattr(self, "_user_test_number") else 1
        if restore_number:
            current = self._saved_user_test_number()
        cases, error = user_test_catalog.load_user_tests(self._user_test_path)
        self._user_test_cases = cases
        self._user_test_error = error
        self._user_test_overrides = {}
        self._rebuild_user_test_combo()
        self._set_user_test_number(current)
        self._user_test_path_label.setText(self._user_test_path)
        if error:
            self._status_bar.setText(error)
        return cases

    def _rebuild_user_test_combo(self) -> None:
        combo = self._user_test_combo
        old_block = combo.blockSignals(True)
        combo.clear()
        for case in self._user_test_cases:
            combo.addItem(case.label, case.number)
        combo.blockSignals(old_block)

    def _set_user_test_number(self, number: int) -> None:
        number = max(0, int(number))
        old_spin = self._user_test_number.blockSignals(True)
        self._user_test_number.setValue(number)
        self._user_test_number.blockSignals(old_spin)
        self._sync_user_test_combo(number)
        self._load_user_test_into_editor(number)

    def _sync_user_test_combo(self, number: int) -> None:
        combo = self._user_test_combo
        old_block = combo.blockSignals(True)
        index = combo.findData(number)
        combo.setCurrentIndex(index)
        combo.blockSignals(old_block)

    def _load_user_test_into_editor(self, number: int) -> None:
        if number in self._user_test_overrides:
            source = self._user_test_overrides[number]
        else:
            case = user_test_catalog.case_by_number(self._user_test_cases, number)
            source = case.source if case is not None else ""
        self._user_test_loading = True
        self._source_edit.setPlainText(source)
        self._user_test_loading = False

    def _on_user_test_combo_changed(self, index: int) -> None:
        if index < 0:
            return
        number = self._user_test_combo.itemData(index)
        if number is None:
            return
        self._set_user_test_number(int(number))

    def _on_user_test_number_changed(self, number: int) -> None:
        self._set_user_test_number(int(number))

    def _on_user_test_source_changed(self) -> None:
        if self._user_test_loading:
            return
        self._user_test_overrides[self._current_user_test_number()] = self._source_edit.toPlainText()

    def _on_reload_user_tests(self) -> None:
        self._reload_user_tests()
        if self._user_test_error:
            return
        self._status_bar.setText(f"Reloaded user tests from {self._user_test_path}.")

    def _on_save_user_test(self) -> bool:
        number = self._current_user_test_number()
        source = self._source_edit.toPlainText()
        case = user_test_catalog.case_by_number(self._user_test_cases, number)
        title = case.title if case is not None else ""
        error = user_test_catalog.save_user_test(
            number,
            source,
            path=self._user_test_path,
            title=title or None,
        )
        if error:
            self._status_bar.setText(error)
            return False
        self._user_test_overrides.pop(number, None)
        self._reload_user_tests()
        self._set_user_test_number(number)
        self._persist_user_test_number(number)
        self._status_bar.setText(f"Saved user test {number} to {self._user_test_path}.")
        return True

    def _on_open_user_test_file(self) -> None:
        path = os.path.abspath(self._user_test_path)
        opened = QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(path))
        if opened:
            self._status_bar.setText(f"Opened {path}.")
        else:
            self._status_bar.setText(f"User tests file: {path}")

    def _on_run_user_test(self) -> bool:
        number = self._current_user_test_number()
        source = self._source_edit.toPlainText()
        self._persist_user_test_number(number)
        if user_test_catalog.script_is_empty(source):
            self._current_source = source
            self._status_bar.setStyleSheet("color: #a0a8c0; font-size: 11px; padding: 4px 0;")
            self._status_bar.setText(f"User test {number} is empty.")
            return True
        if self._execute_and_display(source):
            self._status_bar.setText(f"Played user test {number}.")
            return True
        return False

    def _on_replay_source_history(self) -> None:
        source = self._source_history.current()
        if source is None:
            self._status_bar.setText("No source history to replay.")
            return
        self._load_source_history_snapshot(source)
        if self._execute_and_display(source, record_history=False):
            self._status_bar.setText(f"Replayed source history {self._source_history.position_label}.")

    def _on_previous_source_history(self) -> None:
        source = self._source_history.previous()
        if source is None:
            self._status_bar.setText("Already at oldest source history entry.")
            self._update_source_history_buttons()
            return
        self._load_source_history_snapshot(source)

    def _on_next_source_history(self) -> None:
        source = self._source_history.next()
        if source is None:
            self._status_bar.setText("Already at newest source history entry.")
            self._update_source_history_buttons()
            return
        self._load_source_history_snapshot(source)

    def _record_source_history(self, source: str) -> None:
        self._source_history.add(source)
        self._update_source_history_buttons()

    def _load_source_history_snapshot(self, source: str) -> None:
        self._current_source = source
        self._user_test_overrides[self._current_user_test_number()] = source
        self._user_test_loading = True
        self._source_edit.setPlainText(source)
        self._user_test_loading = False
        self._update_source_history_buttons()
        self._status_bar.setText(f"Loaded source history {self._source_history.position_label}.")

    def _update_source_history_buttons(self) -> None:
        if not hasattr(self, "_replay_source_btn"):
            return
        has_history = self._source_history.current() is not None
        self._replay_source_btn.setEnabled(has_history)
        self._history_prev_btn.setEnabled(self._source_history.can_previous())
        self._history_next_btn.setEnabled(self._source_history.can_next())


    def _on_run_test_suite(self):
        suite = self._selected_test_suite()
        if not suite:
            self._status_bar.setText("Select a prog_test suite before running.")
            return
        if not test_runner.tests_available():
            reason = test_runner.unavailable_reason()
            self._test_log.setPlainText(reason)
            self._status_bar.setText("In-process prog_test runner unavailable.")
            return

        self._test_is_running = True
        self._refresh_test_panel()
        self._test_log.setPlainText(f"Running prog_test suite: {suite}\n\n{test_runner.DEBUG_GUIDANCE}")
        self._status_bar.setText(f"Running prog_test suite: {suite}.")
        QtWidgets.QApplication.processEvents()

        try:
            result = test_runner.run_prog_test_suite(suite)
        except Exception as exc:
            self._test_log.setPlainText(f"prog_test error: {exc}")
            self._status_bar.setText(f"prog_test error: {exc}")
            return None
        finally:
            self._test_is_running = False
            self._refresh_test_panel(reset_log=False)

        draw_count = self._append_prog_test_draw_events(result)
        elapsed = result.get("elapsed_seconds", 0.0)
        outcome = "passed" if result.get("ok") else "failed"
        summary = (
            f"Suite {result.get('suite_name', suite)} {outcome} "
            f"({result.get('status_name', result.get('status'))}) in {elapsed:.3f}s."
        )
        if draw_count:
            summary += f"\nDraw events displayed: {draw_count}."
        log_text = str(result.get("log", "")).strip()
        self._test_log.setPlainText(summary + (f"\n\n{log_text}" if log_text else ""))
        self._status_bar.setText(summary)
        return result

    def _append_prog_test_draw_events(self, result: dict) -> int:
        count = 0
        for name, kind, handle in test_runner.draw_events_to_overlays(result.get("draw_events", ())):
            overlay = make_overlay_object(name, kind, handle, self._next_object_id())
            self._objects.append(overlay)
            self._selected_object_id = overlay.object_id
            count += 1
        if count:
            self._retessellate_and_display()
            self._rebuild_object_list(self._selected_object_id)
        return count

    def _on_load_file(self):
        filename, selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load",
            "",
            LOAD_FILE_FILTER,
        )
        if filename:
            self._load_file(filename, selected_filter)

    def _on_save_file(self):
        filename, selected_filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save",
            "",
            SAVE_FILE_FILTER,
        )
        if filename:
            self._save_file(filename, selected_filter)

    def _load_file(self, filename: str, selected_filter: str = "") -> list[ActiveObject]:
        filename = ensure_extension_for_filter(filename, selected_filter)
        kind = file_kind_for_path(filename)
        if kind == "usd":
            return self._import_usd_file(filename)
        if kind in ("smb", "smp"):
            return self._import_native_brep_file(filename)
        if kind == "occt_brep":
            return self._import_occt_brep_file(filename)
        self._status_bar.setText(f"Load error: unsupported file type for {filename}.")
        return []

    def _save_file(self, filename: str, selected_filter: str = "") -> bool:
        filename = ensure_extension_for_filter(filename, selected_filter)
        kind = file_kind_for_path(filename)
        if kind == "usd":
            return self._export_usd_file(filename) > 0
        if kind == "smb":
            return self._save_selected_native_brep(filename)
        if kind == "occt_brep":
            return self._save_selected_occt_brep(filename)
        self._status_bar.setText(f"Save error: unsupported file type for {filename}.")
        return False

    def _import_usd_file(self, filename: str) -> list[ActiveObject]:
        try:
            objects = import_usd_objects(filename)
        except Exception as exc:
            self._status_bar.setText(f"Import error: {exc}")
            return []
        self._objects = objects
        self._current_source = ""
        self._selected_object_id = objects[0].object_id if objects else None
        self._clear_topology_selection()
        self._retessellate_and_display(reset_camera=True)
        self._rebuild_object_list(self._selected_object_id)
        self._status_bar.setText(f"Imported {len(objects)} object(s) from {filename}.")
        return objects

    def _import_native_brep_file(self, filename: str) -> list[ActiveObject]:
        try:
            objects = import_native_brep_object(filename)
        except Exception as exc:
            self._status_bar.setText(f"Load error: {exc}")
            return []
        self._objects = objects
        self._current_source = ""
        self._selected_object_id = objects[0].object_id if objects else None
        self._clear_topology_selection()
        self._retessellate_and_display(reset_camera=True)
        self._rebuild_object_list(self._selected_object_id)
        self._status_bar.setText(f"Loaded {len(objects)} native BRep object(s) from {filename}.")
        return objects

    def _import_occt_brep_file(self, filename: str) -> list[ActiveObject]:
        try:
            objects = import_occt_brep_objects(filename)
        except Exception as exc:
            self._status_bar.setText(f"Load error: {exc}")
            return []
        self._objects = objects
        self._current_source = ""
        self._selected_object_id = objects[0].object_id if objects else None
        self._clear_topology_selection()
        self._retessellate_and_display(reset_camera=True)
        self._rebuild_object_list(self._selected_object_id)
        self._status_bar.setText(f"Loaded {len(objects)} OpenCASCADE BRep object(s) from {filename}.")
        return objects

    def _export_usd_file(self, filename: str) -> int:
        try:
            count = export_usd_objects(self._objects, filename)
        except Exception as exc:
            self._status_bar.setText(f"Export error: {exc}")
            return 0
        self._status_bar.setText(f"Exported {count} object(s) to {filename}.")
        return count

    def _save_selected_native_brep(self, filename: str) -> bool:
        obj = self._selected_object()
        if obj is None:
            self._status_bar.setText("Select a BRep before saving.")
            return False
        try:
            export_native_brep(obj, filename)
        except Exception as exc:
            self._status_bar.setText(f"Save error: {exc}")
            return False
        self._status_bar.setText(f"Saved {obj.name} to {filename}.")
        return True

    def _save_selected_occt_brep(self, filename: str) -> bool:
        obj = self._selected_object()
        if obj is None:
            self._status_bar.setText("Select a BRep before saving.")
            return False
        try:
            export_occt_brep(obj, filename)
        except Exception as exc:
            self._status_bar.setText(f"Save error: {exc}")
            return False
        self._status_bar.setText(f"Saved {obj.name} to OpenCASCADE BRep {filename}.")
        return True

    def _apply_boolean_operation(self, operation: str) -> ActiveObject | None:
        selected_ids = self._selected_object_ids_from_list()
        try:
            objects, result, source_line = apply_boolean_operation(
                self._objects,
                selected_ids,
                operation,
            )
        except Exception as exc:
            self._status_bar.setText(f"Boolean error: {exc}")
            return None

        self._objects = objects
        self._selected_object_id = result.object_id
        self._clear_topology_selection()
        self._record_operation_source(source_line)
        self._retessellate_and_display()
        self._rebuild_object_list(result.object_id)
        self._status_bar.setText(f"Boolean {operation} created {result.name}.")
        return result

    def _apply_modify_operation(self, operation: str) -> ActiveObject | None:
        obj = self._selected_non_overlay_object()
        if obj is None:
            self._status_bar.setText("Select a model object before running modify actions.")
            return None
        try:
            result = apply_modify_operation(obj, operation)
        except Exception as exc:
            self._status_bar.setText(f"Modify error: {exc}")
            return None

        self._selected_object_id = result.object.object_id
        self._clear_topology_selection()
        if result.source_line:
            self._record_operation_source(result.source_line)
        if result.retessellate:
            self._retessellate_and_display()
        else:
            self._sync_parts_snapshot()
            self._refresh_selected_object_summary()
        self._rebuild_object_list(result.object.object_id)
        if result.details:
            self._object_info.setPlainText(result.details)
        self._status_bar.setText(result.status)
        return result.object

    def _record_operation_source(self, source_line: str) -> None:
        existing = self._current_source.rstrip()
        if existing:
            self._current_source = existing + "\n" + source_line + "\n"
        else:
            self._current_source = source_line + "\n"

    def _next_object_id(self) -> int:
        return max((obj.object_id for obj in self._objects), default=0) + 1

    def _report_draw_status(self, message: str) -> None:
        notice = getattr(self, "_kernel_draw_unavailable_notice", "")
        self._kernel_draw_unavailable_notice = ""
        if notice:
            self._status_bar.setText(f"{message} {notice}")
        else:
            self._status_bar.setText(message)

    def _append_debug_overlay(
        self,
        name: str,
        kind: str,
        handle: object,
        status_message: str | None = None,
    ) -> ActiveObject:
        overlay = make_overlay_object(name, kind, handle, self._next_object_id())
        self._objects.append(overlay)
        self._selected_object_id = overlay.object_id
        self._retessellate_and_display()
        self._rebuild_object_list(overlay.object_id)
        self._report_draw_status(status_message or f"Added {kind.lower()} overlay {name}.")
        return overlay

    def _add_debug_point(self, name: str, point) -> ActiveObject:
        return self._append_debug_overlay(name, "Point", PointOverlay(tuple(point)))

    def _add_debug_line(self, name: str, start, end) -> ActiveObject:
        return self._append_debug_overlay(name, "Line", LineOverlay(tuple(start), tuple(end)))

    def _add_debug_polyline(self, name: str, points) -> ActiveObject:
        return self._append_debug_overlay(
            name,
            "Polyline",
            PolylineOverlay([tuple(point) for point in points]),
        )

    def _add_debug_box(self, name: str, minimum, maximum) -> ActiveObject:
        return self._append_debug_overlay(name, "Box", BoxOverlay(tuple(minimum), tuple(maximum)))

    def _add_debug_curve(self, name: str, curve: CurveSampleOverlay) -> ActiveObject:
        return self._append_debug_overlay(name, "CurveSample", curve)

    def _add_debug_surface(self, name: str, surface: SurfaceSampleOverlay) -> ActiveObject:
        return self._append_debug_overlay(name, "SurfaceSample", surface)

    def _add_debug_topology(
        self,
        name: str,
        topology: TopologyOverlay,
        status_message: str | None = None,
    ) -> ActiveObject:
        return self._append_debug_overlay(name, "Topology", topology, status_message=status_message)

    def _add_debug_display_list(
        self,
        name: str,
        display_list: DrawBatchOverlay,
        status_message: str | None = None,
    ) -> ActiveObject:
        return self._append_debug_overlay(name, "DisplayList", display_list, status_message=status_message)

    def _add_topology_draw_overlay(self, name: str, overlay, status_message: str | None = None) -> ActiveObject:
        if isinstance(overlay, DrawBatchOverlay):
            return self._add_debug_display_list(name, overlay, status_message=status_message)
        return self._add_debug_topology(name, overlay, status_message=status_message)

    def _add_debug_section(self, name: str, section: SectionOverlay) -> ActiveObject:
        return self._append_debug_overlay(name, "Section", section)

    def _set_topology_selection(self, selection: TopologyPick | None) -> None:
        new_selection_key = self._topology_selection_key(selection)
        if new_selection_key != self._topology_draw_cycle_key:
            self._remove_topology_transient_overlays(refresh=True)
        self._topology_selection = selection
        self._topology_draw_cycle_key = new_selection_key
        self._topology_draw_cycle_indices = {}
        if selection is not None and hasattr(self, "_object_list"):
            for row in range(self._object_list.count()):
                item = self._object_list.item(row)
                if item is not None and item.data(QtCore.Qt.UserRole) == selection.object_id:
                    self._object_list.setCurrentRow(row)
                    break
        self._update_topology_selection_panel()
        self._render_topology_selection_marker()

    def _clear_topology_selection(self) -> None:
        self._topology_selection = None
        self._topology_pick_last_screen = None
        self._topology_pick_cycle_index = 0
        self._topology_draw_cycle_key = None
        self._topology_draw_cycle_indices = {}
        self._remove_topology_transient_overlays(refresh=True)
        self._update_topology_selection_panel()
        if hasattr(self, "_viewport"):
            self._viewport.clear_topology_pick_marker()

    def _render_topology_selection_marker(self) -> None:
        if not hasattr(self, "_viewport"):
            return
        if self._topology_selection is None:
            self._viewport.clear_topology_pick_marker(render=False)
            return
        self._viewport.render_topology_pick_marker(self._topology_selection.hit_point)

    def _update_topology_selection_panel(self) -> None:
        if not hasattr(self, "_topology_summary"):
            return
        selection = self._topology_selection
        buttons = getattr(self, "_topology_action_buttons", {})
        for btn in buttons.values():
            btn.setEnabled(False)
        if selection is None:
            self._topology_summary.setText("No topology selected.")
            return

        self._topology_summary.setText(
            f"{selection.topology_kind} {selection.topology_index} on {selection.object_name}\n"
            f"Depth {selection.ray_depth:.4g}"
            + (
                f"\nEdgeuses {len(selection.edgeuses)} | Loopuses {len(selection.loopuses)}"
                if selection.topology_kind == "Edge"
                else ""
            )
            + (
                f"\nLoopuses {len(self._loopuse_candidates_for_selection(selection))}"
                if selection.topology_kind == "Face"
                else ""
            )
            + (
                f"\nEdges {len(self._edge_candidates_for_selection(selection))} | "
                f"Edgeuses {len(self._edgeuse_candidates_for_selection(selection))}"
                if selection.topology_kind == "Vertex"
                else ""
            )
        )

        def _enable(key: str, enabled: bool) -> None:
            button = buttons.get(key)
            if button is not None:
                button.setEnabled(enabled)

        kind = selection.topology_kind
        if kind == "Face":
            _enable("face", selection.face is not None)
            _enable("surface", selection.face is not None)
            _enable("loopuse", bool(self._loopuse_candidates_for_selection(selection)))
        elif kind == "Edge":
            _enable("edge", selection.edge is not None)
            _enable("curve", selection.edge is not None)
            _enable("edgeuse", bool(self._edgeuse_candidates_for_selection(selection)))
            _enable("loopuse", bool(self._loopuse_candidates_for_selection(selection)))
        elif kind == "Vertex":
            _enable("edge", bool(self._edge_candidates_for_selection(selection)))
            _enable("edgeuse", bool(self._edgeuse_candidates_for_selection(selection)))
        _enable("assert_valid", smdev is not None and selection.handle is not None)
        _enable("dump", smdev is not None and selection.handle is not None)

    @staticmethod
    def _topology_selection_key(selection: TopologyPick | None) -> tuple[object, ...] | None:
        if selection is None:
            return None
        return (
            selection.object_id,
            selection.topology_kind,
            selection.topology_index,
            selection.solver_hit_index,
        )

    @staticmethod
    def _topology_handle_index(handles, handle) -> int:
        for index, candidate in enumerate(handles):
            if candidate is handle:
                return index
        return -1

    @staticmethod
    def _append_unique_topology_candidate(handles: list[object], handle) -> None:
        if handle is None:
            return
        if not any(candidate is handle for candidate in handles):
            handles.append(handle)

    def _edge_candidates_for_selection(self, selection: TopologyPick | None) -> tuple[object, ...]:
        if selection is None:
            return ()
        edges: list[object] = []
        if selection.topology_kind == "Vertex" and selection.vertex is not None:
            try:
                for edge in selection.vertex.edges():
                    self._append_unique_topology_candidate(edges, edge)
            except Exception:
                pass
        self._append_unique_topology_candidate(edges, selection.edge)
        return tuple(edges)

    def _edgeuse_candidates_for_selection(self, selection: TopologyPick | None) -> tuple[object, ...]:
        if selection is None:
            return ()
        edgeuses: list[object] = []
        if selection.topology_kind == "Vertex":
            for edge in self._edge_candidates_for_selection(selection):
                try:
                    if smdev is not None:
                        for edgeuse in smdev.edgeuses(edge):
                            self._append_unique_topology_candidate(edgeuses, edgeuse)
                except Exception:
                    if selection.face is not None and smdev is not None:
                        try:
                            self._append_unique_topology_candidate(
                                edgeuses, smdev.edgeuse_of_face(edge, selection.face)
                            )
                        except Exception:
                            pass
        for edgeuse in selection.edgeuses:
            self._append_unique_topology_candidate(edgeuses, edgeuse)
        self._append_unique_topology_candidate(edgeuses, selection.edgeuse)
        return tuple(edgeuses)

    def _loopuse_candidates_for_selection(self, selection: TopologyPick | None) -> tuple[object, ...]:
        if selection is None:
            return ()
        loopuses: list[object] = []
        if selection.topology_kind == "Face" and selection.face is not None and smdev is not None:
            try:
                loops = smdev.face_loops(selection.face)
            except Exception:
                loops = ()
            for loop in loops:
                try:
                    self._append_unique_topology_candidate(loopuses, loop.loopuse())
                except Exception:
                    pass
                try:
                    for loopuse in loop.loopuses():
                        try:
                            if loopuse.face() is not selection.face:
                                continue
                        except Exception:
                            pass
                        self._append_unique_topology_candidate(loopuses, loopuse)
                except Exception:
                    pass
        for loopuse in selection.loopuses:
            self._append_unique_topology_candidate(loopuses, loopuse)
        self._append_unique_topology_candidate(loopuses, selection.loopuse)
        return tuple(loopuses)

    def _candidate_edge_index(self, obj: ActiveObject, candidate) -> int:
        edge = candidate
        if candidate is not None and hasattr(candidate, "edge"):
            try:
                edge = candidate.edge()
            except Exception:
                edge = None
        if edge is None:
            return -1
        try:
            return self._topology_handle_index(obj.handle.edges(), edge)
        except Exception:
            return -1

    @staticmethod
    def _topology_draw_source_label(selection: TopologyPick) -> str:
        return f"{selection.topology_kind.lower()}{selection.topology_index}"

    def _next_topology_draw_candidate(self, key: str, candidates: tuple[object, ...]):
        selection_key = self._topology_selection_key(self._topology_selection)
        if selection_key != self._topology_draw_cycle_key:
            self._topology_draw_cycle_key = selection_key
            self._topology_draw_cycle_indices = {}
        if not candidates:
            return None, -1, 0
        index = self._topology_draw_cycle_indices.get(key, 0) % len(candidates)
        self._topology_draw_cycle_indices[key] = index + 1
        return candidates[index], index, len(candidates)

    def _candidate_face_index(self, obj: ActiveObject, candidate) -> int:
        face = None
        if candidate is not None and hasattr(candidate, "face"):
            try:
                face = candidate.face()
            except Exception:
                face = None
        if face is None:
            return -1
        try:
            return self._topology_handle_index(obj.handle.faces(), face)
        except Exception:
            return -1

    def _candidate_loop_index(self, candidate) -> int:
        loop = None
        face = None
        if candidate is not None and hasattr(candidate, "loop"):
            try:
                loop = candidate.loop()
            except Exception:
                loop = None
        if loop is None:
            loop = candidate
        if loop is not None and hasattr(loop, "face"):
            try:
                face = loop.face()
            except Exception:
                face = None
        if face is None or loop is None or smdev is None:
            return -1
        try:
            return self._topology_handle_index(smdev.face_loops(face), loop)
        except Exception:
            return -1

    def _remove_topology_transient_overlays(self, keys: set[str] | None = None, refresh: bool = True) -> int:
        if not self._topology_transient_overlay_ids:
            return 0
        remove_keys = set(self._topology_transient_overlay_ids) if keys is None else set(keys)
        remove_ids = {
            object_id for key, object_id in self._topology_transient_overlay_ids.items() if key in remove_keys
        }
        self._topology_transient_overlay_ids = {
            key: object_id for key, object_id in self._topology_transient_overlay_ids.items() if key not in remove_keys
        }
        if not remove_ids:
            return 0

        before_count = len(self._objects)
        self._objects = [obj for obj in self._objects if obj.object_id not in remove_ids]
        removed = before_count - len(self._objects)
        if removed <= 0:
            return 0

        if self._selected_object_id in remove_ids:
            preferred_id = self._topology_selection.object_id if self._topology_selection is not None else None
            if preferred_id is None or self._object_by_id(preferred_id) is None:
                preferred_id = next(
                    (obj.object_id for obj in self._objects if not is_overlay(obj)),
                    self._objects[0].object_id if self._objects else None,
                )
            self._selected_object_id = preferred_id
        if refresh:
            self._retessellate_and_display()
            self._rebuild_object_list(self._selected_object_id)
        return removed

    def _add_transient_topology_overlay(
        self,
        key: str,
        name: str,
        topology,
        status_message: str | None = None,
    ) -> ActiveObject:
        self._remove_topology_transient_overlays({key}, refresh=False)
        overlay = self._add_topology_draw_overlay(name, topology, status_message=status_message)
        self._topology_transient_overlay_ids[key] = overlay.object_id
        return overlay

    @staticmethod
    def _merge_overlay_metadata(overlay, metadata: dict[str, object] | None):
        if metadata and hasattr(overlay, "metadata"):
            overlay.metadata.update(metadata)
        return overlay

    def _kernel_topology_overlay(
        self,
        obj: ActiveObject,
        handle,
        topology_kind: str,
        metadata: dict[str, object] | None = None,
        variant: str = "default",
    ) -> DrawBatchOverlay | None:
        try:
            return kernel_draw_overlay(obj, handle, topology_kind, metadata, variant=variant)
        except RuntimeError as exc:
            if "SM_GFX_OUTPUT_CODE" in str(exc):
                self._kernel_draw_unavailable_notice = str(exc)
            return None
        except Exception:
            return None

    def _picked_brep_object(self) -> ActiveObject | None:
        selection = self._topology_selection
        if selection is None:
            return None
        obj = self._object_by_id(selection.object_id)
        if obj is None or obj.kind != "Brep":
            return None
        return obj

    def _select_topology_from_ray(
        self, ray_point, ray_direction, screen_pos: tuple[float, float] | None = None
    ) -> TopologyPick | None:
        hits = topology_hits_for_ray(self._visible_objects(), ray_point, ray_direction)
        if not hits:
            self._clear_topology_selection()
            self._status_bar.setText("No topology hit.")
            return None

        same_screen_pick = False
        if screen_pos is not None and self._topology_pick_last_screen is not None:
            dx = float(screen_pos[0]) - self._topology_pick_last_screen[0]
            dy = float(screen_pos[1]) - self._topology_pick_last_screen[1]
            same_screen_pick = (dx * dx + dy * dy) ** 0.5 <= self._topology_pick_pixel_tolerance
        self._topology_pick_cycle_index = self._topology_pick_cycle_index + 1 if same_screen_pick else 0
        if screen_pos is not None:
            self._topology_pick_last_screen = (float(screen_pos[0]), float(screen_pos[1]))

        selection = hits[self._topology_pick_cycle_index % len(hits)]
        self._set_topology_selection(selection)
        self._status_bar.setText(
            f"Picked {selection.topology_kind} {selection.topology_index} on "
            f"{selection.object_name} ({self._topology_pick_cycle_index % len(hits) + 1}/{len(hits)})."
        )
        return selection

    def _pick_topology_at_screen(self, x: float, y: float) -> TopologyPick | None:
        try:
            ray_point, ray_direction = self._viewport.screen_to_world_ray(x, y)
        except Exception as exc:
            self._status_bar.setText(f"Topology pick error: {exc}")
            return None
        return self._select_topology_from_ray(ray_point, ray_direction, screen_pos=(x, y))

    def _add_picked_face_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        if obj is None or selection is None or selection.topology_kind != "Face" or selection.face is None:
            self._status_bar.setText("Right-click a face before drawing it.")
            return None
        metadata = {"face_index": selection.face_index}
        overlay = self._kernel_topology_overlay(obj, selection.face, "face", metadata)
        if overlay is None:
            try:
                overlay = self._merge_overlay_metadata(
                    brep_face_highlight_from_face(obj, selection.face, selection.face_index),
                    metadata,
                )
            except Exception as exc:
                self._status_bar.setText(f"Face highlight error: {exc}")
                return None
        return self._add_topology_draw_overlay(f"{obj.name}_face{selection.face_index}", overlay)

    def _add_picked_surface_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        if obj is None or selection is None or selection.topology_kind != "Face" or selection.face is None:
            self._status_bar.setText("Right-click a face before drawing its surface.")
            return None
        metadata = {"selection_source": "face", "face_index": selection.face_index}
        overlay = self._kernel_topology_overlay(
            obj,
            selection.face.surface(),
            "surface",
            metadata,
            variant="draw_uv",
        )
        if overlay is not None:
            return self._add_debug_display_list(f"{obj.name}_face{selection.face_index}_surface_uv", overlay)
        try:
            fallback = self._merge_overlay_metadata(
                brep_face_surface_overlay(obj, selection.face, selection.face_index),
                metadata,
            )
        except Exception as exc:
            self._status_bar.setText(f"Surface overlay error: {exc}")
            return None
        return self._add_debug_surface(f"{obj.name}_face{selection.face_index}_surface", fallback)

    def _add_picked_edge_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        if obj is None or selection is None:
            self._status_bar.setText("Right-click an edge or vertex before drawing an attached edge.")
            return None
        candidates = self._edge_candidates_for_selection(selection)
        edge, candidate_index, candidate_count = self._next_topology_draw_candidate("edge", candidates)
        if edge is None or selection.topology_kind not in {"Edge", "Vertex"}:
            self._status_bar.setText("Right-click an edge or vertex before drawing an attached edge.")
            return None
        edge_index = self._candidate_edge_index(obj, edge)
        if edge_index < 0:
            edge_index = selection.edge_index
        metadata = {
            "edge_index": edge_index,
            "candidate_index": candidate_index,
            "candidate_count": candidate_count,
            "edge_cycle_index": candidate_index,
            "selection_kind": selection.topology_kind,
            "selection_index": selection.topology_index,
        }
        overlay = self._kernel_topology_overlay(obj, edge, "edge", metadata)
        if overlay is None:
            try:
                overlay = self._merge_overlay_metadata(
                    brep_edge_highlight_from_edge(obj, edge, edge_index),
                    metadata,
                )
            except Exception as exc:
                self._status_bar.setText(f"Edge highlight error: {exc}")
                return None
        name = f"{obj.name}_{self._topology_draw_source_label(selection)}_edge{edge_index}"
        if selection.topology_kind == "Vertex":
            return self._add_transient_topology_overlay(
                "edge",
                name,
                overlay,
                status_message=(
                    f"Added edge {candidate_index + 1}/{candidate_count} for vertex {selection.vertex_index}."
                ),
            )
        return self._add_topology_draw_overlay(name, overlay)

    def _add_picked_curve_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        if obj is None or selection is None or selection.topology_kind != "Edge" or selection.edge is None:
            self._status_bar.setText("Right-click an edge before drawing its curve.")
            return None
        try:
            curve = selection.edge.curve()
        except Exception as exc:
            self._status_bar.setText(f"Curve lookup error: {exc}")
            return None
        metadata = {
            "curve_source": "edge",
            "edge_index": selection.edge_index,
        }
        overlay = self._kernel_topology_overlay(obj, curve, "curve", metadata)
        name = f"{obj.name}_edge{selection.edge_index}_curve"
        if overlay is not None:
            return self._add_debug_display_list(name, overlay)
        try:
            fallback = sample_curve_overlay(
                curve,
                metadata={
                    "object_id": obj.object_id,
                    "object_name": obj.name,
                    "topology_kind": "curve",
                    "source": "edge_curve",
                    **metadata,
                },
            )
        except Exception as exc:
            self._status_bar.setText(f"Curve overlay error: {exc}")
            return None
        return self._add_debug_curve(name, fallback)

    def _add_picked_edgeuse_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        candidates = self._edgeuse_candidates_for_selection(selection)
        edgeuse, candidate_index, candidate_count = self._next_topology_draw_candidate("edgeuse", candidates)
        if obj is None or selection is None or selection.topology_kind not in {"Edge", "Vertex"} or edgeuse is None:
            self._status_bar.setText("Right-click an edge or vertex before drawing an attached edgeuse.")
            return None
        face_index = self._candidate_face_index(obj, edgeuse)
        if face_index < 0:
            face_index = selection.face_index
        edge_index = self._candidate_edge_index(obj, edgeuse)
        if edge_index < 0:
            edge_index = selection.edge_index
        metadata = {
            "edge_index": edge_index,
            "face_index": face_index,
            "candidate_index": candidate_index,
            "candidate_count": candidate_count,
            "edgeuse_cycle_index": candidate_index,
            "selection_kind": selection.topology_kind,
            "selection_index": selection.topology_index,
            "orientation": edgeuse.orientation() if hasattr(edgeuse, "orientation") else "",
        }
        try:
            metadata["direction_start"] = tuple(edgeuse.point_at_normalized(0.0))
            metadata["direction_end"] = tuple(edgeuse.point_at_normalized(1.0))
        except Exception:
            pass
        overlay = self._kernel_topology_overlay(obj, edgeuse, "edgeuse", metadata)
        if overlay is None:
            try:
                overlay = self._merge_overlay_metadata(
                    brep_edgeuse_highlight(
                        obj,
                        edgeuse,
                        edge_index=edge_index,
                        face_index=face_index,
                        candidate_index=candidate_index,
                        candidate_count=candidate_count,
                    ),
                    metadata,
                )
            except Exception as exc:
                self._status_bar.setText(f"Edgeuse highlight error: {exc}")
                return None
        return self._add_transient_topology_overlay(
            "edgeuse",
            f"{obj.name}_{self._topology_draw_source_label(selection)}_edgeuse{candidate_index}",
            overlay,
            status_message=(
                f"Added edgeuse {candidate_index + 1}/{candidate_count} for "
                f"{selection.topology_kind.lower()} {selection.topology_index}."
            ),
        )

    def _add_picked_loopuse_overlay(self) -> ActiveObject | None:
        selection = self._topology_selection
        obj = self._picked_brep_object()
        candidates = self._loopuse_candidates_for_selection(selection)
        loopuse, candidate_index, candidate_count = self._next_topology_draw_candidate("loopuse", candidates)
        if obj is None or selection is None or selection.topology_kind not in {"Edge", "Face"} or loopuse is None:
            self._status_bar.setText("Right-click an edge or face before drawing an attached loopuse.")
            return None
        face_index = self._candidate_face_index(obj, loopuse)
        if face_index < 0:
            face_index = selection.face_index
        loop_index = self._candidate_loop_index(loopuse)
        if loop_index < 0:
            loop_index = selection.loop_index
        metadata = {
            "loop_index": loop_index,
            "face_index": face_index,
            "candidate_index": candidate_index,
            "candidate_count": candidate_count,
            "loopuse_cycle_index": candidate_index,
            "selection_kind": selection.topology_kind,
            "selection_index": selection.topology_index,
            "edgeuse_count": len(loopuse.edgeuses()) if hasattr(loopuse, "edgeuses") else 0,
            "orientation": loopuse.orientation() if hasattr(loopuse, "orientation") else "",
        }
        overlay = self._kernel_topology_overlay(obj, loopuse, "loopuse", metadata)
        if overlay is None:
            try:
                overlay = self._merge_overlay_metadata(
                    brep_loopuse_highlight(
                        obj,
                        loopuse,
                        loop_index=loop_index,
                        face_index=face_index,
                        candidate_index=candidate_index,
                        candidate_count=candidate_count,
                    ),
                    metadata,
                )
            except Exception as exc:
                self._status_bar.setText(f"Loopuse highlight error: {exc}")
                return None
        return self._add_transient_topology_overlay(
            "loopuse",
            f"{obj.name}_{self._topology_draw_source_label(selection)}_loopuse{candidate_index}",
            overlay,
            status_message=(
                f"Added loopuse {candidate_index + 1}/{candidate_count} for "
                f"{selection.topology_kind.lower()} {selection.topology_index}."
            ),
        )

    def _add_selected_box_overlay(self) -> ActiveObject | None:
        obj = self._selected_non_overlay_object()
        if obj is None:
            self._status_bar.setText("Select a model object before adding a box overlay.")
            return None
        try:
            mn, mx = object_bounds(obj)
        except Exception as exc:
            self._status_bar.setText(f"Debug overlay error: {exc}")
            return None
        return self._add_debug_box(f"{obj.name}_bounds", mn, mx)

    def _add_selected_center_overlay(self) -> ActiveObject | None:
        obj = self._selected_non_overlay_object()
        if obj is None:
            self._status_bar.setText("Select a model object before adding a center overlay.")
            return None
        try:
            center = object_center(obj)
        except Exception as exc:
            self._status_bar.setText(f"Debug overlay error: {exc}")
            return None
        return self._add_debug_point(f"{obj.name}_center", center)

    def _add_selected_section_overlay(self) -> ActiveObject | None:
        obj = self._selected_non_overlay_object()
        if obj is None:
            self._status_bar.setText("Select a displayed model object before adding a section overlay.")
            return None
        try:
            overlay = section_overlay(obj)
        except Exception as exc:
            self._status_bar.setText(f"Section overlay error: {exc}")
            return None
        return self._add_debug_section(f"{obj.name}_section", overlay)

    def _clear_debug_overlays(self) -> int:
        removed = sum(1 for obj in self._objects if is_overlay(obj))
        self._objects = [obj for obj in self._objects if not is_overlay(obj)]
        self._topology_transient_overlay_ids = {}
        if self._selected_object_id is not None:
            selected = self._object_by_id(self._selected_object_id)
            if selected is None or selected.kind in OVERLAY_KINDS:
                self._selected_object_id = self._objects[0].object_id if self._objects else None
        self._retessellate_and_display()
        self._rebuild_object_list(self._selected_object_id)
        self._status_bar.setText(f"Removed {removed} debug overlay(s).")
        return removed

    def _execute_and_display(self, source: str, record_history: bool = True) -> bool:
        self._current_source = source
        self._status_bar.setText("Executing...")
        QtWidgets.QApplication.processEvents()

        objects, err = execute_script(source)
        if err:
            self._status_bar.setText(f"Error: {err[:200]}")
            self._status_bar.setStyleSheet("color: #ff6b6b; font-size: 11px; padding: 4px 0;")
            self._update_source_history_buttons()
            return False

        self._status_bar.setStyleSheet("color: #a0a8c0; font-size: 11px; padding: 4px 0;")
        self._objects = objects
        self._selected_object_id = objects[0].object_id if objects else None
        self._clear_topology_selection()

        self._retessellate_and_display(reset_camera=True)
        self._rebuild_object_list(self._selected_object_id)
        if record_history:
            self._record_source_history(source)
        else:
            self._update_source_history_buttons()
        return True

    def _visible_objects(self) -> list[ActiveObject]:
        return [obj for obj in self._objects if obj.visible]

    def _sync_parts_snapshot(self) -> None:
        self._parts = legacy_parts_from_objects(self._objects)

    def _sync_tessellation_settings(self) -> None:
        self._tessellation_settings = TessellationSettings(
            chord_height_tolerance=self._chord_spin.value(),
            angle_tolerance_deg=self._angle_spin.value(),
            max_edge_length=self._max_edge_spin.value(),
            max_aspect_ratio=self._max_aspect_spin.value(),
        )

    def _retessellate_and_display(self, reset_camera: bool = False):
        self._sync_tessellation_settings()

        try:
            tessellate_objects(self._objects, settings=self._tessellation_settings)
        except Exception as e:
            self._status_bar.setText(f"Tessellation error: {e}")
            self._sync_parts_snapshot()
            self._refresh_selected_object_summary()
            return

        meshes = self._pyvista_batches()
        self._display_meshes(meshes, reset_camera=reset_camera)
        self._sync_parts_snapshot()

        total_pts = sum(m.n_points for m in meshes)
        total_faces = sum(m.n_cells for m in meshes)
        displayable_count = sum(1 for obj in self._objects if obj.mesh)
        status = (
            f"{len(self._objects)} object(s), {displayable_count} displayed, "
            f"{total_pts:,} verts, {total_faces:,} faces"
        )
        no_mesh = mesh_error_summary(self._objects)
        if no_mesh:
            status += f"; no mesh: {no_mesh}"
        self._status_bar.setText(status)
        names = ", ".join(obj.name for obj in self._objects[:5])
        if len(self._objects) > 5:
            names += f", +{len(self._objects) - 5}"
        self._info_label.setText(f"Objects: {names}" if names else "")
        self._refresh_selected_object_summary()

    def _pyvista_batches(self) -> list[pv.PolyData]:
        return objects_to_pyvista_batches(
            self._visible_objects(),
            orient_to_face_normals=color_mode_orients_to_brep_normals(self._display_settings.color_mode),
        )

    def _display_meshes(self, meshes: list[pv.PolyData], reset_camera: bool | None = None) -> int:
        if reset_camera is None:
            reset_camera = self._first_display
        self._bbox_diag = bbox_diagonal_from_meshes(meshes)
        self._viewport.render_meshes(
            meshes,
            self._display_settings.display_mode,
            color_mode=self._display_settings.color_mode,
            reset_camera=reset_camera,
            render=False,
        )
        if self._display_settings.display_mode in DISPLAY_MODES_WITH_BREP_WIREFRAME:
            self._viewport.render_brep_edges(
                self._visible_objects(),
                shaded=self._display_settings.display_mode in DISPLAY_MODES_WITH_SHADED_SURFACE,
                render=False,
            )
        self._viewport.render_overlays(self._visible_objects(), render=False)
        self._viewport.render_inspection_overlays(
            meshes,
            self._visible_objects(),
            normals_mode=self._display_settings.normals_mode,
            show_vertices=self._display_settings.show_vertices,
            bbox_diag=self._bbox_diag,
            render=False,
        )
        self._render_topology_selection_marker()
        self._update_viewport_hud(meshes)
        if self._topology_selection is None:
            self._plotter.render()
        self._first_display = False
        return 0

    def _object_by_id(self, object_id: int | None) -> ActiveObject | None:
        for obj in self._objects:
            if obj.object_id == object_id:
                return obj
        return None

    def _selected_object(self) -> ActiveObject | None:
        return self._object_by_id(self._selected_object_id)

    def _selected_non_overlay_object(self) -> ActiveObject | None:
        obj = self._selected_object()
        if obj is not None and not is_overlay(obj):
            return obj
        for entry in self._objects:
            if not is_overlay(entry):
                return entry
        return None

    def _selected_object_ids_from_list(self) -> list[int]:
        if not hasattr(self, "_object_list"):
            return [self._selected_object_id] if self._selected_object_id is not None else []
        ids = []
        for item in self._object_list.selectedItems():
            object_id = item.data(QtCore.Qt.UserRole)
            if object_id is not None:
                ids.append(object_id)
        if not ids and self._selected_object_id is not None:
            ids.append(self._selected_object_id)
        return ids

    def _rebuild_object_list(self, preferred_id: int | None = None):
        target_id = preferred_id if preferred_id is not None else self._selected_object_id
        old_block = self._object_list.blockSignals(True)
        self._object_list.clear()
        selected_row = -1
        for row, obj in enumerate(self._objects):
            item = QtWidgets.QListWidgetItem(obj.label)
            item.setData(QtCore.Qt.UserRole, obj.object_id)
            item.setToolTip(object_summary_text(obj))
            self._object_list.addItem(item)
            if obj.object_id == target_id:
                selected_row = row
        self._object_list.blockSignals(old_block)

        if selected_row < 0 and self._objects:
            selected_row = 0
        if selected_row >= 0:
            self._object_list.setCurrentRow(selected_row)
        else:
            self._set_selected_object_id(None)

    def _set_selected_object_id(self, object_id: int | None) -> None:
        self._selected_object_id = object_id
        selected_ids = set(self._selected_object_ids_from_list())
        if not selected_ids and object_id is not None:
            selected_ids = {object_id}
        for obj in self._objects:
            obj.selected = obj.object_id in selected_ids
        self._refresh_selected_object_summary()
        has_selection = self._selected_object() is not None
        self._delete_object_btn.setEnabled(has_selection)
        self._dump_object_btn.setEnabled(object_can_dump(self._selected_object()))
        self._assert_valid_object_btn.setEnabled(object_can_assert_valid(self._selected_object()))

    def _refresh_selected_object_summary(self) -> None:
        if not hasattr(self, "_object_info"):
            return
        obj = self._selected_object()
        self._object_info.setPlainText(object_summary_text(obj) if obj else "")

    def _on_object_selection_changed(self, row: int):
        item = self._object_list.item(row)
        object_id = item.data(QtCore.Qt.UserRole) if item is not None else None
        self._set_selected_object_id(object_id)

    def _on_delete_object(self):
        obj = self._selected_object()
        if obj is None:
            return
        delete_id = obj.object_id
        self._objects = [entry for entry in self._objects if entry.object_id != delete_id]
        self._topology_transient_overlay_ids = {
            key: object_id for key, object_id in self._topology_transient_overlay_ids.items() if object_id != delete_id
        }
        if self._topology_selection is not None and self._topology_selection.object_id == delete_id:
            self._clear_topology_selection()
        self._selected_object_id = self._objects[0].object_id if self._objects else None
        self._retessellate_and_display()
        self._rebuild_object_list(self._selected_object_id)
        self._status_bar.setText(f"Deleted {obj.name}.")

    def _on_dump_object(self):
        obj = self._selected_object()
        if obj is None or not object_can_dump(obj):
            self._status_bar.setText("Select a BRep, Curve, or Surface before Dump.")
            return None
        try:
            result = run_dump(obj.handle, label=obj.name, source_expr=obj.name)
        except Exception as exc:
            self._status_bar.setText(f"Dump error: {exc}")
            return None
        self._show_kernel_action_result(result)
        return result

    def _on_dump_topology(self):
        selection = self._topology_selection
        if selection is None or selection.handle is None:
            self._status_bar.setText("Select topology before Dump.")
            return None
        label = f"{selection.topology_kind} {selection.topology_index} on {selection.object_name}"
        try:
            result = run_dump(
                selection.handle,
                label=label,
                source_expr=topology_source_expr(selection),
            )
        except Exception as exc:
            self._status_bar.setText(f"Dump error: {exc}")
            return None
        self._show_kernel_action_result(result)
        return result

    def _show_kernel_action_result(self, result) -> None:
        if result.source_line:
            self._record_operation_source(result.source_line)
        self._object_info.setPlainText(result.details)
        print(result.details)
        self._status_bar.setText(result.status)

    def _show_assert_valid_result(self, result) -> None:
        self._show_kernel_action_result(result)

    def _on_assert_valid_object(self):
        obj = self._selected_object()
        if obj is None or not object_can_assert_valid(obj):
            self._status_bar.setText("Select a BRep, Curve, or Surface before AssertValid.")
            return None
        try:
            result = run_assert_valid(
                obj.handle,
                label=obj.name,
                source_expr=obj.name,
            )
        except Exception as exc:
            self._status_bar.setText(f"AssertValid error: {exc}")
            return None
        self._show_assert_valid_result(result)
        return result

    def _on_assert_valid_topology(self):
        selection = self._topology_selection
        if selection is None or selection.handle is None:
            self._status_bar.setText("Select topology before AssertValid.")
            return None
        label = f"{selection.topology_kind} {selection.topology_index} on {selection.object_name}"
        try:
            result = run_assert_valid(
                selection.handle,
                label=label,
                source_expr=topology_source_expr(selection),
            )
        except Exception as exc:
            self._status_bar.setText(f"AssertValid error: {exc}")
            return None
        self._show_assert_valid_result(result)
        return result

    def eventFilter(self, watched, event):
        if watched is self._plotter and event.type() == QtCore.QEvent.MouseButtonPress:
            if event.button() == QtCore.Qt.RightButton:
                pos = event.position() if hasattr(event, "position") else event.pos()
                self._pick_topology_at_screen(float(pos.x()), float(pos.y()))
                event.accept()
                return True
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        if hasattr(self, "_user_test_number"):
            self._persist_user_test_number(self._current_user_test_number())
        self._plotter.close()
        super().closeEvent(event)


def main():
    pv.global_theme.allow_empty_mesh = True

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())
