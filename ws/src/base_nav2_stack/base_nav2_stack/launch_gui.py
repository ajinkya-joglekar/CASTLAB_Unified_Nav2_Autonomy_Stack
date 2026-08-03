"""Tkinter front end for staged base_nav2_stack bringup."""

import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import yaml
from ament_index_python.packages import get_package_share_directory

from base_nav2_stack.profiles import validate_registries


TOPIC_MAPPINGS = (
    ("Command velocity", "cmd_vel_topic", "/cmd_vel", "geometry_msgs/msg/Twist"),
    ("Raw odometry", "odom_topic", "/odom", "nav_msgs/msg/Odometry"),
    ("Filtered odometry", "filtered_odom_topic", "/odometry/filtered", "nav_msgs/msg/Odometry"),
    ("IMU", "imu_topic", "/vectornav/imu/data", "sensor_msgs/msg/Imu"),
    ("GPS fix", "gps_topic", "/gps/fix", "sensor_msgs/msg/NavSatFix"),
    ("Laser scan", "scan_topic", "/scan", "sensor_msgs/msg/LaserScan"),
    ("Point cloud", "points_topic", "/points", "sensor_msgs/msg/PointCloud2"),
    ("Camera RGB image", "camera_image_topic", "/camera/image_raw", "sensor_msgs/msg/Image"),
    ("Camera info", "camera_info_topic", "/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo"),
    ("Camera depth image", "camera_depth_image_topic", "/camera/depth/image_raw", "sensor_msgs/msg/Image"),
    ("Camera depth points", "camera_points_topic", "/camera/depth/points", "sensor_msgs/msg/PointCloud2"),
)


class LauncherGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Nav2 Base Stack Launcher")
        self.share = Path(get_package_share_directory("base_nav2_stack"))
        self.vehicles, self.worlds = validate_registries(self.share)
        self.segmentation_models = self._load_segmentation_models()
        self.sim_process = None
        self.nav_process = None
        self.output = queue.Queue()
        self.topic_widgets = {}
        self.topic_keys = {key for _, key, _, _ in TOPIC_MAPPINGS}
        self.discovered_topics_by_type = {}
        self.perception_widgets = []
        self.mapping_dirty = False
        self.restart_alert_shown = False
        self._updating_topics = False
        self.vars = {
            "mode": tk.StringVar(value="sim"), "vehicle": tk.StringVar(value="ranger"),
            "world": tk.StringVar(value="CAST_new"), "world_name": tk.StringVar(),
            "localization_mode": tk.StringVar(value="managed"),
            "spawn_x": tk.StringVar(), "spawn_y": tk.StringVar(),
            "spawn_z": tk.StringVar(), "spawn_yaw": tk.StringVar(),
            "ekf_config": tk.StringVar(), "nav2_config": tk.StringVar(),
            "cmd_vel_topic": tk.StringVar(), "odom_topic": tk.StringVar(),
            "filtered_odom_topic": tk.StringVar(value="/odometry/filtered"),
            "imu_topic": tk.StringVar(), "gps_topic": tk.StringVar(),
            "scan_topic": tk.StringVar(), "points_topic": tk.StringVar(),
            "camera_image_topic": tk.StringVar(), "camera_info_topic": tk.StringVar(),
            "camera_depth_image_topic": tk.StringVar(), "camera_points_topic": tk.StringVar(),
            "use_perception": tk.BooleanVar(value=False),
            "segmentation_model": tk.StringVar(value="rellis_7_class"),
            "segmentation_model_path": tk.StringVar(),
            "segmentation_config_path": tk.StringVar(),
            "terrain_mapper_config": tk.StringVar(value="config/perception/terrain_mapper.yaml"),
            "terrain_semantic_profile": tk.StringVar(),
            "use_depth_image_proc": tk.BooleanVar(value=True),
            "use_rviz": tk.BooleanVar(value=True), "use_mapviz": tk.BooleanVar(value=False),
            "use_composition": tk.BooleanVar(value=False),
            "autostart": tk.BooleanVar(value=True), "use_respawn": tk.BooleanVar(value=False),
        }
        self._build()
        self._vehicle_changed()
        self._world_changed()
        self._mode_changed()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(100, self._drain_output)

    def _build(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        row = 0
        ttk.Label(frame, text="Mode").grid(row=row, column=0, sticky="w")
        for index, value in enumerate(("sim", "hardware"), 1):
            ttk.Radiobutton(frame, text=value.title(), value=value, variable=self.vars["mode"],
                            command=self._mode_changed).grid(row=row, column=index, sticky="w")
        row += 1
        self.vehicle = self._combo(frame, row, "Vehicle", "vehicle", list(self.vehicles), self._vehicle_changed); row += 1
        self.world = self._combo(frame, row, "World", "world", list(self.worlds), self._world_changed); row += 1
        ttk.Label(frame, text="Gazebo world name").grid(row=row, column=0, sticky="w")
        self.world_name = ttk.Entry(frame, textvariable=self.vars["world_name"])
        self.world_name.grid(row=row, column=1, columnspan=4, sticky="ew"); row += 1
        self.localization = self._combo(frame, row, "Localization", "localization_mode",
                                        ["managed", "external"], self._mode_changed); row += 1

        self.spawn_widgets = []
        ttk.Label(frame, text="Spawn x / y / z / yaw").grid(row=row, column=0, sticky="w")
        for col, key in enumerate(("spawn_x", "spawn_y", "spawn_z", "spawn_yaw"), 1):
            widget = ttk.Entry(frame, textvariable=self.vars[key], width=9)
            widget.grid(row=row, column=col, sticky="ew")
            self.spawn_widgets.append(widget)
        row += 1

        for label, key in (("EKF YAML", "ekf_config"), ("Nav2 YAML", "nav2_config")):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(frame, textvariable=self.vars[key], width=55).grid(row=row, column=1, columnspan=3, sticky="ew")
            ttk.Button(frame, text="Browse", command=lambda k=key: self._browse(k)).grid(row=row, column=4)
            row += 1

        ttk.Separator(frame).grid(row=row, column=0, columnspan=5, sticky="ew", pady=(4, 4)); row += 1
        ttk.Checkbutton(
            frame,
            text="Enable perception / traversability",
            variable=self.vars["use_perception"],
            command=self._perception_changed,
        ).grid(row=row, column=0, columnspan=2, sticky="w")
        ttk.Label(
            frame,
            text="Nav2 uses terrain only when the selected Nav2 YAML includes terrain_layer.",
        ).grid(row=row, column=2, columnspan=3, sticky="w")
        row += 1
        self.segmentation_model = self._combo(
            frame, row, "Segmentation model", "segmentation_model",
            list(self.segmentation_models),
        ); row += 1
        self.perception_widgets.append(self.segmentation_model)
        for label, key in (("Custom ONNX model", "segmentation_model_path"),
                           ("Custom ontology YAML", "segmentation_config_path"),
                           ("Terrain mapper YAML", "terrain_mapper_config")):
            label_widget = ttk.Label(frame, text=label)
            label_widget.grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(frame, textvariable=self.vars[key], width=55)
            entry.grid(row=row, column=1, columnspan=3, sticky="ew")
            button = ttk.Button(frame, text="Browse", command=lambda k=key: self._browse(k))
            button.grid(row=row, column=4)
            self.perception_widgets.extend([label_widget, entry, button])
            row += 1
        semantic_label = ttk.Label(frame, text="Semantic profile override")
        semantic_label.grid(row=row, column=0, sticky="w")
        semantic_entry = ttk.Entry(frame, textvariable=self.vars["terrain_semantic_profile"], width=55)
        semantic_entry.grid(row=row, column=1, columnspan=4, sticky="ew")
        self.perception_widgets.extend([semantic_label, semantic_entry])
        row += 1
        depth_checkbox = ttk.Checkbutton(
            frame,
            text="Generate RGB-D points from depth image + camera info",
            variable=self.vars["use_depth_image_proc"],
        )
        depth_checkbox.grid(row=row, column=0, columnspan=3, sticky="w")
        depth_label = ttk.Label(
            frame,
            text="Disable only if /camera/depth/points is already a trusted organized ROS point cloud.",
        )
        depth_label.grid(row=row, column=3, columnspan=2, sticky="w")
        self.perception_widgets.extend([depth_checkbox, depth_label])
        row += 1

        for col, (label, key) in enumerate((("RViz", "use_rviz"), ("Mapviz", "use_mapviz"),
                                             ("Composition", "use_composition"),
                                             ("Autostart", "autostart"), ("Respawn", "use_respawn"))):
            ttk.Checkbutton(frame, text=label, variable=self.vars[key]).grid(row=row, column=col, sticky="w")
        row += 1

        self.command = tk.Text(frame, height=4, wrap="word")
        self.command.grid(row=row, column=0, columnspan=5, sticky="ew"); row += 1

        buttons = ttk.Frame(frame)
        buttons.grid(row=row, column=0, columnspan=5, sticky="ew", pady=(4, 4))
        ttk.Button(buttons, text="Launch Sim", command=self.launch_sim).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(buttons, text="Launch Nav2", command=self.launch_nav).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(buttons, text="Refresh Topics", command=self.refresh_topics).grid(row=0, column=2, padx=(0, 4))
        ttk.Button(buttons, text="Map Topics", command=self.show_topics).grid(row=0, column=3, padx=(0, 4))
        ttk.Button(buttons, text="Show Logs", command=self.show_logs).grid(row=0, column=4, padx=(0, 4))
        ttk.Button(buttons, text="Stop Sim", command=self.stop_sim).grid(row=0, column=5, padx=(0, 4))
        ttk.Button(buttons, text="Stop Nav2", command=self.stop_nav).grid(row=0, column=6, padx=(0, 4))
        ttk.Button(buttons, text="Stop All", command=self.stop_all).grid(row=0, column=7, padx=(0, 4))
        row += 1

        self.lower = ttk.Frame(frame)
        self.lower.grid(row=row, column=0, columnspan=5, sticky="nsew")
        self.logs_frame = ttk.Frame(self.lower)
        self.topics_frame = ttk.Frame(self.lower)
        for child in (self.logs_frame, self.topics_frame):
            child.grid(row=0, column=0, sticky="nsew")
        self.lower.rowconfigure(0, weight=1)
        self.lower.columnconfigure(0, weight=1)
        self._build_logs_panel()
        self._build_topics_panel()
        self.show_logs()
        self._perception_changed()

        frame.rowconfigure(row, weight=1)
        frame.columnconfigure(3, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        for key, var in self.vars.items():
            var.trace_add("write", lambda *_, k=key: self._vars_changed(k))

    def _build_logs_panel(self):
        self.log = tk.Text(self.logs_frame, height=18, width=110, state="disabled")
        self.log.grid(row=0, column=0, sticky="nsew")
        self.logs_frame.rowconfigure(0, weight=1)
        self.logs_frame.columnconfigure(0, weight=1)

    def _build_topics_panel(self):
        self.topic_warning = tk.StringVar(value="")
        ttk.Label(
            self.topics_frame,
            textvariable=self.topic_warning,
            foreground="dark orange",
        ).grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 6))
        ttk.Label(self.topics_frame, text="Vehicle / platform source topic").grid(row=1, column=1, sticky="w")
        ttk.Label(self.topics_frame, text="Canonical base-stack topic").grid(row=1, column=2, sticky="w")
        ttk.Label(self.topics_frame, text="Expected type").grid(row=1, column=3, sticky="w")
        for row, (label, key, canonical, msg_type) in enumerate(TOPIC_MAPPINGS, 2):
            ttk.Label(self.topics_frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8))
            dropdown = ttk.Combobox(self.topics_frame, textvariable=self.vars[key], width=34, state="normal")
            dropdown.grid(row=row, column=1, sticky="ew", padx=(0, 8))
            dropdown.bind("<<ComboboxSelected>>", lambda _event, k=key: self._topic_selected(k))
            ttk.Label(self.topics_frame, text=canonical).grid(row=row, column=2, sticky="w", padx=(0, 8))
            ttk.Label(self.topics_frame, text=msg_type).grid(row=row, column=3, sticky="w")
            self.topic_widgets[key] = dropdown
        ttk.Label(
            self.topics_frame,
            text="Discovered ROS topics (read-only)",
        ).grid(row=1, column=4, sticky="w", padx=(16, 0))
        self.discovered = tk.Text(self.topics_frame, height=14, width=52, state="disabled")
        self.discovered.grid(row=2, column=4, rowspan=len(TOPIC_MAPPINGS), sticky="nsew", padx=(16, 0))
        self.topics_frame.columnconfigure(1, weight=1)
        self.topics_frame.columnconfigure(4, weight=1)
        self.topics_frame.rowconfigure(len(TOPIC_MAPPINGS) + 1, weight=1)

    def _combo(self, parent, row, label, key, values, callback=None):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        widget = ttk.Combobox(parent, textvariable=self.vars[key], values=values, state="readonly")
        widget.grid(row=row, column=1, columnspan=4, sticky="ew")
        if callback:
            widget.bind("<<ComboboxSelected>>", lambda _: callback())
        return widget

    def _load_segmentation_models(self):
        path = self.share / "config/perception/segmentation_models.yaml"
        try:
            with path.open("r", encoding="utf-8") as stream:
                return (yaml.safe_load(stream) or {}).get("models", {})
        except FileNotFoundError:
            return {}

    def _browse(self, key):
        path = filedialog.askopenfilename(filetypes=[("YAML", "*.yaml *.yml"), ("All", "*")])
        if path:
            self.vars[key].set(path)

    def _vehicle_changed(self):
        self._updating_topics = True
        try:
            vehicle = self.vehicles[self.vars["vehicle"].get()]
            topics = vehicle["topics"]
            for key in ("cmd_vel", "odom", "imu", "gps", "scan", "points",
                        "camera_image", "camera_info", "camera_depth_image",
                        "camera_points"):
                self.vars[key + "_topic"].set(str(topics.get(key, "")))
            self.vars["filtered_odom_topic"].set(str(topics["filtered_odom"]))
        finally:
            self._updating_topics = False
            self.mapping_dirty = False
            self.restart_alert_shown = False
            self.topic_warning.set("")
            self._refresh_topic_dropdown_values()
            self._render_discovered_topics()

    def _world_changed(self):
        world = self.worlds[self.vars["world"].get()]
        spawn = world["spawn"]
        self.vars["world_name"].set(str(world["gazebo_name"]))
        for key in ("x", "y", "z", "yaw"):
            self.vars["spawn_" + key].set(str(spawn[key]))

    def _mode_changed(self):
        sim = self.vars["mode"].get() == "sim"
        self.world.configure(state="readonly" if sim else "disabled")
        self.world_name.configure(state="normal" if sim else "disabled")
        for widget in self.spawn_widgets:
            widget.configure(state="normal" if sim else "disabled")

    def _perception_changed(self):
        state = "normal" if self.vars["use_perception"].get() else "disabled"
        for widget in self.perception_widgets:
            if widget is self.segmentation_model and state == "normal":
                widget.configure(state="readonly")
            else:
                widget.configure(state=state)

    def _vars_changed(self, key):
        if key == "use_perception":
            self._perception_changed()
        if key in self.topic_keys and not self._updating_topics:
            self.mapping_dirty = True
            self._topic_mapping_changed()
        self._show_command()

    def _topic_selected(self, _key):
        self._show_command()

    def _topic_mapping_changed(self):
        sim_running = self.sim_process and self.sim_process.poll() is None
        warning = "Topic mappings changed: restart Nav2 for changes to take effect."
        if sim_running:
            warning += " Restart sim/bridge too if Gazebo source topics changed."
        self.topic_warning.set(warning)
        nav_running = self.nav_process and self.nav_process.poll() is None
        if nav_running and not self.restart_alert_shown:
            messagebox.showwarning(
                "Restart Nav2 required",
                warning,
            )
            self.restart_alert_shown = True

    def _bool_arg(self, key):
        return str(self.vars[key].get())

    def _base_args(self, stage):
        args = ["ros2", "launch", "base_nav2_stack", "base_stack.launch.py",
                f"launch_stage:={stage}"]
        keys = ("mode", "vehicle", "world", "world_name", "localization_mode",
                "ekf_config", "nav2_config",
                "cmd_vel_topic", "odom_topic", "filtered_odom_topic", "imu_topic", "gps_topic",
                "scan_topic", "points_topic", "camera_image_topic", "camera_info_topic",
                "camera_depth_image_topic", "camera_points_topic",
                "use_perception", "segmentation_model", "segmentation_model_path",
                "segmentation_config_path", "terrain_mapper_config", "terrain_semantic_profile",
                "use_depth_image_proc",
                "spawn_x", "spawn_y", "spawn_z", "spawn_yaw",
                "use_rviz", "use_mapviz", "use_composition", "autostart", "use_respawn")
        for key in keys:
            value = self.vars[key].get()
            if isinstance(value, bool):
                value = str(value)
            if value != "":
                args.append(f"{key}:={value}")
        return args

    def _show_command(self):
        if not hasattr(self, "command"):
            return
        sim_cmd = " ".join(shlex.quote(item) for item in self._base_args("sim"))
        nav_cmd = " ".join(shlex.quote(item) for item in self._base_args("nav"))
        rendered = f"# Sim\n{sim_cmd}\n# Nav2\n{nav_cmd}"
        self.command.delete("1.0", "end")
        self.command.insert("1.0", rendered)

    def _validate_common(self):
        if self.vars["mode"].get() == "sim":
            for key in ("spawn_x", "spawn_y", "spawn_z", "spawn_yaw"):
                float(self.vars[key].get())
        for key in ("ekf_config", "nav2_config"):
            value = self.vars[key].get().strip()
            if value and not Path(value).expanduser().is_file():
                raise ValueError(f"Missing {key}: {value}")
        if self.vars["use_perception"].get():
            if self.vars["segmentation_model"].get() not in self.segmentation_models:
                raise ValueError(f"Unknown segmentation model: {self.vars['segmentation_model'].get()}")
            for key in ("segmentation_model_path", "segmentation_config_path"):
                value = self.vars[key].get().strip()
                if value and not Path(value).expanduser().is_file():
                    raise ValueError(f"Missing {key}: {value}")
            terrain_config = self.vars["terrain_mapper_config"].get().strip()
            if terrain_config:
                terrain_path = Path(terrain_config).expanduser()
                package_path = self.share / terrain_config
                if not terrain_path.is_file() and not package_path.is_file():
                    raise ValueError(f"Missing terrain_mapper_config: {terrain_config}")
        for _, key, _, _ in TOPIC_MAPPINGS:
            if not self.vars[key].get().strip():
                raise ValueError(f"Missing topic mapping: {key}")

    def _start_process(self, name, args):
        process = getattr(self, f"{name}_process")
        if process and process.poll() is None:
            messagebox.showwarning("Already running", f"Stop the current {name} launch first.")
            return
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, bufsize=1, creationflags=flags,
                                   start_new_session=(os.name != "nt"))
        setattr(self, f"{name}_process", process)
        threading.Thread(target=self._read_output, args=(name, process), daemon=True).start()
        self.show_logs()

    def launch_sim(self):
        if self.vars["mode"].get() != "sim":
            messagebox.showinfo("Simulation only", "Launch Sim is only available in sim mode.")
            return
        try:
            self._validate_common()
        except Exception as exc:
            messagebox.showerror("Invalid configuration", str(exc))
            return
        self._start_process("sim", self._base_args("sim"))
        self.mapping_dirty = False

    def launch_nav(self):
        if self.sim_process and self.sim_process.poll() is None and self.mapping_dirty:
            messagebox.showwarning(
                "Topic mappings changed",
                "Source topic mappings changed after sim/bridge launch. Restart sim/bridge to apply bridge mappings.",
            )
        try:
            self._validate_common()
        except Exception as exc:
            messagebox.showerror("Invalid configuration", str(exc))
            return
        self._start_process("nav", self._base_args("nav"))
        self.mapping_dirty = False
        self.restart_alert_shown = False
        self.topic_warning.set("")

    def refresh_topics(self):
        self.show_topics()
        threading.Thread(target=self._refresh_topics_worker, daemon=True).start()

    def _refresh_topics_worker(self):
        try:
            result = subprocess.run(["ros2", "topic", "list", "-t"], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, timeout=8)
            self.output.put(("topics", self._parse_topic_list(result.stdout or "")))
            return
        except FileNotFoundError:
            text = "ros2 executable not found. Source your ROS environment before refreshing topics."
        except subprocess.TimeoutExpired:
            text = "Timed out waiting for ros2 topic list -t."
        self.output.put(("topic_error", text if text.endswith("\n") else text + "\n"))

    @staticmethod
    def _parse_topic_list(text):
        topics_by_type = {}
        for line in text.splitlines():
            line = line.strip()
            if not line or "[" not in line or "]" not in line:
                continue
            topic, typed = line.split("[", 1)
            topic = topic.strip()
            msg_type = typed.split("]", 1)[0].strip()
            if topic and msg_type:
                topics_by_type.setdefault(msg_type, []).append(topic)
        return {key: sorted(set(values)) for key, values in topics_by_type.items()}

    def _read_output(self, name, process):
        if process.stdout:
            for line in process.stdout:
                self.output.put((name, line))
        label = "nav2" if name == "nav" else name
        self.output.put((name, f"\n[{label} process exited: {process.wait()}]\n"))

    def _drain_output(self):
        while True:
            try:
                source, line = self.output.get_nowait()
            except queue.Empty:
                break
            if source == "topics":
                self.discovered_topics_by_type = line
                self._refresh_topic_dropdown_values()
                self._render_discovered_topics()
                count = sum(len(values) for values in self.discovered_topics_by_type.values())
                self._append_log(f"[topics] refreshed {count} typed topics\n")
            elif source == "topic_error":
                self._append_log(f"[topics] {line}")
            else:
                label = "nav2" if source == "nav" else source
                self._append_log(f"[{label}] {line}")
        self.root.after(100, self._drain_output)

    def _refresh_topic_dropdown_values(self):
        if not self.topic_widgets:
            return
        for _label, key, _canonical, msg_type in TOPIC_MAPPINGS:
            current = self.vars[key].get().strip()
            candidates = list(self.discovered_topics_by_type.get(msg_type, []))
            if current and current not in candidates:
                candidates.insert(0, current)
            self.topic_widgets[key]["values"] = candidates

    def _render_discovered_topics(self):
        if not hasattr(self, "discovered"):
            return
        if not self.discovered_topics_by_type:
            text = (
                "No live topics discovered yet.\n\n"
                "Launch the sim or hardware drivers, then click Refresh Topics.\n"
                "Use the source-topic dropdowns on the left to map into the fixed canonical topics.\n"
            )
        else:
            lines = []
            for msg_type in sorted(self.discovered_topics_by_type):
                lines.append(f"{msg_type}")
                for topic in self.discovered_topics_by_type[msg_type]:
                    lines.append(f"  {topic}")
                lines.append("")
            text = "\n".join(lines).rstrip() + "\n"
        self.discovered.configure(state="normal")
        self.discovered.delete("1.0", "end")
        self.discovered.insert("1.0", text)
        self.discovered.configure(state="disabled")

    def _append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _stop_process(self, name):
        process = getattr(self, f"{name}_process")
        if not process or process.poll() is not None:
            return
        if os.name == "nt":
            process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            if os.name == "nt":
                process.terminate()
            else:
                os.killpg(process.pid, signal.SIGTERM)

    def stop_sim(self):
        self._stop_process("sim")

    def stop_nav(self):
        self._stop_process("nav")

    def stop_all(self):
        self.stop_nav()
        self.stop_sim()

    def show_logs(self):
        self.logs_frame.tkraise()

    def show_topics(self):
        self.topics_frame.tkraise()

    def _close(self):
        self.stop_all()
        self.root.destroy()


def main():
    root = tk.Tk()
    LauncherGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
