#!/usr/bin/env python3
"""
STM32 SD Card Snapshot Visualizer
"""

import struct
import sys
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk, ImageEnhance
import numpy as np
import cv2
import threading
import customtkinter as ctk

# ========================= CONFIG =========================
HEADER_SIZE = 64
BLOCK_SIZE = 512
HEADER_TAG = 0x49444745
FMT_NAME = {0: "YUV422", 1: "RGB565", 2: "GRAY8"}
HDR_FMT = '<IIIIIII36s'
HDR_FIELDS = ['magic', 'width', 'height', 'pixel_format', 'data_size', 'timestamp', 'checksum', '_rsv']

# Block count per snapshot: matches C code exactly
# (HEADER_SIZE + frame_size + BLOCK_SIZE - 1) // BLOCK_SIZE
# For 2592x1944 YUV422: (64 + 10077696 + 511) // 512 = 19684 blocks
# For 1296x972 YUV422:  (64 +  2519424 + 511) // 512 =  4913 blocks
SNAP_W, SNAP_H = 1296, 972  #640, 480  #1296, 972 #2592, 1944
SNAP_FRAME_SIZE = SNAP_W * SNAP_H * 2
SNAP_BLOCKS = (HEADER_SIZE + SNAP_FRAME_SIZE + BLOCK_SIZE - 1) // BLOCK_SIZE  # 19684 for full res

# Try both old base (1000) and new base (3072) to support both firmware versions
SNAP_BASE_NEW = 3072
SNAP_BASE_OLD = 1000

_gf = None
_gfd = None
_gpath = None


def _open_source(path):
    global _gf, _gfd, _gpath
    if _gpath == path:
        return
    close_drive()

    _gpath = path
    if path.startswith('\\\\.\\PhysicalDrive'):
        flags = os.O_RDONLY
        if hasattr(os, 'O_BINARY'):
            flags |= os.O_BINARY
        _gfd = os.open(path, flags)
    else:
        _gf = open(path, 'rb')


def rblk(path, blk):
    global _gf, _gfd
    _open_source(path)
    offset = int(blk) * BLOCK_SIZE

    if _gfd is not None:
        if hasattr(os, 'pread'):
            data = os.pread(_gfd, BLOCK_SIZE, offset)
        else:
            os.lseek(_gfd, offset, os.SEEK_SET)
            data = os.read(_gfd, BLOCK_SIZE)
        return data

    _gf.seek(offset, os.SEEK_SET)
    return _gf.read(BLOCK_SIZE)


def rbulk(path, start_blk, num_blks):
    """Read multiple consecutive blocks in ONE I/O call (much faster than rblk loop).
    
    For full-res images (19684 blocks), reading block-by-block takes forever.
    This function does a single seek + read of all bytes at once."""
    global _gf, _gfd
    _open_source(path)
    offset = int(start_blk) * BLOCK_SIZE
    total_bytes = int(num_blks) * BLOCK_SIZE

    if _gfd is not None:
        if hasattr(os, 'pread'):
            data = os.pread(_gfd, total_bytes, offset)
        else:
            os.lseek(_gfd, offset, os.SEEK_SET)
            data = os.read(_gfd, total_bytes)
        return data

    _gf.seek(offset, os.SEEK_SET)
    return _gf.read(total_bytes)


def close_drive():
    global _gf, _gfd, _gpath
    if _gf:
        _gf.close()
        _gf = None
    if _gfd is not None:
        os.close(_gfd)
        _gfd = None
    _gpath = None


def parse_hdr(b):
    # Firmware header writes 8 u32 fields first; the remaining bytes are reserved.
    if len(b) < HEADER_SIZE:
        raise ValueError("Header too short")
    head = struct.unpack('<8I', b[:32])
    return {
        'magic': head[0],
        'width': head[1],
        'height': head[2],
        'pixel_format': head[3],
        'data_size': head[4],
        'timestamp': head[5],
        'checksum': head[6],
        'snap_id': head[7],
    }


def get_drives():
    out = []
    for n in range(32):
        p = f'\\\\.\\PhysicalDrive{n}'
        try:
            with open(p, 'rb') as f:
                out.append(p)
        except:
            pass
    return [d for d in out if 'PhysicalDrive0' not in d]


class Snapshot:
    def __init__(self, idx, block, header):
        self.idx = idx
        self.block = block
        self.header = header
        self.image_data = None


class SDVisualizer(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("STM32 SD Card Snapshot Visualizer")
        self.geometry("1420x900")
        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self.drive = None
        self.snapshots = []
        self.current_snap = None
        self.current_image = None
        self.current_scale = 1.0
        self.current_photo = None

        self._build_ui()
        self.refresh_drives()

    def _build_ui(self):
        # Toolbar
        toolbar = ctk.CTkFrame(self, height=70, fg_color="#f0f0f0")
        toolbar.pack(fill="x", padx=10, pady=8)

        ctk.CTkLabel(toolbar, text="Drive:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=(10,5))
        self.drive_var = tk.StringVar()
        self.drive_combo = ctk.CTkComboBox(toolbar, variable=self.drive_var, width=220, height=35)
        self.drive_combo.pack(side="left", padx=5)

        ctk.CTkButton(toolbar, text="↻ Refresh", width=100, height=35, command=self.refresh_drives).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="Scan Snapshots", width=140, height=35, command=self.scan_snapshots).pack(side="left", padx=5)

        sep = ctk.CTkFrame(toolbar, width=2, height=35, fg_color="gray70")
        sep.pack(side="left", padx=15)

        ctk.CTkButton(toolbar, text="Extract Selected", fg_color="green", width=150, height=35, command=self.extract_selected).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="🗑 Delete Selected", fg_color="#d9534f", width=150, height=35, command=self.delete_selected).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="Disconnect", width=120, height=35, command=self.disconnect_drive).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="Format SD", fg_color="red", width=110, height=35, command=self.format_card).pack(side="right", padx=10)

        # Main Area
        main_pane = ttk.PanedWindow(self, orient="horizontal")
        main_pane.pack(fill="both", expand=True, padx=10, pady=5)

        # Left List
        left = ctk.CTkFrame(main_pane, width=380)
        main_pane.add(left, weight=1)

        ctk.CTkLabel(left, text="Snapshots", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=8)
        self.listbox = tk.Listbox(left, font=("Segoe UI", 11), selectmode="extended")
        scroll = ctk.CTkScrollbar(left, command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=scroll.set)
        self.listbox.pack(fill="both", expand=True, padx=10, pady=5)
        scroll.pack(side="right", fill="y")
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        # Right Preview
        right = ctk.CTkFrame(main_pane)
        main_pane.add(right, weight=3)

        preview_frame = ctk.CTkFrame(right)
        preview_frame.pack(fill="both", expand=True, padx=10, pady=(10, 5))

        self.canvas = tk.Canvas(preview_frame, bg="#f0f0f0", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        v_scroll = ttk.Scrollbar(preview_frame, orient="vertical", command=self.canvas.yview)
        h_scroll = ttk.Scrollbar(preview_frame, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)

        v_scroll.pack(side="right", fill="y")
        h_scroll.pack(side="bottom", fill="x")

        # Mouse Wheel Zoom
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)
        self.canvas.bind("<Button-4>", self._on_mouse_wheel)
        self.canvas.bind("<Button-5>", self._on_mouse_wheel)

        # Controls
        ctrl = ctk.CTkFrame(right, height=165, fg_color="#f8f8f8")
        ctrl.pack(fill="x", padx=10, pady=8)
        ctrl.pack_propagate(False)

        ctk.CTkLabel(ctrl, text="Escala de Imagen", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=10, pady=(8,2))

        scale_frame = ctk.CTkFrame(ctrl)
        scale_frame.pack(fill="x", padx=10, pady=5)

        self.scale_var = tk.StringVar(value="Fit")
        scales = ["Fit", "0.25", "0.5", "0.75", "1.0", "1.5", "2.0"]
        for s in scales:
            ctk.CTkRadioButton(scale_frame, text=s if s != "Fit" else "Ajustar a Ventana",
                             variable=self.scale_var, value=s,
                             command=self.update_preview).pack(side="left", padx=8)

        # Save buttons
        save_frame = ctk.CTkFrame(ctrl)
        save_frame.pack(fill="x", padx=10, pady=8)

        ctk.CTkButton(save_frame, text="💾 Guardar Original", fg_color="green", height=35,
                      command=self.save_original).pack(side="left", padx=5)
        ctk.CTkButton(save_frame, text="💾 Guardar con Escala Actual", fg_color="#2e8b57", height=35,
                      command=self.save_scaled).pack(side="left", padx=5)

        self.status = ctk.CTkLabel(self, text="Ready — Ejecutar como Administrador", anchor="w")
        self.status.pack(fill="x", padx=12, pady=6)

    # ==================== CORE ====================

    def refresh_drives(self):
        drives = get_drives()
        self.drive_combo.configure(values=drives)
        if drives:
            self.drive_combo.set(drives[0])
            self.drive = drives[0]

    def _validate_header(self, h):
        """Validate header with relaxed checks to avoid false negatives."""
        if h.get('magic') != HEADER_TAG:
            return False

        w = h.get('width', 0)
        hh = h.get('height', 0)
        data_size = h.get('data_size', 0)

        # Width and height must be positive and reasonable
        if w == 0 or hh == 0 or w > 4096 or hh > 4096:
            return False

        # data_size should match w*hh*2 (YUV422) within a generous tolerance
        expected = w * hh * 2
        if expected > 0:
            tolerance = max(BLOCK_SIZE * 4, expected * 0.01)  # at least 4 blocks or 1%
            if abs(data_size - expected) > tolerance:
                return False

        return True

    def scan_snapshots(self):
        """Scan SD card for snapshots.

        Strategy: For each base block position, first try the FIXED-stride
        approach (fast, works when all images are same resolution). Then do
        a SECOND pass with VARIABLE stride: read the header, compute actual
        block count from data_size in the header, advance by that amount.
        This handles mixed-resolution SD cards (e.g., some images from
        callback-batch at 1296x972 and some from button at 2592x1944).
        """
        def task():
            self.set_status("Escaneando...")
            best_results = []
            best_label = ""

            for snap_base, label in [(SNAP_BASE_NEW, "new"), (SNAP_BASE_OLD, "old")]:

                # --- Pass 1: Fixed stride (fast) ---
                probe = snap_base
                count = 0
                gaps = 0
                fixed_results = []

                while count < 300:
                    try:
                        raw = rblk(self.drive, probe)
                        if len(raw) < HEADER_SIZE:
                            break
                        h = parse_hdr(raw[:HEADER_SIZE])
                        if self._validate_header(h):
                            fixed_results.append(Snapshot(count, probe, h))
                            count += 1
                            gaps = 0
                        else:
                            gaps += 1
                            if gaps >= 3:
                                break
                        probe += SNAP_BLOCKS
                    except OSError as e:
                        if getattr(e, 'errno', None) != 22:
                            print(f"[SCAN {label}] OSError at block {probe}: {e}")
                        break
                    except Exception as e:
                        print(f"[SCAN {label}] Exception at block {probe}: {e}")
                        break

                # --- Pass 2: Variable stride (handles mixed resolutions) ---
                probe = snap_base
                count = 0
                gaps = 0
                var_results = []

                while count < 300:
                    try:
                        raw = rblk(self.drive, probe)
                        if len(raw) < HEADER_SIZE:
                            break
                        h = parse_hdr(raw[:HEADER_SIZE])
                        if self._validate_header(h):
                            # Compute actual block count from header data_size
                            actual_nb = (HEADER_SIZE + h['data_size'] + BLOCK_SIZE - 1) // BLOCK_SIZE
                            if actual_nb < 1:
                                actual_nb = SNAP_BLOCKS  # fallback
                            var_results.append(Snapshot(count, probe, h))
                            count += 1
                            gaps = 0
                            probe += actual_nb  # variable stride!
                        else:
                            gaps += 1
                            if gaps >= 3:
                                break
                            probe += SNAP_BLOCKS  # use fixed stride for gaps
                    except OSError as e:
                        if getattr(e, 'errno', None) != 22:
                            print(f"[SCAN {label}] OSError at block {probe}: {e}")
                        break
                    except Exception as e:
                        print(f"[SCAN {label}] Exception at block {probe}: {e}")
                        break

                print(f"[SCAN] base={snap_base} ({label}): fixed={len(fixed_results)}, variable={len(var_results)}")

                # Pick the best result for this base
                if len(var_results) > len(fixed_results):
                    if len(var_results) > len(best_results):
                        best_results = var_results
                        best_label = f"{label} (variable stride, base={snap_base})"
                else:
                    if len(fixed_results) > len(best_results):
                        best_results = fixed_results
                        best_label = f"{label} (fixed stride, base={snap_base})"

            self.snapshots = best_results
            if self.snapshots:
                msg = f"Found {len(self.snapshots)} snapshots ({best_label})"
            else:
                msg = "No snapshots found"

            self.after(0, self._populate_list)
            self.after(0, lambda: self.set_status(msg))

        threading.Thread(target=task, daemon=True).start()

    def _populate_list(self):
        self.listbox.delete(0, tk.END)
        for s in self.snapshots:
            fmt = FMT_NAME.get(s.header['pixel_format'], "?")
            ts = time.strftime("%Y-%m-%d %H:%M", time.localtime(s.header.get('timestamp', 0)))
            sid = s.header.get('snap_id', s.idx)
            self.listbox.insert(tk.END, f"#{s.idx+1:03d} (id:{sid})   {s.header['width']}×{s.header['height']}   {fmt}   {ts}")

    def on_select(self, event=None):
        sel = self.listbox.curselection()
        if not sel:
            return
        self.current_snap = self.snapshots[sel[0]]
        self.load_preview()

    def load_full_image(self, snap):
        if snap.image_data:
            return snap.image_data

        h = snap.header
        w, hh = h['width'], h['height']
        data_size = h['data_size']

        if w <= 0 or hh <= 0:
            print(f"[LOAD] Invalid dimensions: {w}x{hh}")
            return None

        expected_size = w * hh * 2 if h.get('pixel_format', 0) == 0 else w * hh
        # Prefer the expected pixel payload size; reject unreasonable header values.
        if data_size <= 0 or data_size > expected_size * 2:
            print(f"[LOAD] Suspicious data_size={data_size}, expected={expected_size}. Using expected size.")
            data_size = expected_size
        elif abs(data_size - expected_size) > (BLOCK_SIZE * 4):
            print(f"[LOAD] data_size mismatch ({data_size} vs {expected_size}). Using expected size.")
            data_size = expected_size

        # Calculate total blocks for this image
        nb = (HEADER_SIZE + data_size + BLOCK_SIZE - 1) // BLOCK_SIZE
        # Full-res 2592x1944 = 19684 blocks; low-res 1296x972 = 4913 blocks
        if nb <= 0 or nb > 20000:
            print(f"[LOAD] Invalid block count: {nb}")
            return None

        import time
        t0 = time.time()

        # BULK READ: Read ALL blocks in ONE I/O call instead of 19684 separate reads.
        # This is ~100x faster for full-resolution images.
        print(f"[LOAD] Reading {nb} blocks ({nb*BLOCK_SIZE/1048576:.1f} MB) from block {snap.block}...")
        all_data = rbulk(self.drive, snap.block, nb)
        
        if not all_data or len(all_data) < nb * BLOCK_SIZE:
            print(f"[LOAD] Short bulk read: got {len(all_data)}, expected {nb*BLOCK_SIZE}")
            return None

        # Extract image data: skip header (first 64 bytes), take data_size bytes
        img_data = bytes(all_data[HEADER_SIZE:HEADER_SIZE + data_size])
        
        load_time = time.time() - t0
        print(f"[LOAD] Read complete in {load_time:.1f}s ({nb*BLOCK_SIZE/1048576/load_time:.1f} MB/s)")

        if len(img_data) < expected_size:
            print(f"[LOAD] Incomplete image payload: got {len(img_data)}, expected {expected_size}")
            return None

        # Convert to PIL Image
        try:
            if h['pixel_format'] == 0:
                # YUV422 (YUY2) to RGB
                arr = np.frombuffer(img_data, np.uint8)
                if len(arr) >= hh * w * 2:
                    arr = arr[:hh * w * 2].reshape(hh, w, 2)
                    bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)
                    pil = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                else:
                    raise ValueError(f"Image data too short: {len(arr)} < {hh * w * 2}")
            else:
                arr = np.frombuffer(img_data, np.uint8)
                pil = Image.fromarray(arr.reshape(hh, w))
        except Exception as e:
            print(f"[LOAD] Error converting image: {e}")
            return None

        snap.image_data = pil
        return pil

    def load_preview(self):
        def task():
            self.set_status("Cargando imagen...")
            img = self.load_full_image(self.current_snap)
            if img:
                self.current_image = img
                # IMPORTANT: Use self.after() to schedule canvas updates on main thread
                # This prevents the hang that occurred when calling canvas methods from a thread
                self.after(0, self.fit_to_window)
                self.after(0, lambda: self.set_status("Listo - Rueda del ratón para zoom"))
            else:
                self.after(0, lambda: self.set_status("Error cargando imagen"))

        threading.Thread(target=task, daemon=True).start()

    def fit_to_window(self):
        """Ajusta la imagen automáticamente al tamaño visible.
        MUST be called on the main thread (via self.after())."""
        if not self.current_image:
            return

        # Get canvas size
        self.canvas.update_idletasks()
        canvas_w = self.canvas.winfo_width() - 20
        canvas_h = self.canvas.winfo_height() - 20

        if canvas_w <= 0 or canvas_h <= 0:
            canvas_w = 800
            canvas_h = 600

        img_w, img_h = self.current_image.width, self.current_image.height

        scale_w = canvas_w / img_w
        scale_h = canvas_h / img_h
        self.current_scale = min(scale_w, scale_h, 1.0)

        self.show_image_on_canvas()

    def show_image_on_canvas(self):
        """Display image on canvas at current scale.
        MUST be called on the main thread."""
        if not self.current_image:
            return

        new_w = int(self.current_image.width * self.current_scale)
        new_h = int(self.current_image.height * self.current_scale)

        resized = self.current_image.resize((new_w, new_h), Image.LANCZOS)

        if self.current_scale > 1.1:
            enhancer = ImageEnhance.Sharpness(resized)
            resized = enhancer.enhance(1.4)

        self.current_photo = ImageTk.PhotoImage(resized)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self.current_photo)
        self.canvas.configure(scrollregion=(0, 0, new_w, new_h))

    def _on_mouse_wheel(self, event):
        if not self.current_image:
            return

        if event.num == 5 or event.delta < 0:
            self.current_scale *= 0.9
        else:
            self.current_scale *= 1.1

        self.current_scale = max(0.15, min(self.current_scale, 5.0))
        self.show_image_on_canvas()
        self.set_status(f"Zoom: {self.current_scale:.2f}x")

    def update_preview(self):
        if not self.current_image:
            return
        if self.scale_var.get() == "Fit":
            self.fit_to_window()
        else:
            self.current_scale = float(self.scale_var.get())
            self.show_image_on_canvas()

    # ====================== SAVE ======================
    def save_original(self):
        self._save_image(scale=1.0, suffix="original")

    def save_scaled(self):
        self._save_image(scale=self.current_scale, suffix=f"{self.current_scale:.2f}x")

    def _save_image(self, scale=1.0, suffix=""):
        if not self.current_snap or not self.current_image:
            messagebox.showwarning("Advertencia", "Selecciona una imagen primero")
            return
        out_dir = filedialog.askdirectory(title="Seleccionar carpeta")
        if not out_dir:
            return

        try:
            img = self.current_image.copy()
            if scale != 1.0:
                new_size = (int(img.width * scale), int(img.height * scale))
                img = img.resize(new_size, Image.LANCZOS)
                if scale > 1.1:
                    img = ImageEnhance.Sharpness(img).enhance(1.4)

            filename = f"snapshot_{self.current_snap.idx:03d}_{suffix}.png"
            path = os.path.join(out_dir, filename)
            img.save(path)
            messagebox.showinfo("Guardado", f"Imagen guardada:\n{path}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def set_status(self, text):
        self.after(0, lambda: self.status.configure(text=text))

    def extract_selected(self):
        messagebox.showinfo("Info", "Extract Selected - pendiente")

    def delete_selected(self):
        messagebox.showinfo("Info", "Delete Selected - pendiente")

    def disconnect_drive(self):
        close_drive()
        self.set_status("Drive desconectado")

    def format_card(self):
        if not self.drive:
            messagebox.showwarning("Error", "Primero selecciona una unidad")
            return

        # Extract drive number
        drive_num = self.drive.replace('\\\\.\\PhysicalDrive', '')

        answer = messagebox.askyesno(
            "⚠️ FORMATEAR TARJETA SD",
            f"¿Estás SEGURO que quieres formatear la unidad PhysicalDrive{drive_num} como FAT32?\n\n"
            "¡Esta acción BORRARÁ TODOS los datos de forma permanente!\n\n"
            "Solo continúa si estás 100% seguro.",
            icon="warning"
        )

        if not answer:
            return

        # Double confirmation
        confirm = tk.simpledialog.askstring(
            "Confirmación Final",
            f"Escribe 'SI' para confirmar el formateo de PhysicalDrive{drive_num}:",
            parent=self
        )

        if confirm != "SI":
            messagebox.showinfo("Cancelado", "Formateo cancelado.")
            return

        self.set_status(f"Iniciando formateo de PhysicalDrive{drive_num} como FAT32...")

        def format_task():
            try:
                import subprocess

                self.set_status("Desmontando unidad...")
                # Try to dismount
                subprocess.run(f"mountvol {drive_num} /p", shell=True, capture_output=True)

                self.set_status("Formateando como FAT32 (Large FAT32)...")

                # Use PowerShell for more modern formatting
                ps_command = f'''
                $drive = Get-Disk -Number {drive_num} | Get-Partition | Get-Volume
                if ($drive) {{
                    Format-Volume -DriveLetter $drive.DriveLetter -FileSystem FAT32 -NewFileSystemLabel "SD_CARD" -Full -Force -Confirm:$false
                }} else {{
                    Format-Volume -DiskNumber {drive_num} -FileSystem FAT32 -NewFileSystemLabel "SD_CARD" -Full -Force -Confirm:$false
                }}
                '''

                result = subprocess.run(["powershell", "-Command", ps_command],
                                       capture_output=True, text=True, timeout=120)

                if result.returncode == 0:
                    self.after(0, lambda: messagebox.showinfo("Éxito",
                        f"Tarjeta formateada correctamente como FAT32.\n\nUnidad: PhysicalDrive{drive_num}"))
                    self.after(0, self.refresh_drives)
                else:
                    # Fallback: use format.exe
                    self.set_status("Intentando método alternativo...")
                    fallback = subprocess.run(f"format {drive_num}: /FS:FAT32 /Q /Y /V:SD_CARD",
                                            shell=True, capture_output=True, text=True)
                    if fallback.returncode == 0:
                        self.after(0, lambda: messagebox.showinfo("Éxito", "Formateado con método alternativo."))
                    else:
                        self.after(0, lambda: messagebox.showerror("Error",
                            f"Error al formatear:\n{result.stderr or fallback.stderr}"))

            except Exception as e:
                self.after(0, lambda: messagebox.showerror("Error", f"Error durante el formateo:\n{str(e)}"))
            finally:
                self.after(0, lambda: self.set_status("Formateo finalizado."))

        threading.Thread(target=format_task, daemon=True).start()


if __name__ == "__main__":
    if os.name != 'nt':
        print("Solo funciona en Windows")
        sys.exit(1)
    app = SDVisualizer()
    app.mainloop()
    close_drive()