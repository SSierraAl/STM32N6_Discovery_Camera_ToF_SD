#!/usr/bin/env python3
"""
STM32 SD Card Snapshot Visualizer — Pro + AI Enhancer
"""

import struct
import sys
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk, ImageEnhance, ImageFilter
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

SNAP_BASE = 3072
SNAP_W, SNAP_H = 2592, 1944
SNAP_BLOCKS = (HEADER_SIZE + SNAP_W * SNAP_H * 2 + BLOCK_SIZE - 1) // BLOCK_SIZE
COUNT_FILE = 'snapshot_count.txt'

_gf = None

def rblk(path, blk):
    global _gf
    if _gf is None or _gf.name != path:
        if _gf: _gf.close()
        _gf = open(path, 'rb')
    _gf.seek(blk * BLOCK_SIZE)
    return _gf.read(BLOCK_SIZE)

def close_drive():
    global _gf
    if _gf:
        _gf.close()
        _gf = None

def parse_hdr(b):
    return dict(zip(HDR_FIELDS, struct.unpack(HDR_FMT, b)))

def get_drives():
    out = []
    for n in range(32):
        p = f'\\\\.\\PhysicalDrive{n}'
        try:
            with open(p, 'rb') as f: out.append(p)
        except: pass
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
        self.title("STM32 SD Card Snapshot Visualizer — Pro AI")
        self.geometry("1360x860")
        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self.drive = None
        self.snapshots = []
        self.current_snap = None
        self.current_photo = None
        self.enhanced_photo = None

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
        ctk.CTkButton(toolbar, text="AI Enhance", fg_color="#6f42c1", hover_color="#5a32a3",
                      width=140, height=35, command=self.ai_enhance_current).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="Disconnect", width=120, height=35, command=self.disconnect_drive).pack(side="left", padx=5)
        ctk.CTkButton(toolbar, text="Format SD", fg_color="red", width=110, height=35, command=self.format_card).pack(side="right", padx=10)

        # Main Paned Window
        main_pane = ttk.PanedWindow(self, orient="horizontal")
        main_pane.pack(fill="both", expand=True, padx=10, pady=5)

        # Left: List
        left = ctk.CTkFrame(main_pane, width=400)
        main_pane.add(left, weight=1)

        ctk.CTkLabel(left, text="Snapshots", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=8)
        self.listbox = tk.Listbox(left, font=("Segoe UI", 11), selectmode="extended")
        scroll = ctk.CTkScrollbar(left, command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=scroll.set)
        self.listbox.pack(fill="both", expand=True, padx=10, pady=5)
        scroll.pack(side="right", fill="y")
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        # Right: Preview + Controls
        right = ctk.CTkFrame(main_pane)
        main_pane.add(right, weight=3)

        self.preview_label = ctk.CTkLabel(right, text="Select a snapshot...\n\nDouble-click for fullscreen")
        self.preview_label.pack(expand=True, fill="both", padx=10, pady=10)

        # Quick Enhancer Controls
        ctrl = ctk.CTkFrame(right, height=160)
        ctrl.pack(fill="x", padx=10, pady=8)
        ctrl.pack_propagate(False)

        ctk.CTkLabel(ctrl, text="AI Enhancer", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=10, pady=(5,2))

        btn_frame = ctk.CTkFrame(ctrl)
        btn_frame.pack(fill="x", padx=10, pady=5)

        ctk.CTkButton(btn_frame, text="Auto AI Enhance (2x Upscale)", fg_color="#6f42c1",
                      command=self.ai_enhance_current).pack(side="left", padx=5)
        ctk.CTkButton(btn_frame, text="Save Enhanced", fg_color="green",
                      command=self.save_enhanced).pack(side="left", padx=5)

        self.status = ctk.CTkLabel(self, text="Ready — Run as Administrator", anchor="w")
        self.status.pack(fill="x", padx=12, pady=6)

    # ==================== CORE FUNCTIONS ====================

    def refresh_drives(self):
        drives = get_drives()
        self.drive_combo.configure(values=drives)
        if drives:
            self.drive_combo.set(drives[0])
            self.drive = drives[0]

    def scan_snapshots(self):
        # ... (same as previous version - unchanged for brevity)
        def task():
            self.set_status("Scanning...")
            self.snapshots.clear()
            self.listbox.delete(0, tk.END)
            probe = SNAP_BASE
            count = 0
            while count < 300:
                try:
                    hdr = rblk(self.drive, probe)[:HEADER_SIZE]
                    h = parse_hdr(hdr)
                    if h['magic'] != HEADER_TAG: break
                    self.snapshots.append(Snapshot(count, probe, h))
                    count += 1
                    probe += SNAP_BLOCKS
                except: break

            self.after(0, self._populate_list)
            self.set_status(f"Found {len(self.snapshots)} snapshots")
        threading.Thread(target=task, daemon=True).start()

    def _populate_list(self):
        self.listbox.delete(0, tk.END)
        for s in self.snapshots:
            fmt = FMT_NAME.get(s.header['pixel_format'], "?")
            ts = time.strftime("%Y-%m-%d %H:%M", time.localtime(s.header.get('timestamp', 0)))
            self.listbox.insert(tk.END, f"#{s.idx+1:03d}   {s.header['width']}×{s.header['height']}   {fmt}   {ts}")

    def on_select(self, event=None):
        sel = self.listbox.curselection()
        if not sel: return
        self.current_snap = self.snapshots[sel[0]]
        self.load_preview()

    def load_preview(self):
        def task():
            self.set_status("Loading original image...")
            img = self.load_full_image(self.current_snap)
            if img:
                display = img.resize((900, 600), Image.Resampling.LANCZOS)
                self.current_photo = ImageTk.PhotoImage(display)
                self.after(0, lambda: self.preview_label.configure(image=self.current_photo, text=""))
        threading.Thread(target=task, daemon=True).start()

    def load_full_image(self, snap):
        if snap.image_data: return snap.image_data
        # ... (same loading logic as before)
        hd = rblk(self.drive, snap.block)
        h = snap.header
        nb = (HEADER_SIZE + h['data_size'] + BLOCK_SIZE - 1) // BLOCK_SIZE

        img_bytes = bytearray(hd[HEADER_SIZE:BLOCK_SIZE])
        for i in range(1, nb):
            rem = h['data_size'] - len(img_bytes)
            if rem <= 0: break
            img_bytes.extend(rblk(self.drive, snap.block + i)[:rem])

        img = bytes(img_bytes[:h['data_size']])
        w, hh = h['width'], h['height']

        if h['pixel_format'] == 0:
            arr = cv2.cvtColor(np.frombuffer(img, np.uint8).reshape(hh, w, 2), cv2.COLOR_YUV2BGR_YUY2)
            pil = Image.fromarray(cv2.cvtColor(arr, cv2.COLOR_BGR2RGB))
        else:
            pil = Image.fromarray(np.frombuffer(img, np.uint8).reshape(hh, w))

        snap.image_data = pil
        return pil

    # ====================== AI ENHANCER ======================
    def ai_enhance_current(self):
        if not self.current_snap or not self.current_snap.image_data:
            messagebox.showwarning("Warning", "Load a snapshot first")
            return

        def enhance_task():
            self.set_status("Applying AI-style enhancement + 2x Upscale...")
            try:
                img = self.current_snap.image_data.copy()

                # 1. Convert to OpenCV
                cv_img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

                # 2. Super-resolution (Lanczos + sharpening + detail enhancement)
                # Upscale 2x
                upscaled = cv2.resize(cv_img, None, fx=2, fy=2, interpolation=cv2.INTER_LANCZOS4)

                # 3. Detail enhancement
                sharpen = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
                enhanced = cv2.filter2D(upscaled, -1, sharpen)

                # 4. Light bilateral filter for noise reduction (AI-like)
                enhanced = cv2.bilateralFilter(enhanced, 9, 75, 75)

                # Back to PIL
                pil_enhanced = Image.fromarray(cv2.cvtColor(enhanced, cv2.COLOR_BGR2RGB))

                # Cache it
                self.enhanced_photo = ImageTk.PhotoImage(pil_enhanced.resize((900, 600), Image.Resampling.LANCZOS))

                self.after(0, lambda: self.preview_label.configure(image=self.enhanced_photo))
                self.after(0, lambda: self.set_status("AI Enhancement applied (2x upscale)"))

            except Exception as e:
                self.after(0, lambda: messagebox.showerror("Enhance Error", str(e)))

        threading.Thread(target=enhance_task, daemon=True).start()

    def save_enhanced(self):
        if not self.enhanced_photo or not self.current_snap:
            messagebox.showinfo("Info", "Enhance an image first")
            return

        out_dir = filedialog.askdirectory(title="Save Enhanced Image")
        if not out_dir: return

        try:
            enhanced_pil = self.current_snap.image_data.copy()  # we'll re-enhance for saving
            cv_img = cv2.cvtColor(np.array(enhanced_pil), cv2.COLOR_RGB2BGR)
            up = cv2.resize(cv_img, None, fx=2, fy=2, interpolation=cv2.INTER_LANCZOS4)
            up = cv2.bilateralFilter(cv2.filter2D(up, -1, np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])), 9, 75, 75)

            final = Image.fromarray(cv2.cvtColor(up, cv2.COLOR_BGR2RGB))
            path = os.path.join(out_dir, f"snapshot_{self.current_snap.idx:03d}_AI_2x.png")
            final.save(path)
            messagebox.showinfo("Saved", f"Enhanced image saved:\n{path}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # Other functions (extract, delete, etc.) remain the same as previous version
    def extract_selected(self): ...   # keep from previous
    def delete_selected(self): ... 
    def disconnect_drive(self): ...
    def format_card(self): ...
    def set_status(self, text):
        self.after(0, lambda: self.status.configure(text=text))


if __name__ == "__main__":
    if os.name != 'nt':
        print("Windows only")
        sys.exit(1)
    app = SDVisualizer()
    app.mainloop()
    close_drive()