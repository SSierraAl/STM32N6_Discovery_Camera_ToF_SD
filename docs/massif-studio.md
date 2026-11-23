# MASSIF Studio

MASSIF Studio is a separate native desktop viewer for the STM32N6 capture
workflow. It uses the same Python environment and SD backend as
`SD_Image_Viewer.py` and leaves that launcher available.

Run from the repository root in an elevated PowerShell:

```powershell
python .\MASSIF_Studio.py
```

1. Select the capture card and choose **Scan SD**. The library shows capture
   number, resolution, date, time, category and filename. Date and time are
   shown in UTC as YYYY-MM-DD and HH:mm. A dash marks an unavailable capture
   timestamp. The category comes from the capture metadata.
   Hover over a timestamp for its full value, or drag the divider to widen
   the library.
2. Filter by typing in the library search field. Select a row to preview it.
   Use Ctrl or Shift to select several rows. **Export selected** uses those
   rows; **Export all** uses the entire scan even when a filter is active.
3. Scroll to zoom, drag to pan, and double-click to fit. **Save PNG** (Ctrl+S)
   exports the current preview at its capture resolution. It preserves the
   displayed appearance regardless of screen zoom.
4. **Delete selected** and **Reset SD catalog** use the existing Windows volume
   lock and confirmation prompts. Reset clears the scanned captures and
   verifies the journal reset; it does not format the filesystem. A drive
   change or drive refresh clears the scan, so scan again before editing
   the card.

**Original contrast** and **Noise reduction: Off** are the startup choices.
**Mild contrast** and **Color + luminance grain reduction** remain available
independently. The established color profile is always applied, with chroma
`0.40` and white balance `1.0` from the current repository configuration.
The existing image metadata and export suffixes are preserved.

The catalog holds metadata only. Preview requests share a single worker,
which reads and renders the latest pending selection without accumulating a
queue during rapid navigation. A bounded 96 MiB cache stores decoded images
and recent renders; the current preview and in-flight worker buffers are
separate. Preview processing and PNG writes run off the GUI thread. No new
dependencies, firmware changes, or SD format changes are required.

The masthead uses `Logos/Massif_logo_white.png` if present, with a text-only
fallback for checkouts without the logo.
