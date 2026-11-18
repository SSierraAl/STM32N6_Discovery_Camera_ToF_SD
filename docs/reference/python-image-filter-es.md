# Image_filter.py — Documentación del algoritmo de detección de objetos

> Filtro de imágenes "vacías vs. algo nuevo" para la cámara de insectos STM32N6
> (CMW-IMX335, 1296×972, YUV422). El archivo Python es una **plantilla 1:1**
> para el port a C: toda la parte CORE usa solo aritmética entera (sumas,
> comparaciones y corrimientos de bits) — sin floats, sin librerías, sin RAM
> extra.

---

## 1. ¿Qué hace?

Dada una foto, decide en **milisegundos** si contiene algo nuevo (un insecto
minúsculo) o si es un fondo vacío (con el cambio de iluminación que sea).

- **Modo `batch`** → análisis offline de una carpeta completa. Construye un
  fondo de referencia robusto (mediana por sesión), calcula todas las métricas
  de cada imagen, las rankea y marca las diferentes.
- **Modo `live`** → **simulación exacta del pipeline que correrá en el MCU**:
  procesa las imágenes en orden cronológico, mantiene un fondo con EMA entera,
  clasifica cada frame y recalibra automáticamente. Muestra qué frames el MCU
  habría guardado.

Ambos modos comparten el mismo núcleo (`compute_metrics` + `classify`).

---

## 2. Uso

```bat
:: Ejecución por defecto (ambos modos, marca las 20 mejores)
python Image_filter.py --mode both --top 20

:: Solo la simulación del pipeline del MCU
python Image_filter.py --mode live

:: Solo el análisis offline
python Image_filter.py --mode batch

:: Otra carpeta de fotos, sin generar imágenes marcadas
python Image_filter.py --folder D:\otras_fotos --no-mark

:: Marcar más imágenes
python Image_filter.py --mode both --top 50
```

### Carpetas de ENTRADA / SALIDA

| Rol | Ruta por defecto | Contenido |
|---|---|---|
| **ENTRADA** | `C:\Users\ssierra\Downloads\test_photos` | PNGs 1296×972 (raw YUV422 convertido). El timestamp se lee del nombre: `IMG_YYYYMMDD_HHMMSSZ_seq_seq.png` |
| **SALIDA batch** | `C:\Users\ssierra\Downloads\test_photos_marked_batch\` | ver abajo |
| **SALIDA live** | `C:\Users\ssierra\Downloads\test_photos_marked_live\` | ver abajo |

La carpeta de salida siempre es `<carpeta_de_entrada>_marked_batch` o
`..._marked_live` (hermana de la carpeta de entrada; **los originales nunca se
modifican**).

### Archivos de salida

| Prefijo / archivo | Qué es |
|---|---|
| `MARKED_<orig>.png` | Copia de la imagen con **borde rojo**, texto amarillo con las métricas y **rectángulo rojo sobre la zona donde está el objeto** |
| `DIFF_<orig>.png` | Imagen en escala de grises de la **diferencia residual amplificada ×6** (el objeto se ve brillante sobre negro) |
| `CROP_<orig>.png` | Recorte apretado (±60 px) de la bounding box del objeto — la forma más rápida de ver el insecto sin abrir el PNG de 2 MB |
| `insect_filter_batch.csv` / `insect_filter_live.csv` | Tabla completa de TODAS las métricas de TODAS las imágenes (columnas explicadas en la sección 8) |

---

## 3. El algoritmo CORE, paso a paso

```
  IMAGEN 1296×972 (RGB u YUV422)
          │
          ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 0 — Luma                               │
  │  Y = (77·R + 150·G + 29·B) >> 8            │
  │  (en el MCU la Y ya viene de DCMIPP)        │
  └─────────────────────────────────────────────┘
          │
          ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 1 — Downsample 8×8 (promedio de 64 px) │
  │  Rejilla 121×162 = 19 602 enteros 0..255    │
  │  RAM en MCU: ~20 KB                         │
  └─────────────────────────────────────────────┘
          │   g (frame actual)         bg (fondo calibrado)
          ▼                            ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 2 — Quitar el cambio global de luz     │
  │  gs = media(g) − media(bg)                  │
  │  resid = g − bg − gs                        │
  │  → un cambio de iluminación NO parece       │
  │    objeto; solo lo local queda en resid     │
  └─────────────────────────────────────────────┘
          │
          ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 3 — Métricas sobre |resid|             │
  │  • Histograma 16 bins (forma del cambio)    │
  │  • Pixels cambiados: |resid| ≥ T_PIXEL      │
  │  • Bloques calientes 32×32:                 │
  │      débil  ≥ T_BLOCK (10)                  │
  │      fuerte ≥ 2·T_BLOCK (20)                │
  │  • Bounding box de los bloques fuertes      │
  │  • Máximo |resid|                           │
  └─────────────────────────────────────────────┘
          │
          ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 4 — Clasificación (solo comparaciones) │
  │  CLEAN / DIFFERENT / BROAD                  │
  └─────────────────────────────────────────────┘
          │
          ▼
  ┌─────────────────────────────────────────────┐
  │ PASO 5 — Actualizar el fondo bg             │
  │  según la etiqueta (sección 5)              │
  └─────────────────────────────────────────────┘
```

### 3.1 ¿Por qué downsample 8×8?
Un insecto "super tiny" mide ~20–100 px. Promediar bloques 8×8 reduce el
ruido del sensor (los valores aleatorios de 1–2 LSB se cancelan) y deja el
objeto con un tamaño cómodo de analizar (3–12 celdas). Toda la detección se
hace sobre 19.6K enteros, no sobre 1.26M de pixels.

### 3.2 ¿Por qué quitar el shift global (gs)?
La iluminación (LED WS2812, día/noche, exposure auto) cambia la **media** de
la imagen completa. Si no se resta, cada cambio de luz parece que "todo
cambió". Al restar `gs`, el residual solo conserva lo que cambió **de forma
espacial** — que es exactamente lo que es un objeto.

### 3.3 Bloques calientes (hot blocks)
El residual se promedia en bloques de 4×4 celdas del downsample = **32×32 px
originales** (1200 bloques por imagen, rejilla 30×40):

- **débil**: media del bloque ≥ `T_BLOCK` (10) → el borde/halo del objeto
- **fuerte**: media ≥ `2·T_BLOCK` (20) → el **núcleo** del objeto

Lo fuerte decide dos cosas: (a) si el cambio es **localizado** (objeto) o
**extendido** (iluminación), y (b) la **bounding box** (se calcula solo con
bloques fuertes, para que el halo no estire la caja).

---

## 4. Clasificación (la máquina de estados)

Es una cadena de `if` con comparaciones enteras — se porta a C sin cambios:

```
                       ┌────────────────────────────────────────────┐
                       │  ¿changed_bps ≤ 25  Y  max_diff ≤ 16 ?     │
                       │  (≤0.25% px cambiados, contraste mínimo)   │
                       └──────────────┬─────────────────────────────┘
                          SÍ          │          NO
                 ┌────────────────────┘          └──────────────────┐
                 ▼                                                  ▼
            ┌─────────┐                                  ┌────────────────────────┐
            │  CLEAN  │                                  │ ¿0 bloques calientes   │
            │ (vacía) │                                  │  Y changed < 0.05% ?   │
            └─────────┘                                  └──────┬─────────┬───────┘
                                       SÍ (ruido/ghost)         │         │ NO
                                                                ▼         ▼
                                                           ┌────────┐  ┌─────────────────────────┐
                                                           │ CLEAN  │  │ ¿bloques FUERTES ≤ 40 ? │
                                                           └────────┘  └──────┬──────────┬───────┘
                                                       SÍ (objeto local)      │          │ NO
                                                                              ▼          ▼
                                                                        ┌───────────┐ ┌────────┐
                                                                        │ DIFFERENT │ │ BROAD  │
                                                                        │  → GUARDAR│ │→ ver 5 │
                                                                        └───────────┘ └────────┘
```

- **CLEAN** → la imagen es el fondo: no se guarda.
- **DIFFERENT** → hay un objeto localizado (insecto): **se guarda la foto**.
- **BROAD** → el cambio cubre toda la escena (iluminación, o un objeto
  enorme). En modo live: se guarda **solo el PRIMER frame** del evento
  ("event start") y los siguientes disparan recalibración rápida (`ENV-CAL`).

### Histéresis (anti-parpadeo)
Mientras racha un objeto, el umbral CLEAN baja de 25 a `HYS_BPS` (12) bps:
un insecto que se mueve despacio no hace que el sistema parpadee
DIFFERENT→CLEAN→DIFFERENT.

---

## 5. Fondo (background) y calibración

`bg` es la misma rejilla 121×162 que el frame. Cómo se actualiza según la
etiqueta:

| Situación | Actualización | Efecto |
|---|---|---|
| **CLEAN** | EMA completa: `bg += (g − bg) >> 4` (α = 1/16) | La calibración "periódica" continua: el fondo sigue la deriva lenta de la luz |
| **CLEAN × 300 seguidas** | **Snap total**: `bg = g` | Re-calibración periódica forzada (`CAL_EVERY_CLEAN`) contra derivas muy lentas |
| **DIFFERENT** | EMA **enmascarada**: `bg += (g − bg) >> 4` **solo donde NO hay objeto** (fuera de la máscara de bloques calientes) | El fondo sigue el escenario alrededor del insecto, pero NO absorbe al insecto mientras está ahí |
| **BROAD (primer frame)** | nada (se guarda el frame) | Se conserva la evidencia del evento |
| **BROAD (frames siguientes)** | **Snap rápido**: 2 pases de `bg += (g − bg) >> 2` (α = 1/4) | Recalibración por cambio de iluminación: converge en ~3 frames |
| **STATIC-CAL** | `bg[máscara] = g[máscara]` (absorber la zona) | Un objeto que lleva ≥ 4 frames **estático en el mismo lugar** ya es parte de la escena → se absorbe y deja de guardarse |

### ¿Qué es STATIC-CAL y por qué existe?
Si algo (una mota, una pata de un bote, un insecto muerto quieto) produce el
mismo cambio localizado en el mismo lugar frame tras frame, guardar 40 fotos
idénticas no aporta nada. Después de `ABSORB_STATIC` (4) frames iguales, se
incorpora al fondo. **Si luego se mueve o se va, se vuelve a detectar**
(al moverse cambia la posición; al irse, el "ghost" se disuelve con la EMA
enmascarada en ~16 frames).


---

## 6. Parámetros (los "knobs")

Todos están en el bloque `# TUNABLE PARAMETERS` al inicio de `Image_filter.py`
(y deben vivir en `app_config.h` en el port a C).

### 6.1 Geometría del análisis

| Parámetro | Valor | Qué controla |
|---|---|---|
| `DS` | `8` | Tamaño del bloque de downsample (px originales). 8×8 = 64 px → rejilla 121×162 |
| `HOT_DS` | `4` | Bloque "caliente" = 4×4 celdas del downsample = **32×32 px** originales. Tamaño mínimo que el detector puede localizar |
| `NHIST` | `16` | Bins del histograma de \|diferencia\| (bin 0 = 0..7, el resto de 4 en 4 hasta 255) |

### 6.2 Umbrales de detección

| Parámetro | Valor | Qué controla | Si lo bajes… | Si lo subes… |
|---|---|---|---|---|
| `T_PIXEL` | `24` | Contraste mínimo de un pixel del residual para contar como "cambiado" | detecta insectos más tenues, pero más ruido del sensor pasa | puedes perder insectos pequeños/tenues |
| `T_BLOCK` | `10` | Media mínima de un bloque 32×32 para ser "caliente débil" | el objeto se localiza con más bloques (caja más grande) | objetos pequeños pueden no formar bloque caliente → el guard-ruido los mata |
| `CLEAN_MAX_BPS` | `25` | Umbral CLEAN: ≤ 0.25 % de pixels cambiados (bps = por 10 000) | casi todo se considera "con algo" | imágenes con insecto diminuto podrían marcarse CLEAN |
| `CLEAN_MAX_DIFF` | `16` | Umbral CLEAN: máximo residual ≤ 16 LSB | más sensible | menos sensible |
| `MIN_OBJECT_BPS` | `5` | Mínimo de cambio (0.05 %) para que cuente como objeto (y mata el ruido) | el ruido del sensor genera falsos positivos | objetos de 1 bloque muy tenue podrían no contarse |
| `MAX_HOT_OBJECT` | `40` | Bloques FUERTES máximos para que sea "objeto localizado" (si no → BROAD) | más cosas se tratan como objetos grandes | objetos grandes (un insecto grande cubriendo ~10 % de la imagen) pasan a BROAD |
| `HYS_BPS` | `12` | Umbral CLEAN más laxo durante una racha de objeto (histéresis) | el objeto "pegado" dura más frames | parpadeo al final del evento |

### 6.3 Velocidad de calibración del fondo

| Parámetro | Valor | Qué controla |
|---|---|---|
| `EMA_SHIFT` | `4` | Velocidad de la EMA en frames CLEAN: `bg += (g−bg) >> 4` (α = 1/16). Menor = fondo más lento (robusto), mayor = se adapta más rápido a la luz |
| `SNAP_SHIFT` | `2` | Velocidad del snap en BROAD/iluminación: `bg += (g−bg) >> 2` (α = 1/4), se aplican 2 pases por frame |
| `CAL_EVERY_CLEAN` | `300` | Cada 300 frames CLEAN seguidos → **re-calibración completa** (`bg = g`). Protege contra deriva muy lenta de la luz que la EMA no alcanza |
| `ABSORB_STATIC` | `4` | Frames consecutivos con el mismo cambio en el mismo lugar antes de absorberlo al fondo (STATIC-CAL) |
| `SESSION_GAP_S` | `60` | Hueco de tiempo (s) entre fotos que define "nueva sesión" (solo modo batch) |
| `GLOBAL_MEAN` | `8` | Shift medio mínimo para mencionar "cambio de iluminación" en las notas (informativo) |
| `MARK_MIN_BPS` | `2` | Modo batch: solo se marcan imágenes con ≥ este cambio |

### 6.4 Cómo "oír" si un parámetro está mal

| Síntoma | Causa probable | Ajuste |
|---|---|---|
| Guarda fotos vacías por el ruido del sensor | `T_PIXEL`/`T_BLOCK` bajos o `MIN_OBJECT_BPS` bajo | sube `MIN_OBJECT_BPS` a 8–10 |
| Se pierde el insecto diminuto | `T_PIXEL` o `T_BLOCK` altos | bájalos (18 / 8) y revisa `MIN_OBJECT_BPS` |
| Un cambio de LED genera 2–3 fotos guardadas | normal (event start) — si molesta | baje `MAX_HOT_OBJECT` o guarde el primer BROAD también con snap |
| El fondo "perseguida" a un insecto quieto y lo olvida | `ABSORB_STATIC` pequeño o `EMA_SHIFT` grande | sube `ABSORB_STATIC` a 8–10 |
| Tras irse el insecto quedan frames de "ghost" | la EMA enmascarada es lenta | es normal ~10–16 frames; baja `EMA_SHIFT` a 3 si molesta |

---

## 7. Métricas que se calculan por imagen (y por qué)

| Métrica (CSV) | Cálculo | Dónde se usa |
|---|---|---|
| `mean_y` | Media de la luma del frame | Diagnóstico (nivel de exposición) |
| `std_y` | Desviación típica de la luma | Diagnóstico (textura/contraste de la escena) |
| `global_shift` (`gs`) | `media(frame) − media(bg)` | Restarlo antes de medir el residual; notas de iluminación |
| `max_diff` | Máximo del residual \|g−bg−gs\| | Umbral CLEAN; contraste del objeto |
| `changed` / `changed_bps` | Pixels del residual > `T_PIXEL` (y en bps) | Umbral CLEAN; guard-ruido; tamaño del objeto |
| `n_hot` / `n_hot_strong` | Bloques 32×32 débiles / fuertes | Localizado vs BROAD; bounding box (fuertes) |
| `bbox_x0..y1` | Caja de los bloques fuertes (px originales) | Dibujo en `MARKED_*`; recorte `CROP_*` |
| `hist_peak` / `hist_peak_bps` | Bin dominante del histograma de 16 bins y su peso % | Diagnóstico de la "forma" del cambio |
| `rgb_hist_dist` | Distancia L1 normalizada (0–100 %) del histograma RGB 24 bins vs. referencia | Detecta cambio de **color/balance** (LED vs. luz día) |
| `color_cr` / `color_cb` | `media(R)−media(Y)`, `media(B)−media(Y)` | Deriva de tinte (diagnóstico) |
| `score` | `0.45·min(1, bps/400) + 0.35·maxd/255 + 0.20·min(1, fuertes/8)` (×0.25 si BROAD) | Solo para **ordenar/reporte**, NO para clasificar |


---

## 8. Resultados sobre las fotos de prueba (233 imágenes)

Ejecutado el 25/09/2026 sobre `C:\Users\ssierra\Downloads\test_photos`
(22 sesiones de capturas, varias condiciones de luz distintas):

| Métrica | Modo batch | Modo live (simulación MCU) |
|---|---|---|
| Imágenes "diferentes" | 29 objetos + 17 eventos amplios | **41 guardadas de 233** (82 % de ahorro) |
| CLEAN | 187 | 159 |
| Recalibraciones del fondo | — | 32 (27 por iluminación BROAD + 5 absorciones STATIC) |
| Errores (falsos positivos) | — | 0 sobre los dos eventos reales; solo los "event start" de cambios de luz (1 foto por cambio) |

**Evento 1 — 23/09 13:59:46–14:02:08** (ids 0730–0792): objeto grande que
aparece en el centro de la imagen y se mueve, caja ~(256,224)–(895,831).
Frames más fuertes: 0790, 0791, 0792, 0745, 0746, 0742, 0754–0756.

**Evento 2 — 24/09 17:13:41–17:15:24** (ids 2553–2592): el **insecto diminuto**:
entra por arriba (caja 64×96 px) → se mueve a la zona inferior-izquierda
(2557–2560) → llega al borde izquierdo (2565–2568) → se queda quieto en la
esquina **(0,128)–(31,159)**: solo **1–2 bloques calientes, contraste 54–85 LSB,
0.1–0.12 % de pixels** (2569–2580) → se va. Detectado en todos los tramos.

Las demás ~209 fotos son fondo vacío o variación de iluminación → filtradas.

---

## 9. Port a STM32N6 (mapa Python → C)

Todo el CORE es portable tal cual. Correspondencia:

| Python (`Image_filter.py`) | C en el firmware | Notas |
|---|---|---|
| `to_luma()` | — | En el MCU la Y del YUV422 ya está en el buffer de DCMIPP (stride 2: `buf[2·i]`). Este paso solo existe porque los PNGs de prueba son RGB |
| `downsample()` | `IF_Downsample8x8(y_plane, grid)` | 1 pasada de 1.26M pixels: suma acumulada por bloque + `>> 6`. ~5 ops/pixel |
| `diff_histogram()` | `IF_Hist16(resid, hist)` | 16 contadores `uint16` |
| `hot_blocks()` | `IF_HotBlocks(resid, …)` | 1ª pasada sobre la rejilla → rejilla 30×40 de medias (`>> 4`); 2ª pasada: conteos fuertes/débiles, bbox y máscara (1 bit por celda) |
| `compute_metrics()` | `IF_Metrics(g, bg, m)` | Una sola pasada sobre 19 602 celdas: suma, gs, residual, `changed`, max, histograma; luego la rejilla 30×40 |
| `classify()` | `IF_Classify(m)` | La cadena de `if` de la sección 4, idéntica |
| EMA/snap/máscara (sección 5) | `IF_UpdateBg(bg, g, mask, modo)` | `bg += (g−bg) >> SHIFT` con/`sin` máscara |
| `run_live()` (bucle) | callback del **frame completo de DCMIPP** (evento de fin de frame, tras el warm-up) | Decide **antes** de empezar la escritura SD |

### Presupuesto de recursos en el MCU

| Recurso | Tamaño |
|---|---|
| Rejilla del frame `g` (uint8, 0..255) | 19 602 B ≈ **20 KB** (PSRAM) |
| Fondo `bg` (uint8, 0..255) | ≈ **20 KB** (PSRAM) |
| Medias de bloques (uint8/int16, 30×40) | 0.6–2.4 KB |
| Máscara (1 bit/celda) | 2.5 KB |
| Contadores (hist 16, bps, bbox, estado) | < 1 KB (SRAM) |
| Tiempo de cómputo | 1.26M ops de downsample + 19.6K ops de métricas → **< 5 ms** a 300 MHz (la resta `g−bg` se hace en int16 en registro, sin buffer) |

### El gran ahorro
La decisión se toma **antes** de escribir a SD. Según `PROJECT_DOCUMENTATION.md`
la escritura de un frame 1296×972 tarda **700–900 ms**: descartar las fotos
vacías ahí ahorra ese tiempo completo en cada frame filtrado y llena menos
la tarjeta.

Flujo propuesto dentro de la app:

```
Fin de frame (DCMIPP) → IF_Downsample8x8 → IF_Metrics → IF_Classify
   ├─ CLEAN / ENV-CAL / STATIC-CAL → actualizar bg → NO escribir SD
   └─ DIFFERENT → actualizar bg enmascarado → escribir SD (flujo actual)
```

---

## 10. Estructura del código (`Image_filter.py`)

| Bloque / función | Líneas aprox. | Rol |
|---|---|---|
| `# TUNABLE PARAMETERS` | inicio | Todos los parámetros (sección 6) |
| `FrameMetrics` | ~90 | Estructura de datos de una imagen (misma que la de C) |
| `to_luma / downsample / diff_histogram / hot_blocks / rgb_hist24` | ~115–185 | **CORE aritmético — se porta 1:1** |
| `compute_metrics` | ~190 | Ensambla las métricas |
| `classify` | ~230 | **La máquina de estados — se porta 1:1** |
| `object_score` | ~245 | Score de reporte (no portar) |
| `load_rgb / parse_ts / write_csv / mark_image / hot_ascii` | ~260–320 | I/O, anotación y diagnóstico (solo Python) |
| `run_batch` | ~345 | Modo offline (solo Python) |
| `run_live` | ~440 | **Simulación del pipeline MCU — lógica que se porta** |
| `main` | ~620 | `argparse` |

### Limitaciones conocidas
1. **Objeto estático ≥ 4 frames** → se absorbe (diseño, sección 5). Si se
   quisiera seguir guardándolo, sube `ABSORB_STATIC` (20–30) o ponlo a 0 para
   desactivarlo (el ghost se disuelve solo por la EMA enmascarada).
2. **Objeto que cubre > ~3 % de la imagen** (> 40 bloques fuertes) → se trata
   como BROAD (solo se guarda el primer frame). Ajusta `MAX_HOT_OBJECT`.
3. **Dos insectos muy juntos** se ven como un solo blob (una sola caja).
4. El modo `batch` asume carpeta estática con timestamps en el nombre; el
   `live` asume fotos ya ordenadas cronológicamente (en el MCU viene solo).
5. Los tiempos del live incluyen el **decodificado PNG** (~100 ms); en el MCU
   el coste real es el del CORE: unos pocos ms por frame.

