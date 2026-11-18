# Systematic Optical Analysis of 6 mm, 8 mm and 10 mm f/8 Lenses

## 1. Objective of the Analysis

The objective of this analysis is to compare different lens configurations for an embedded insect-imaging system.

The camera must capture insects moving through a controlled observation area while preserving enough visual information for:

- insect detection,
- segmentation,
- morphological analysis,
- machine-learning classification,
- and potentially taxonomic identification.

The optical system must therefore balance several parameters simultaneously:

- field of view,
- spatial resolution,
- object magnification,
- depth of field,
- illumination requirements,
- motion blur,
- mechanical tolerance,
- and optical resolution.

The configurations considered here are:

- 6 mm f/8
- 8 mm f/8
- 10 mm f/8

All three configurations use the same aperture so that the main effect of focal length can first be analyzed independently.

---

## 2. Camera Sensor

The imaging system uses a Sony IMX335 sensor.

Relevant characteristics are approximately:

- Resolution: 2592 × 1944 pixels
- Resolution class: approximately 5 MP
- Pixel size: approximately 2 µm
- Optical format: approximately 1/2.8"

The sensor therefore provides a relatively high pixel density.

However, the number of sensor pixels alone does not determine the useful image resolution.

The lens determines how much of the physical scene is projected onto those pixels.

For this reason, an important parameter is:

\[
\text{Object sampling} =
\frac{\text{number of pixels}}{\text{physical field of view}}
\]

This is normally expressed in:

\[
pixels/mm
\]

The higher the value, the more image pixels are available to represent a given insect.

---

## 3. First Parameter: Focal Length

Focal length determines the relationship between:

- the physical area visible by the camera,
- and the apparent size of the insect in the image.

In simple terms:

> Short focal length = larger visible area.

> Long focal length = greater magnification.

Therefore:

| Focal length | Field of view | Magnification |
|---|---|---|
| 6 mm | Large | Low |
| 8 mm | Medium | Medium |
| 10 mm | Small | High |

This relationship creates the first major trade-off.

A large field of view makes the system more tolerant to insect position.

A small field of view increases the number of pixels available on the insect.

---

## 4. Field of View

The field of view defines the physical region observed by the camera.

For a fixed sensor and fixed working distance:

\[
FOV \propto \frac{1}{f}
\]

where:

- \(FOV\) = field of view
- \(f\) = focal length

Therefore, increasing the focal length reduces the field of view.

The relationship can be visualized approximately as:

```text
6 mm

+--------------------------------------+
|                                      |
|              insect                  |
|                                      |
|                                      |
+--------------------------------------+

Large observation area
Small image of the insect
```

```text
8 mm

+------------------------------+
|                              |
|           insect             |
|                              |
+------------------------------+

Intermediate observation area
Intermediate insect size
```

```text
10 mm

+------------------------+
|                        |
|        insect          |
|                        |
+------------------------+

Smaller observation area
Larger image of the insect
```

This means that a 6 mm lens provides more spatial tolerance, while a 10 mm lens provides more image detail.

---

## 5. Spatial Sampling

A useful way to compare lenses is to calculate how many pixels represent one millimeter of the physical object.

The relationship is:

\[
Sampling =
\frac{Image\ Width\ [pixels]}
{Horizontal\ FOV\ [mm]}
\]

For the IMX335:

\[
Image\ Width = 2592\ pixels
\]

As an illustrative example, suppose that at the selected working distance the horizontal field of view is approximately:

| Lens | Example horizontal FOV |
|---|---:|
| 6 mm | 70 mm |
| 8 mm | 52 mm |
| 10 mm | 42 mm |

These values are only used to explain the relationship. The final values must be calculated from the real working distance and lens specifications.

The corresponding spatial sampling is:

### 6 mm

\[
2592 / 70 \approx 37\ pixels/mm
\]

### 8 mm

\[
2592 / 52 \approx 50\ pixels/mm
\]

### 10 mm

\[
2592 / 42 \approx 62\ pixels/mm
\]

Therefore:

| Lens | Approximate sampling |
|---|---:|
| 6 mm | 37 px/mm |
| 8 mm | 50 px/mm |
| 10 mm | 62 px/mm |

Increasing focal length therefore increases the number of pixels available to describe the insect.

---

## 6. Effect on Different Insect Sizes

Using the previous illustrative sampling values:

\[
Object\ pixels =
Object\ size\ [mm] \times Sampling\ [pixels/mm]
\]

The approximate number of pixels across an insect becomes:

| Insect size | 6 mm | 8 mm | 10 mm |
|---:|---:|---:|---:|
| 1 mm | 37 px | 50 px | 62 px |
| 2 mm | 74 px | 100 px | 124 px |
| 5 mm | 185 px | 250 px | 310 px |
| 10 mm | 370 px | 500 px | 620 px |
| 20 mm | 740 px | 1000 px | 1240 px |

This illustrates the advantage of increasing focal length.

For very small insects, the difference becomes particularly relevant.

A small insect captured with a 10 mm lens will generally contain more morphological information than the same insect captured with a 6 mm lens.

However, spatial resolution is not the only design criterion.

---

## 7. Risk of Cropping

Increasing magnification reduces the available observation area.

This creates a second constraint.

An insect does not always pass exactly through the center of the system.

Its trajectory can vary horizontally and vertically.

Large insects can also occupy a significant fraction of the observation area.

The probability of partially cropping the insect therefore increases as focal length increases.

Qualitatively:

| Lens | Cropping risk |
|---|---|
| 6 mm | Low |
| 8 mm | Moderate / Low |
| 10 mm | Higher |

This is particularly important for an autonomous monitoring system because the insect cannot be manually repositioned before every image.

---

## 8. Sensitivity to Mechanical Alignment

The field of view also affects mechanical tolerances.

With a large field of view, small variations in:

- camera position,
- lens position,
- trap assembly,
- insect trajectory,

have relatively little influence.

With a smaller field of view, the same mechanical displacement represents a larger fraction of the image.

Therefore:

| Lens | Mechanical tolerance |
|---|---|
| 6 mm | High |
| 8 mm | Intermediate |
| 10 mm | Lower |

This becomes increasingly important when several identical systems must be manufactured and deployed.

An optical configuration should ideally work reliably even if small assembly differences exist between units.

---

## 9. Magnification vs System Robustness

The focal-length problem can therefore be summarized as two competing objectives.

### Objective A: maximize image information

This favors a longer focal length.

```text
6 mm  -> less magnification
8 mm  -> intermediate magnification
10 mm -> more magnification
```

### Objective B: maximize acquisition robustness

This favors a shorter focal length.

```text
6 mm  -> high tolerance
8 mm  -> intermediate tolerance
10 mm -> lower tolerance
```

The system therefore cannot be optimized using magnification alone.

---

## 10. Aperture

The second major optical parameter is the aperture.

The aperture is described using the f-number:

\[
N = \frac{f}{D}
\]

where:

- \(N\) = f-number
- \(f\) = focal length
- \(D\) = effective aperture diameter

A lower f-number means a larger opening.

For example:

```text
f/2.8  -> large aperture
f/4    -> medium aperture
f/8    -> smaller aperture
f/11   -> even smaller aperture
```

The aperture influences three major properties:

1. amount of light,
2. depth of field,
3. diffraction.

---

## 11. Aperture and Light

A larger aperture allows more light to reach the sensor.

Therefore:

```text
f/2.8 -> more light
f/4   -> less light
f/8   -> significantly less light
```

This normally favors low f-numbers.

However, insect imaging introduces another constraint:

the object can move.

If the camera compensates for low light by increasing exposure time, the insect can move during the exposure.

This produces motion blur.

Therefore, exposure time should remain short.

---

## 12. Depth of Field

Depth of field describes the range of object distances that appear acceptably sharp.

This is especially important because insects are three-dimensional objects.

Different structures may be located at different distances from the camera:

- antennae,
- head,
- thorax,
- abdomen,
- wings,
- legs.

Different insects may also have different body thicknesses.

Additionally, the insect may not always pass through exactly the same vertical position.

A larger depth of field therefore increases the probability that the complete insect remains usable.

---

## 13. Effect of Aperture on Depth of Field

In general:

> Increasing the f-number increases the depth of field.

At a relatively open aperture, only a narrow region may be perfectly focused.

At f/8, the acceptable focus region becomes significantly larger.

This increases tolerance to variations in insect position.

---

## 14. Why Depth of Field Matters More Than Perfect Focus

For an automated system, perfect laboratory focus on one single plane is not enough.

For example, consider a beetle whose thorax is correctly focused but whose legs extend several millimeters closer to the camera.

With a shallow depth of field, some body structures can become blurred even if the central body is correctly focused.

With a larger depth of field, a larger fraction of the complete insect remains acceptably sharp.

The second case contains more usable morphological information.

Therefore, the useful optical quality of the system is not determined only by maximum theoretical sharpness.

It is also determined by how consistently the insect remains sharp under real operating conditions.

---

## 15. Controlled Illumination Changes the Aperture Trade-Off

Using f/8 reduces the amount of incoming light.

However, the imaging system includes controlled artificial illumination.

This changes the normal photographic trade-off.

In a conventional camera, reducing aperture might require:

\[
f/8 \rightarrow longer\ exposure
\]

which could generate motion blur.

In this system, additional illumination can instead be used:

\[
f/8
+
strong\ controlled\ illumination
+
short\ exposure
\]

This makes it possible to obtain:

- increased depth of field,
- low motion blur,
- controlled illumination conditions.

Therefore, the loss of light associated with f/8 is partially compensated by the architecture of the imaging system itself.

---

## 16. Why Not Close the Aperture More?

If depth of field improves with increasing f-number, it may appear logical to use:

- f/11,
- f/16,
- or even higher values.

However, this introduces diffraction.

Diffraction occurs when light passes through a small aperture.

Instead of reproducing a point as a perfect point, the lens produces a small diffraction pattern called an Airy disk.

Its approximate diameter is:

\[
d = 2.44 \lambda N
\]

where:

- \(d\) = Airy disk diameter
- \(\lambda\) = wavelength
- \(N\) = f-number

---

## 17. Diffraction at f/8

For approximately green light:

\[
\lambda \approx 550\ nm
\]

At:

\[
N = 8
\]

the Airy disk diameter is approximately:

\[
d =
2.44 \times 0.55\ \mu m \times 8
\]

\[
d \approx 10.7\ \mu m
\]

The IMX335 pixel size is approximately:

\[
2\ \mu m
\]

Therefore, the theoretical diffraction spot is larger than one individual sensor pixel.

This means that the optical system is already influenced by diffraction at f/8.

Increasing the aperture number further would progressively increase this blur.

For example:

\[
f/11 > f/8
\]

in terms of diffraction blur.

Therefore aperture cannot simply be reduced indefinitely to increase depth of field.

---

## 18. Aperture Trade-Off

The aperture therefore introduces another balance:

| Aperture | Light | Depth of field | Diffraction |
|---|---|---|---|
| f/2.8 | High | Low | Low |
| f/4 | High/Medium | Medium | Low |
| **f/8** | Lower | **High** | Moderate |
| f/11 | Low | Higher | Higher |
| f/16 | Very low | Very high | Significant |

The useful operating point depends on the application.

For this system, depth of field is particularly important because the insect position cannot be perfectly controlled.

At the same time, diffraction and illumination requirements prevent indefinitely increasing the f-number.

Therefore f/8 represents an intermediate operating region.

---

## 19. Focal Length and Aperture Must Be Considered Together

Focal length and aperture solve different problems.

### Focal length mainly controls:

- field of view,
- magnification,
- pixels per insect,
- positioning tolerance.

### Aperture mainly controls:

- depth of field,
- light transmission,
- diffraction.

They should therefore not be evaluated independently when designing the complete imaging system.

The useful optical configuration is the one that provides acceptable performance across all these parameters simultaneously.

---

## 20. System-Level Comparison

The three configurations can now be compared systematically.

| Parameter | 6 mm f/8 | 8 mm f/8 | 10 mm f/8 |
|---|---|---|---|
| Field of view | Large | Medium | Small |
| Magnification | Low | Medium | High |
| Pixels per insect | Lower | Intermediate | Higher |
| Small-insect detail | Lower | Good | Highest |
| Large-insect coverage | Best | Good | More limited |
| Position tolerance | High | Good | Lower |
| Mechanical tolerance | High | Good | Lower |
| Cropping probability | Low | Low/Medium | Higher |
| Depth of field | High | High | High |
| Light transmission | Same f/8 condition | Same | Same |
| Motion-blur compatibility | Good with controlled light | Good with controlled light | Good with controlled light |
| Diffraction condition | Similar | Similar | Similar |
| Overall balance | FOV-oriented | Balanced | Resolution-oriented |

---

## 21. Interpretation of the Three Configurations

### 6 mm f/8

The main advantage of the 6 mm configuration is robustness.

It provides:

- a large observation area,
- high tolerance to insect trajectory,
- low probability of cropping,
- relaxed mechanical alignment.

Its main limitation is lower object sampling.

Small insects occupy fewer pixels, which reduces the amount of morphological information available for classification.

The 6 mm configuration therefore favors:

> observation-area coverage over fine image detail.

### 8 mm f/8

The 8 mm configuration provides an intermediate optical geometry.

Compared with 6 mm, it increases magnification and the number of pixels available on the insect.

Compared with 10 mm, it retains a larger observation area and greater tolerance to insect-position variation.

It therefore provides:

- useful magnification,
- useful field of view,
- good tolerance to mechanical variations,
- relatively low cropping risk,
- high depth of field through the f/8 aperture.

The configuration is therefore located near the center of the main system trade-offs.

### 10 mm f/8

The main advantage of the 10 mm configuration is image sampling.

It provides:

- higher magnification,
- more pixels per millimeter,
- more pixels across small insects.

This can improve the visibility of small morphological structures.

However, it also produces:

- a smaller observation area,
- increased sensitivity to insect trajectory,
- increased sensitivity to camera alignment,
- increased probability of cropping large or displaced insects.

The 10 mm configuration therefore favors:

> image detail over acquisition tolerance.

---

## 22. Systematic Trade-Off Map

The three focal lengths can be represented as:

```text
                     MORE MAGNIFICATION
                            ↑
                            |
                            |
                       10 mm f/8
                            |
                            |
                         8 mm f/8
                            |
                            |
                        6 mm f/8
                            |
                            ↓
                     LARGER FIELD OF VIEW
```

Another representation is:

```text
6 mm                8 mm                 10 mm

Wide FOV            Balanced             High magnification
    <------------------|-------------------->

High tolerance      Balanced             High detail
    <------------------|-------------------->

Low px/mm           Medium px/mm         High px/mm
    <------------------|-------------------->
```

The aperture introduces a separate axis:

```text
Large aperture                           Small aperture

f/2.8     f/4              f/8       f/11       f/16
  |--------|----------------|-----------|-----------|

More light                              More DOF
Less DOF                                Less light
Less diffraction                        More diffraction
```

The complete system therefore requires selecting an operating point in both dimensions.

---

## 23. Main Engineering Trade-Offs

The optical design can be reduced to four principal competing quantities:

\[
\boxed{\text{Field of View}}
\]

versus

\[
\boxed{\text{Spatial Sampling}}
\]

and

\[
\boxed{\text{Depth of Field}}
\]

versus

\[
\boxed{\text{Light / Diffraction}}
\]

These four quantities cannot all be maximized simultaneously.

For example:

Increasing focal length:

\[
\text{More pixels/object}
\]

but:

\[
\text{Smaller FOV}
\]

Increasing f-number:

\[
\text{Larger DOF}
\]

but:

\[
\text{Less light + more diffraction}
\]

The optical design is therefore fundamentally a multi-parameter optimization problem.

---

## 24. Result of the Comparative Analysis

The three configurations correspond to different optimization priorities:

### 6 mm f/8

Optimizes primarily for:

**coverage and positioning tolerance**

### 8 mm f/8

Provides an intermediate operating point between:

**coverage and spatial resolution**

### 10 mm f/8

Optimizes primarily for:

**magnification and small-object sampling**

The f/8 aperture places all three configurations in a regime where depth of field is prioritized over maximum light transmission, which is compatible with the use of controlled external illumination.

From a system perspective, the 8 mm f/8 configuration lies between the two more specialized alternatives:

```text
6 mm f/8                8 mm f/8                 10 mm f/8

Coverage                System balance           Detail
Tolerance      <------  COMPROMISE  ------>      Magnification
```

It does not maximize any single optical parameter.

Instead, it provides an intermediate balance between the parameters that directly affect autonomous insect image acquisition:

- observation-area coverage,
- insect sampling,
- depth of field,
- positioning tolerance,
- mechanical robustness,
- illumination requirements,
- and usable image detail.

This makes the comparison a system-level optical trade-off rather than a focal-length-only selection problem.
