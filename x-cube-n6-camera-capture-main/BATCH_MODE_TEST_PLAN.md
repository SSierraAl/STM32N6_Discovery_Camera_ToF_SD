# BATCH MODE Test Plan — ToF + Camera Coexistence

## What Changed (I2C Restore After Camera)

After camera warmup completes, we restore I2C1 timing for ToF coexistence:
- Set `I2C1->TIMINGR = 0x00401242` (100kHz standard mode for ToF)
- Enable I2C peripheral (`I2C1->CR1 |= I2C_CR1_PE`)
- Verify ToF device ID read (`0x52` address, `0x010F` register, expect `0xF4`)

**Diagnostic print added** so you can see if I2C restore worked:
- `[CAM] I2C restored — ToF alive (0xF4)` = GREEN = ToF can communicate
- `[CAM] I2C restore failed — ToF unreachable (read=0x00)` = RED = I2C still broken

## Step-by-Step Test

### 1. Flash and Observe Boot Sequence

Expected output (with colors in T