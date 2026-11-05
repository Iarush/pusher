# Hardware Change Log

Running record of hardware changes, faults, and fixes for the Custom EV test platform (Pico 2, motor driver, encoders). Newest entries first.

---

## 2026-10-06: Upload failure, wiring rework, and motor repair

**Status:** Resolved

### Issue
The computer did not detect the Pico 2 over USB when the driver was plugged in, which made uploading code impossible.

### Actions Taken
1. **Reworked the wiring to analog.** This restored USB detection and uploads. Analog wiring is the working baseline.
2. **Switched the wiring to I2C** and tested it. I2C works.
3. **Adjusted motor alignment.** Made minor changes to the motor mounting.
4. **Resoldered a broken motor joint.**

### Result
- The Pico 2 is detected and accepts uploads.
- The I2C configuration works and has been tested.
- The motor joint is repaired and the motors are realigned.

### Follow-ups
- [ ] Record the root cause of the USB detection failure. The original wiring fault was not identified.
- [ ] Add before/after wiring photos or a diagram.
- [ ] Update [Wiring.MD](Wiring.MD) so the documented pin map matches the current I2C wiring.


### Issue
The Car was not reversing. It would get stuck on the Forwards becasue the stiction duty was at 0. 

### Actions Taken'
Testing different low cruise duties and found the minimum duty to be 0.055. (The duty right before it starts reversing)
### Result
### Follow-ups
---

## Entry Template

```
## YYYY-MM-DD: Short title

**Status:** Resolved | In progress | Open

### Issue
### Actions Taken
### Result
### Follow-ups
```
