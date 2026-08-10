# ExoVest — Python Receiver

Receives IMU orientation data from the ExoVest Main microcontroller over **BLE**
(Nordic UART Service), decodes the 100-byte `MasterPacket`, and renders a live
3D orientation view of all 10 IMUs.

- **Firmware side:** `Arduino Code/ImuDataCode/MainMCCode/MainMCcode.ino`
  advertises as **`ExoVest`** and notifies on TX characteristic
  `6e400003-b5a3-f393-e0a9-e50e24dcca9e`.
- **Wire format:** 100 bytes, little-endian, `struct` format `"<5I 16h 16h 8h"`.
  Quaternion components are Q14 fixed-point int16 → divide by `16384.0` for float.

---

## Setup (every teammate does this once, per machine)

> **Important:** the `.venv/` folder is **NOT** in git (it's gitignored on purpose).
> Virtual environments are not portable — they hold absolute paths and
> platform-specific binaries. Everyone builds their own from `requirements.txt`.

### Prerequisites
- **Python 3.9+** installed on your machine.
  - macOS: `python3 --version` (Apple's built-in `/usr/bin/python3` works, or install from python.org).
  - Windows: install from [python.org](https://www.python.org/downloads/) and check "Add Python to PATH".
  - Linux: `sudo apt install python3 python3-venv python3-pip` (or your distro's equivalent).

### 1. Clone and enter the receiver folder
```bash
git clone <your-repo-url>
cd ExoVest/PythonReceiver     # adjust to wherever the repo lands
```

### 2. Create the virtual environment
```bash
# macOS / Linux
python3 -m venv .venv

# Windows (use the py launcher if `python` isn't found)
python -m venv .venv
```
This makes a local, disposable `.venv/` folder — an isolated Python just for this project.
You can delete it anytime and recreate it; nothing important lives only inside it.

### 3. Activate it
```bash
# macOS / Linux
source .venv/bin/activate

# Windows (PowerShell)
.venv\Scripts\Activate.ps1

# Windows (cmd)
.venv\Scripts\activate.bat
```
Your shell prompt should now show `(.venv)`.

### 4. Install dependencies
```bash
pip install -r requirements.txt
```
Installs the exact pinned versions (bleak, numpy, pyqtgraph, PyQt6) into *your* `.venv/`.

### 5. (macOS only) Grant Bluetooth permission
The first BLE scan will prompt for Bluetooth access, or you may need to enable it
manually: **System Settings → Privacy & Security → Bluetooth** → allow your
terminal app (Terminal / iTerm / VS Code).

---

## Running

> Scripts are added as the build progresses. Phase 0 (environment) is complete.

```bash
# (planned) raw BLE link check — prints incoming 100-byte packets
python main.py --dump

# (planned) live 3D orientation viewer
python main.py
```

To leave the venv when you're done: `deactivate`.

---

## How dependencies work here (FYI)

| Thing | In git? | Notes |
|-------|---------|-------|
| Your code (`*.py`), `README.md`, `requirements.txt` | ✅ yes | This is what you share. |
| `requirements.txt` | ✅ yes | The recipe — pins exact package versions. |
| `.venv/` (installed packages) | ❌ no | Gitignored. Each person rebuilds it locally. |
| Python interpreter itself | ❌ no | Must be installed on each machine separately. |

If you add a new dependency, install it in your venv **and** update the recipe:
```bash
pip install <package>
pip freeze > requirements.txt
```
Then commit the updated `requirements.txt` so teammates get it too.
