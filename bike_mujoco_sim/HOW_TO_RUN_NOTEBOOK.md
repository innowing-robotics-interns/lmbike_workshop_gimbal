# How to run the bicycle simulator notebooks

This guide is for someone who has not used Jupyter before. There are five
standalone lessons. Open any one alone. None of them needs another notebook.

| File | What you learn |
| --- | --- |
| `tutorials/01_environment_and_the_fall.ipynb` | MuJoCo window and why the bike falls without balance |
| `tutorials/02_pid_and_the_struggle.ipynb` | PID math in code, and a controller that fights then fails |
| `tutorials/03_tuning_and_balance.ipynb` | Systematic outer-gain tuning until the bike stays up |
| `tutorials/04_teleop_and_disturbance.ipynb` | Joystick / speed as a disturbance to the roll loop |
| `tutorials/05_escaping_the_sandbox.ipynb` | Save YAML and run `python run_sim.py` from the terminal |

Your editable gain panel is `configs/tutorial.yaml`. Paths in the notebooks
are built with `os.path.join`.

You need Python 3.10 or newer. If you do not have it, install it from
[https://www.python.org/downloads/](https://www.python.org/downloads/). On
Windows, turn on **Add python.exe to PATH**.

## 1. Check Python

Open a terminal:

- Windows: press the Windows key, type `PowerShell`, open **Windows PowerShell**.
- macOS: **Terminal** from Applications, Utilities.
- Linux: open **Terminal**.

Windows PowerShell:

```powershell
python --version
```

macOS or Linux:

```bash
python3 --version
```

Use a version of 3.10 or higher.

## 2. Go to the simulator folder

You want the folder that contains `run_sim.py`.

Windows PowerShell:

```powershell
cd C:\Users\YourName\bike-workshop\bike_mujoco_sim
dir run_sim.py
```

macOS or Linux:

```bash
cd ~/bike-workshop/bike_mujoco_sim
ls run_sim.py
```

## 3. Create a virtual environment

```bash
python -m venv .venv
```

On macOS and Linux, if `python` is missing, use `python3`.

## 4. Activate it

Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

If scripts are blocked:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

macOS or Linux:

```bash
source .venv/bin/activate
```

You should see `(.venv)` at the start of the line.

## 5. Install packages

```bash
python -m pip install -r requirements.txt
```

## 6. Register the notebook kernel

```bash
python -m ipykernel install --user --name bike-mujoco-sim --display-name "Bike MuJoCo sim"
```

## 7. Start Jupyter

From the folder that contains `run_sim.py`:

```bash
python -m jupyter lab
```

Or:

```bash
python -m jupyter notebook
```

## 8. Open one lesson

In the file browser open `tutorials`, then open **one** of the five notebooks.
In the kernel menu pick **Bike MuJoCo sim**.

Each lesson is a walkthrough: learning objectives, a glossary, then numbered
steps. Read the step, run the next code cell, and check the result against
the expected outcome before you continue. The last section of each lesson is
**Questions**. Answer those first, then change the marked number and run that
cell again.
Run cells from top to bottom (**Run → Run All Cells**), or one cell at a
time with Shift+Enter.

Cells that open a 3D window sit just before the graph of the same run. The
window cell stays busy until you close the MuJoCo window. Restart the kernel
before a window cell if you already ran a graph cell, because that cell
selects a different renderer.

On macOS, for any cell that opens the MuJoCo window, start Jupyter with:

```bash
mjpython -m jupyter lab
```

## 9. After you have tuned gains

Save your outer PID numbers in `configs/tutorial.yaml`. Then, from an activated
terminal in `bike_mujoco_sim`:

```bash
python run_sim.py --config configs/tutorial.yaml --model new_bike --mode speed-schedule
python run_sim.py --config configs/tutorial.yaml --model new_bike --mode remote
python run_sim.py --config configs/tutorial.yaml --model new_bike --headless --duration 6
```

## Troubleshooting

**`Could not find bike_mujoco_sim`** — start Jupyter from the folder that
contains `run_sim.py`, or from `tutorials` under that folder.

**`No module named sim` or `No module named mujoco`** — wrong kernel. Activate
`.venv`, reinstall requirements, register the kernel again, restart the kernel.

**Windows will not activate** — use `Activate.ps1` above, or
`.venv\Scripts\activate.bat` in Command Prompt.

**Linux EGL / display errors** — install your distro's EGL package if the error
names `libEGL`. Do not set `MUJOCO_GL=egl` on macOS or Windows.

## What was checked

The non-window cells of all five notebooks were executed on Linux. Window cells
were smoke-checked separately. Windows and macOS were not executed on that
machine; the notebooks use `os.path.join` and only force GLFW when opening a
window.
