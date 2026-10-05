# How to run the notebook server

Start Jupyter from the `bike_mujoco_sim` folder, the one that contains `run_sim.py`. Use the project's virtual environment so the notebooks see MuJoCo, matplotlib, and PyYAML.

First-time package install is in `tutorials/00_setup_and_install.ipynb`. This page is only how to start the server after that environment exists.

## Linux or macOS

```bash
cd path/to/bike_mujoco_sim
source .venv/bin/activate
python -m jupyter lab
```

`python3 -m jupyter lab` without activating `.venv` uses the system Python. That Python does not have JupyterLab, and the command ends with `Jupyter command jupyter-lab not found`. Use the environment above, or call it directly:

```bash
cd path/to/bike_mujoco_sim
.venv/bin/python -m jupyter lab
```

On macOS, a cell that opens the MuJoCo window needs:

```bash
mjpython -m jupyter lab
```

## Windows PowerShell

```powershell
cd path\to\bike_mujoco_sim
.\.venv\Scripts\Activate.ps1
python -m jupyter lab
```

## After the server starts

1. The terminal prints a local address, usually `http://127.0.0.1:8888/lab`.
2. Open that address in a browser if it does not open on its own.
3. Open `tutorials/00_setup_and_install.ipynb` once, then `tutorials/01_environment_and_the_fall.ipynb`.
4. In the kernel menu, pick **Bike MuJoCo sim**, or the Python inside `.venv`.
5. Stop the server with Ctrl+C in that same terminal.

`python -m jupyter notebook` starts the older notebook interface from the same folder. The lessons are the same.
