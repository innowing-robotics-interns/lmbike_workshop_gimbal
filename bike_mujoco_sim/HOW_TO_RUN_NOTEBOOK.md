# How to run the notebook server

After you have opened `lmbike_workshop_gimbal` in your IDE, open a terminal in that window. In VS Code, choose **Terminal → New Terminal**, or press `` Ctrl+` `` (on macOS, `` Control+` ``). The terminal starts in that folder. In another IDE, open its terminal the same way.

Jupyter has to start from `bike_mujoco_sim`, the folder that contains `run_sim.py`. Use the project's virtual environment so the notebooks see the required packages (MuJoCo, matplotlib, and PyYAML).

First-time package install is in `tutorials/00_setup_and_install.ipynb`. This page is only how to start the server after that environment exists.

## Linux

In that terminal:

```bash
cd bike_mujoco_sim
.venv/bin/python -m jupyter lab
```

`python3 -m jupyter lab` uses the system Python. That Python does not have JupyterLab, and it stops with `Jupyter command jupyter-lab not found`.

## macOS

In that terminal:

```bash
cd bike_mujoco_sim
.venv/bin/python -m jupyter lab
```

A cell that opens the MuJoCo window needs this command from that same folder:

```bash
mjpython -m jupyter lab
```

## Windows

In that terminal:

```powershell
cd bike_mujoco_sim
.\.venv\Scripts\Activate.ps1
python -m jupyter lab
```

## After the server starts

1. The terminal prints a local address, usually `http://127.0.0.1:8888/lab`.
2. Open that address in a browser if it does not open on its own.
3. Open `tutorials/00_setup_and_install.ipynb` once, then `tutorials/01_environment_and_the_fall.ipynb`.
4. In the kernel menu, pick **Bike MuJoCo sim**, or the Python inside `.venv`.
5. Stop the server with Ctrl+C in that same terminal.

`.venv/bin/python -m jupyter notebook` starts the older notebook interface from the same folder. The lessons are the same.
