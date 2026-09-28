# Bike Project Local

Local MuJoCo workshop for a planar bicycle that has to balance itself.

This repository is an Origin git repo, not GitHub. Commands such as
`git fetch origin pull/3/head:mujoco-workshop` look for a GitHub pull-request
ref that does not exist here, so they fail with `couldn't find remote ref`.
The workshop lives on `main` under `workshop/mujoco_balance`.

## Run the balancer

Python 3.12+ with `python3-venv` (Debian/Ubuntu: `sudo apt install python3.12-venv`).

```bash
cd workshop/mujoco_balance
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python run_balance.py
```

From the repo root you can also run:

```bash
./run_balance.sh
./run_balance.sh --model new_bike
```

That opens MuJoCo's own Simulate window (the same native app you'd get on a
laptop). Close the window to stop. Click the window (or install `xdotool` on
Ubuntu) so **arrow keys** work (WASD is used by MuJoCo UI). `--model new_bike`
loads the mesh bike for free-drive steering/throttle (no cascade balancer).
Add `--debug` to print realtime timing diagnostics.

Headless smoke test (flat ground):

```bash
python run_balance.py --no-viewer --flat --duration 10
```

The default run keeps the road mostly flat and scatters a few small hills
(about 8–32 cm). Each hill is a brown mound with a yellow pin. The overlay
lists the nearest hill. `--flat` turns them off.

`python` is provided by the virtualenv. Outside it, use `python3`.

## What it does

The MJCF model in `models/bike.xml` is a planar bicycle on a height-field
road. By default `environment.py` scatters small hills and bumps on otherwise
flat ground, and randomizes friction and rider mass. `--flat` turns that off.

The cascade controller keeps it upright; keyboard arrows set the speed.

## Tests

```bash
cd workshop/mujoco_balance
source .venv/bin/activate
pytest -q
```
