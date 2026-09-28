# MuJoCo bicycle balance

Planar bicycle that stays upright with wheel torque.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python run_balance.py
python run_balance.py --model new_bike
```

From the repo root (this folder’s parent):

```bash
./run_balance.sh
./run_balance.sh --model new_bike
```

`python run_balance.py` opens MuJoCo's native window. The camera follows
the bike from the side. On Ubuntu, click the window (or install `xdotool`)
so **arrow keys** reach GLFW. WASD is left for MuJoCo Simulate UI toggles.

The floor is **mostly flat**. A few small hills sit on it (about 8–32 cm).

**How to spot a hill:**
- Brown mound on the far side of the road, with a **yellow pin** on the peak
- Top-left overlay: `Nearest hill` (number, metres ahead, height)
- Terminal also prints `hills: #1 at 2.3 m (16 cm), ...`

`--flat` turns the hills off. Close the window to quit.

## Drive the bike

Click the MuJoCo window so it has keyboard focus, then:

| Key | Action |
| --- | --- |
| `→` | Faster (cruise up) |
| `←` | Slower / reverse |
| `↑` | Short shove forward |
| `↓` | Short shove backward |
| `0` | Stop |
| `Space` | Reset the world |

The cascade controller keeps the **planar** bike upright. You set the speed. Overlay line **Keys** repeats this.

`--controller none` turns the balancer off; then `←` `→` are raw shoves and it will fall if you are not careful.

### `--model new_bike`

Loads `models/new_bike.xml` (mesh bike + `step_meshes/`). There is **no**
cascade balancer — arrow keys map to rear-wheel and steering torques:

| Key | Action |
| --- | --- |
| `↑` | More rear torque |
| `↓` | Less / reverse rear torque |
| `→` | Steer right |
| `←` | Steer left |
| `0` | Stop |
| `Space` | Reset |

The viewer multi-steps physics to keep near wall-clock realtime (XML
`timestep` 0.0005 is raised to 0.002 at load for the interactive demo).
Use `--debug` to print ms/step and realtime factor WARN lines.

## Controller

`controller.py` is the cascade balancer (this is the file to edit):

1. **Pitch PD** — cart force to catch lean.
2. **Speed loop** — chooses a lean command so the bike holds a cruise speed.
3. **Slope feed-forward** — extra force `m g sin(theta)` on a bump.

```bash
python run_balance.py                  # controller on (default)
python run_balance.py --controller none
python run_balance.py --speed 0.8
```

The overlay shows commanded vs actual pitch, speed, and drive force.

| Flag | Meaning |
| --- | --- |
| `--model bike\|new_bike` | Planar demo bike (default) or mesh `new_bike` (free-drive). |
| `--debug` | Timing/key diagnostics; WARN if realtime factor is low. |
| `--no-viewer` | Headless (no window). |
| `--duration N` | Simulated seconds. Headless: then exit. Window: then reset and keep going. |
| `--controller pd\|none` | Cascade balancer (planar only), or off. Ignored for `new_bike`. |
| `--speed V` | Cruise speed in m/s (overrides the random world cruise). |
| `--flat` | Level ground, no randomization. |
| `--seed N` | Repeat the same random world. |
| `--lean-deg N` | Initial pitch in degrees (random if omitted). |
| `--save-plot path.png` | Write pitch vs time (headless). |
| `--save-video path.mp4` | Record an offscreen MP4 (headless). |
