from dataclasses import dataclass
from typing import Sequence, Optional, Dict, Any
import shutil
import numpy as np

def _cm2inch(v: float) -> float: return v / 2.54

@dataclass
class TrackJob:
    snapshot: Dict[str, Any]           # controller.track_snapshot()
    sim_data: Sequence[np.ndarray]     # list of (T,3): x,y,z
    out_png: str
    out_gif: Optional[str] = None
    dpi: int = 150
    big_picture: bool = False
    use_latex: bool = True
    animate: bool = True
    title: str = "Track in the intensity field"
    label: str = ""

def run_track_render(job: TrackJob) -> None:
    import matplotlib; matplotlib.use("Agg")             # set non-interactive backend INSIDE subprocess
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib import rc, rcParams, rcParamsDefault
    if job.label:
        print(f"[TrackRender] Starting '{job.label}'", flush=True)
    rcParams.update(rcParamsDefault)
    use_latex = bool(job.use_latex)
    if use_latex and shutil.which('latex') is None:
        use_latex = False
        if job.label:
            print(f"[TrackRender] '{job.label}' falling back to non-LaTeX labels: 'latex' executable not found", flush=True)
        else:
            print("[TrackRender] Falling back to non-LaTeX labels: 'latex' executable not found", flush=True)
    plt.rc('text', usetex=use_latex); rc('font', size=30)

    # required keys: grid_x, grid_y, grid_z, target_isoline, isolines, fps, grid_size
    snap = job.snapshot
    grid_x = np.asarray(snap["grid_x"]); grid_y = np.asarray(snap["grid_y"]); grid_z = np.asarray(snap["grid_z"])
    target_isoline = snap["target_isoline"]; isolines = snap["isolines"]; fps = int(snap["fps"]); grid_size = int(snap["grid_size"])
    colors = list(snap.get("colors") or [])
    if grid_size <= 0: raise ValueError("grid_size must be positive")

    size_cm = (50, 26) if job.big_picture else (25, 25)
    fig, ax = plt.subplots(figsize=(_cm2inch(size_cm[0]), _cm2inch(size_cm[1])), dpi=job.dpi)
    cs = ax.contour(grid_x, grid_y, grid_z, levels=isolines, cmap='viridis'); ax.clabel(cs, inline=True)
    ax.set_xlabel('X, m / East'); ax.set_ylabel('Y, m / North')
    ax.contour(grid_x, grid_y, grid_z, levels=[target_isoline], colors='red')
    ax.set_title(job.title)

    quivers = []; plotData = []
    for i, sim in enumerate(job.sim_data):
        arr = np.asarray(sim); x = arr[:,0]; y = arr[:,1]; z = arr[:,2]
        step = max(len(x)//grid_size, 1); N = y[::step]; E = x[::step]; D = z[::step]
        data = np.array([N, E, -D], dtype=float); color = colors[i] if i < len(colors) else None
        ax.plot(data[0][0], data[1][0], marker='*', markersize=10, color=color, label='_nolegend_', zorder=10)
        q = ax.quiver(data[0][0], data[1][0], data[0][1]-data[0][0], data[1][1]-data[1][0],
                      angles='xy', scale_units='xy', scale=1, color=color, width=0.02, zorder=15, label='_nolegend_')
        quivers.append(q)
        line, = ax.plot(data[0], data[1], lw=2, c=color, zorder=10, label=f'agent {i+1}')
        plotData.append((line, data))
    ax.legend(); fig.tight_layout(); fig.savefig(job.out_png)

    has_ffmpeg = shutil.which('ffmpeg') is not None

    if job.animate and job.out_gif:
        if not has_ffmpeg:
            reason = "'ffmpeg' executable not found"
            if job.label:
                print(f"[TrackRender] '{job.label}' skipping animation: {reason}", flush=True)
            else:
                print(f"[TrackRender] Skipping animation: {reason}", flush=True)
        else:
            def anim_fn(k: int):
                artists = []
                for (line, data), q in zip(plotData, quivers):
                    idx = min(max(k, 0), data.shape[1]-1); line.set_data(data[0:2, :idx+1])
                    if idx < data.shape[1]-1: dx = data[0, idx+1] - data[0, idx]; dy = data[1, idx+1] - data[1, idx]
                    else: p = max(idx-1, 0); dx = data[0, idx] - data[0, p]; dy = data[1, idx] - data[1, p]
                    n = np.hypot(dx, dy) or 1.0; L = 2.0; q.set_offsets([data[0, idx], data[1, idx]]); q.set_UVC(L*dx/n, L*dy/n)
                    artists.extend([line, q])
                return artists
            ani = animation.FuncAnimation(fig, anim_fn, frames=grid_size, interval=200, blit=False, repeat=True)
            writer = None
            try:
                writer = animation.FFMpegWriter(fps=fps)
            except Exception:
                writer = None
            if writer is None:
                if job.label:
                    print(f"[TrackRender] '{job.label}' falling back to PillowWriter: FFMpegWriter unavailable", flush=True)
                else:
                    print("[TrackRender] Falling back to PillowWriter: FFMpegWriter unavailable", flush=True)
                writer = animation.PillowWriter(fps=fps)
            ani.save(job.out_gif, writer=writer)
    plt.close(fig)
    if job.label:
        print(f"[TrackRender] Finished '{job.label}'", flush=True)
