from dataclasses import dataclass
import io, re, sys, time

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import ahrs

# NOTE: For simplicity, if any function accepts a dataframe, any modifications done to it
# be in-place, not on a clone of it.

def format_column_name(quantity: str, dimension: str):
    q = quantity.upper()[0]
    d = dimension.lower()
    # Quantity: either rotation or position
    assert q in ['R', 'P']
    # Dimension: either x, y, z, or w
    assert d in ['w', 'x', 'y', 'z']
    # Position does not have a 4th component: w
    assert not (q == 'P' and d == 'w')

    return f"{q}.{d}"

def cleanup_data_csv(df: pd.DataFrame) -> pd.DataFrame:
    # NOTE: The captured data provides a lot of extra detail that won't be necessary
    # when parsing this info such as the position (x,y,z) of all the markers, etc.
    # Using the markers, the opti-tracking software creates a rigidbody with position
    # (x,y,z) and rotation (quaternion). This script will analyze the turbopi's
    # capabilities using that rigidbody.

    # NOTE: Skipping the first two rows because they contain the frame and time values.
    re_pat: str = r"^Rigid Body(\.(\d+))?$"
    columns = df.columns[2:]
    cols_str = "\n".join(columns)
    matches = re.finditer(re_pat, cols_str, re.MULTILINE)
    cols_to_save = [match.group() for match in matches]
    cols_to_remove = [x for x in columns if x not in cols_to_save]
    for col in cols_to_remove:
        del df[col]

    # NOTE: The 'Name' and 'ID' row are removed because they aren't needed
    df = df.drop(index=[0, 1]).reset_index(drop=True)

    # NOTE: Flatten 'Quantity' and 'Dimension' rows into a single column. Then,
    # remove the two rows, combine them, and rename the columns
    quantities: pd.Series = df.iloc[0].values[2:]
    dimensions: pd.Series = df.iloc[1].values[2:]
    new_names: list[str] = ["Frame", "Time (seconds)"]
    new_names.extend([
        format_column_name(q, d) for q, d in zip(quantities, dimensions)
    ])

    df = df.drop(index=[0, 1]).reset_index(drop=True)
    df.rename(columns={
        df.columns[i]: new_names[i]
        for i in range(len(df.columns))
    }, inplace=True)
    return df

def setup_csv(
    csv_path: str, print_info: bool = True, cleanup: bool = True
) -> tuple[str, pd.DataFrame]:
    with open(csv_path, "r") as f:
        content = f.read()

    # NOTE: First line provides extra information about the captured data
    # TODO: parse this information and turn it into something readable in `stdout`.
    first_line_end = content.find("\n")
    info = content[:first_line_end].strip()
    if print_info:
        values: list[str] = info.split(',')
        print("Info about data captured from opti-track camera:")
        # TODO: convert to dict, if needed
        for i in range(0, len(values), 2):
            k, v = values[i], values[i+1]
            print(f"{k:>25}: {v}")

    csv_data = content[first_line_end:].strip()
    df = pd.read_csv(io.StringIO(csv_data))
    return (info, cleanup_data_csv(df) if cleanup else df)

def create_sample_csv(orig_df: pd.DataFrame, n: int = 100, start: int = 0) -> pd.DataFrame:
    end = len(orig_df) if n < 0 else start + n
    return orig_df.iloc[start:end]

def compute_distances(df: pd.DataFrame) -> None:
    pos_col = "Position"
    dims = ['x', 'y', 'z']

    d_p = { dim: np.array([]) for dim in dims }
    for dim in dims:
        col = format_column_name(pos_col, dim)
        dim_pos = df[col].to_numpy(dtype=np.float64)
        df[f"d_{col}"] = d_p[dim] = np.array([np.nan, *np.diff(dim_pos)])

    # Distance formula: sqrt(x^2 + y^2 + z^2)
    df[f"d_P"] = np.sqrt(d_p['x']*d_p['x'] + d_p['y']*d_p['y'] + d_p['z']*d_p['z'])


def identify_movement_ranges(df: pd.DataFrame) -> list[tuple[int, int]]:
    compute_distances(df)

    # NOTE: Considered as "moved" if delta between two positions is more than 2mm
    # between two frames
    # TODO: find a better way to handle this (because it could be that some robots travel
    # a distance of less than 2mm between frames.)
    THRESHOLD: float = 2.0
    dist = df[f"d_P"].to_numpy(dtype=np.float64)
    df[f"moved"] = moved = np.where(dist > THRESHOLD, 1, 0)

    stop_marks: list[int] = []
    ind: int = 0
    while ind < len(moved)-3:
        temp = moved[ind] + moved[ind+1] + moved[ind+2] + moved[ind+3]
        if temp == 2:
            stop_marks.append(ind+1)
            # NOTE: once a change in movement is detected, jump a few values forward, to
            # prevent the detection of a change in movement very close to one another.
            ind += 5
        else:
            # NOTE: Otherwise, just keep iterating through the values normally
            ind += 1


    # plot_points(
    #     np.arange(0, len(dist), dtype=int),
    #     np.array([0, *np.cumsum(dist[1:])]), # cumulative distance sum
    #     v_hlines=stop_marks
    # )

    # Convert from the 'stop' marks to the start and end of time periods
    # where the robot moves.
    move_ranges: list[tuple[int, int]] = []
    for i in range(0, len(stop_marks), 2):
        move_ranges.append((stop_marks[i], stop_marks[i+1]))

    print(move_ranges)
    print(len(move_ranges))

    df.to_csv("test.csv")
    return move_ranges


# NOTE: Only needed for visualization
def plot_points(xs: np.ndarray, ys: np.ndarray, v_hlines: list[int] = [],
    out_path: str = "plot.svg"
) -> None:
    """A simple utility function that allows for graphing via matplotlib"""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(xs, ys)
    y_min, y_max = ax.get_ylim()
    ax.vlines(x=v_hlines, ymin=y_min, ymax=y_max, colors='r', linestyles="dashed")
    plt.savefig(out_path)


def compute_linear_speeds(df: pd.DataFrame) -> None:
    move_ranges = identify_movement_ranges(df)
    time = df["Time (seconds)"].to_numpy(dtype=np.float64)
    dists = df["d_P"].to_numpy(dtype=np.float64)

    powers = [100, -100, 90, -90, 80, -80, 70, -70, 60, -60, 50, -50]

    for i, (start, end) in enumerate(move_ranges):
        # Speed in mm/s
        speed_mmps = np.sum(dists[start:end]) / (time[end] - time[start])
        # Speed in m/s
        speed_mps = speed_mmps / 1000
        print(f"{powers[i]:>4}%: {speed_mps:.3} m/s")

def compute_angular_speeds(df: pd.DataFrame) -> None:
    # NOTE: This format is required (w, x, y, z) for the computations
    quat_cols = ["R.w", "R.x", "R.y", "R.z"]
    quats = np.array([
        df[col].to_numpy(dtype=np.float64) for col in quat_cols
    ]).T
    assert quats.shape[1] == 4, f"A quat array has to have 4 columns; quats.shape = {quats.shape}"
    dt: float = 0.01

    # Resource for quaternions vs time to angular velocities:
    #   - Blog: https://mariogc.com/post/angular-velocity-quaternions/
    #   - Python package: https://github.com/Mayitzin/ahrs

    q_arr = ahrs.QuaternionArray(quats)
    # NOTE: The ahrs package can compute the angular velocities but requires a constant timestep,
    # dt. If there isn't a constant dt, using the function presented in the blog might work,
    # which is calculating the angular velocity of consecutive pairs of quaternions.
    omegas = np.array([[np.nan]*3, *q_arr.angular_velocities(dt)]).T

    df["omegas.x"] = omegas[0]
    df["omegas.y"] = omegas[1]
    df["omegas.z"] = omegas[2]
    df["omega"] = np.sqrt(omegas[0]*omegas[0] + omegas[1]*omegas[1] + omegas[2]*omegas[2])

    df.to_csv("my_omegas.csv", index=False)


actual_csv_path: str = "optitrack-data/Take 2025-12-05 03.07.49 PM turbopi-01.csv"
sample_csv_path: str = "sample.csv"

# NOTE: The creation of samples is temporary. Will be removed
if len(sys.argv) == 2 and sys.argv[1] == "redo":
    # This runs if 'redo' cmd arg is used like `python ....py redo`
    actual_info, actual_df = setup_csv(actual_csv_path, print_info=False)
    df = create_sample_csv(actual_df, n=-1)
    output = actual_info + "\n\n" + df.to_csv(index=False)
    with open(sample_csv_path, "w") as f:
        f.write(output)

    print("Recreated samples")

info, df = setup_csv(sample_csv_path, cleanup=False)
# compute_linear_speeds(df)
compute_angular_speeds(df)
