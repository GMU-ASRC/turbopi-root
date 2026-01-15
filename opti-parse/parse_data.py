from dataclasses import dataclass
import io, re
import sys

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

def compute_distances(orig_df: pd.DataFrame) -> pd.DataFrame:
    df = orig_df.copy()
    pos_col = "Position"
    dims = ['x', 'y', 'z']

    d_p = { dim: np.array([]) for dim in dims }
    for dim in dims:
        col = format_column_name(pos_col, dim)
        dim_pos = df[col].to_numpy(dtype=np.float64)
        df[f"d_{col}"] = d_p[dim] = np.array([np.nan, *np.diff(dim_pos)])

    # Distance formula: sqrt(x^2 + y^2 + z^2)
    df[f"d_P"] = np.sqrt(d_p['x']*d_p['x'] + d_p['y']*d_p['y'] + d_p['z']*d_p['z'])

    return df

def detect_stops(df: pd.DataFrame) -> pd.DataFrame:
    df = compute_distances(df)

    # NOTE: Considered as "moved" if delta between two positions is more than 2mm
    # between two frames
    # TODO: find a better way to handle this (because it could be that some robots travel
    # a distance of less than 2mm between frames.)
    THRESHOLD: float = 2.0
    dist = df[f"d_P"].to_numpy(dtype=np.float64)
    df[f"moved"] = moved = np.where(dist > THRESHOLD, 1, 0)

    stop_marks: list[float] = []
    for i in range(0, len(moved)-1):
        if moved[i] != moved[i+1]:
            stop_marks.append(i+1)

    # NOTE: Only needed for visualization
    # plot_points(
    #     np.arange(0, len(dist), dtype=int),
    #     np.array([0, *np.cumsum(dist[1:])])   # cumulative distance sum
    #     v_hlines=stop_marks
    # )

    return df

def plot_points(xs: np.ndarray, ys: np.ndarray, v_hlines: list[float] = [],
    out_path: str = "plot.svg"
) -> None:
    """A simple utility function that allows for graphing via matplotlib"""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(xs, ys)
    y_min, y_max = ax.get_ylim()
    ax.vlines(x=v_hlines, ymin=y_min, ymax=y_max, colors='r', linestyles="dashed")
    plt.savefig(out_path)

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
other_df = detect_stops(df)
# other_df.to_csv("test.csv", float_format=lambda x: f"{x:.2}")
other_df.to_csv("test.csv")
# with open("test.txt", "w") as f:
#     print(other_df, file=f)
