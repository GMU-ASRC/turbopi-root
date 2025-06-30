import sys
import pathlib
import argparse
import colorsys
from ast import literal_eval as eval

import numpy as np
import pandas as pd
import matplotlib as mpl
from matplotlib import pyplot as plt

try:
    from itertools import pairwise
except ImportError:
    from more_itertools import pairwise

try:
    from hiwonder_common import project
except ImportError:
    try:
        import project
    except ImportError:
        print("hiwonder_common not found. Certain features will not work.")
        project = None


def hr(h, s, l):  # noqa: E741
    return colorsys.hls_to_rgb(h, l, s)


cr = hr(0.0, 0.9, 0.4)
cb = hr(0.6, 0.9, 0.4)
cg = hr(0.3, 0.9, 0.4)


def get_last_move(moves):
    try:
        return moves[-1]
    except IndexError:
        return float('nan')


def convert_time_from_start(data, start=None):
    tcol = data.iloc[:, 0].name
    start = start or data[tcol][0]
    data[tcol] -= start
    # ts = times.copy() - times[0]
    if tcol.strip().lower() == 'time_ns':
        # cast to float and divide by 1e9 to get seconds
        data = data.assign(**{tcol: data[tcol] / 1e9})
        # ts /= 1e9
    return data


def slice_by_time(data, start=None, end=None, max_length=None):
    a = data.iloc[0, 0] if start is None else start
    b = data.iloc[-1, 0] if end is None else end
    if max_length is not None:
        if end is not None:
            a = max(a, b - max_length)
        else:
            b = min(b, a + max_length)
    ts = data.iloc[:, 0]
    # https://stackoverflow.com/questions/38862657/find-value-greater-than-level-python-pandas
    first_idx = np.argmax(ts >= a)  # returns 0 if not found
    last_idx = np.argmax(ts >= b)
    if first_idx == last_idx or a >= b:
        raise ValueError("All data was excluded.")
    return data.iloc[first_idx:last_idx]


def get_moves(data):
    ts = data.iloc[:, 0]
    # ts = get_time_from_start(data)

    moves = data.iloc[:, -1]
    moves = moves.apply(eval)  # time consuming
    moves = moves.apply(get_last_move)
    moves = moves.apply(pd.Series, index=['v', 'd', 'w'])
    v = moves['v']
    w = moves['w']

    sense = None
    inputs = data.iloc[:, 1:-1]

    if not inputs.empty:
        # create green vertical spanning regions for sensors
        sense = inputs.iloc[:, -1]
        xsen = [ts[0]] if not sense.empty and sense.iloc[0] else []
        xnot = []
        for (xi, si), (xn, sn) in pairwise(zip(ts, sense)):
            if sn > si:
                xsen.append(xn)
            if si > sn:
                xnot.append(xi)
        if not sense.empty and sense.iloc[-1]:
            xnot.append(ts.iloc[-1])

    return ts, v, w, sense, xsen, xnot


def plot_single(fig, ax, data):
    ax.cla()
    axw = ax.twinx()

    ts, v, w, sense, xsen, xnot = get_moves(data)

    if not v.empty:
        # Plot the velocity
        ax.plot(ts, v, label="Velocity", color="blue", alpha=0.5, linestyle="-")
        ax.margins(0.1)

    if not w.empty:
        # Plot the turn rate
        axw.plot(ts, w, label="Turn Rate", color="red", alpha=0.5, linestyle="-")
        axw.margins(0.3)

    if not sense.empty:
        # Plot the binary detection
        # ax.plot(ts, sense, label=sense.name, color="blue", linestyle="-", marker="o")
        ax.plot(ts, sense, c=cg, label=sense.name, alpha=0.1)
        # if plot_state:
        #     ax.subplot(111, aspect='equal')
        for xa, xb in zip(xsen, xnot):
            ax.axvspan(xa, xb, ymin=0.0, ymax=1.0, alpha=0.2, color='green')

    return fig, ax, axw


def graph(data):
    # plt.rcParams["figure.autolayout"] = True
    # read the csv files using pandas excluding the timedate column
    # df = df.drop(columns=['time'], axis=1)

    fig, ax = plt.subplots()

    fig, ax, axw = plot_single(fig, ax, data)

    # Labels and Title

    # make the legend
    # grab the artists from the last row
    handles, labels = ax.get_legend_handles_labels()
    a, b = axw.get_legend_handles_labels()
    handles.insert(-1, *a)
    labels.insert(-1, *b)

    # show the legend
    legend = ax.legend(handles=handles, loc='lower center', ncol=3, fancybox=True, shadow=True)
    # set the linewidth of each legend object
    for obj in legend.legend_handles:
        obj.set_linewidth(3.0)
    s = legend.legend_handles[-1]
    s.set_linewidth(10.0)
    s.set_alpha(0.25)

    plt.title("Output Values over Time")
    ax.set_xlabel("Time since start (seconds)", loc='center')
    ax.set_ylabel("Forward Velocity (m/s)")
    axw.set_ylabel("Angular Velocity (rad/s)")
    # plt.subplots_adjust(hspace=0.2, right=(1 - fig.subplotpars.left))
    plt.grid(True)
    return plt


def make_project(filename=None, root=None):
    if not filename:
        path = pathlib.Path(project.inquire_project())
        return project.make_default_project(path, root=root)
    elif not filename.is_file():
        return project.make_default_project(filename, root=root)


def read_file(filename):
    sep = '\t' if filename.suffix == '.tsv' else ','
    return pd.read_csv(filename, sep=sep, skiprows=[], parse_dates=True)


def data_from_file(filename, offset, offset_end, length):
    data = read_file(filename)
    data = convert_time_from_start(data)
    end = data.iloc[-1, 0] - offset_end if offset_end is not None else None
    try:
        return slice_by_time(data, start=offset, end=end, max_length=length)
    except ValueError as err:
        msg = f"No data was found in the specified range.\nFile length: {data.iloc[-1, 0]} seconds"
        raise ValueError(msg) from err


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=pathlib.Path, help="csv file to be graphed", nargs='?')
    parser.add_argument("--offset", type=float, help="Number of seconds at the start of the file to skip", default=None)
    parser.add_argument("--offset_end", type=float, help="Number of seconds at the end of the file to ignore", default=None)
    parser.add_argument("--length", type=float, help="Length of time to graph in seconds", default=None)
    args = parser.parse_args()

    if project:
        filename = make_project(args.filename, root='logs').root / 'io.tsv'
    else:
        filename = args.filename

    if project and project.inquire_size(filename):
        sys.exit(1)

    plt.rcParams["figure.figsize"] = [7.00, 5.00]
    try:
        data = data_from_file(filename, args.offset, args.offset_end, args.length)
    except ValueError as err:
        print(err)
        sys.exit(1)
    plt = graph(data)
    plt.show()
