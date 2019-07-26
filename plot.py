from matplotlib import pyplot as plt
import numpy as np
import pandas

def unpack_pos_column(df, column):
    try:
        df[f"{column}.x"], df[f"{column}.y"] = df[column].str[0], df[column].str[1]
    except (AttributeError, KeyError):
        df[f"{column}.x"], df[f"{column}.y"] = np.nan, np.nan

def plot(datalog_fname, waypoint_fname):
    df = pandas.read_json(datalog_fname, lines=True)
    unpack_pos_column(df, 'realsense_position')
    unpack_pos_column(df, 'marvelmind_position')
    unpack_pos_column(df, 'sim_position')
    unpack_pos_column(df, 'position')
    unpack_pos_column(df, 'rs_to_mm_offset')

    ax_xy = plt.gca()
    ax_xy.set_aspect(1)
    df.plot.scatter('marvelmind_position.x', 'marvelmind_position.y', ax=ax_xy, color='yellow', marker=",")
    if 'sim_position' in df:
        df.plot.scatter('sim_position.x', 'sim_position.y', ax=ax_xy, color='green', marker=",")
    df.plot.scatter('position.x', 'position.y', ax=ax_xy, color='blue', marker=",")
    df.plot.scatter('realsense_position.x', 'realsense_position.y', ax=ax_xy, color='gray', marker=",")

    if waypoint_fname is not None:
        waypoints = pandas.read_csv(waypoint_fname, header=None, names=['x','y'])
        waypoints.plot.scatter('x', 'y', ax=ax_xy, color='black')

    fig, ax = plt.subplots(nrows=3, sharex=True)

    df.plot.scatter('time', 'realsense_heading', ax=ax[0], color='gray')
    df.plot.scatter('time', 'heading', ax=ax[0], color='blue')
    df.plot.scatter('time', 'drive_angle', ax=ax[0], color='red')
    if 'sim_heading' in df:
        df.plot.scatter('time', 'sim_heading', ax=ax[0], color='green')
    if 'target_bearing' in df:
        df.plot.scatter('time', 'target_bearing', ax=ax[0], color='orange')
    if 'target_bearing_delta' in df:
        df.plot.scatter('time', 'target_delta_heading', ax=ax[0], color='purple')
    
    df.plot.scatter('time', 'rs_to_mm_angle', ax=ax[0], color='purple')
    df.plot.scatter('time', 'rs_to_mm_scale', ax=ax[1], color='green')
    df.plot.scatter('time', 'rs_to_mm_offset.x', ax=ax[2], color='blue')
    df.plot.scatter('time', 'rs_to_mm_offset.y', ax=ax[2], color='cyan')

    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--datalog", help="output datalog json filename")
    parser.add_argument("--waypoints", help="waypoint csv filename")
    args = parser.parse_args()
    plot(args.datalog, args.waypoints)

