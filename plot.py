from matplotlib import pyplot as plt
import pandas

def unpack_pos_column(df, column):
    df[f"{column}.x"], df[f"{column}.y"] = df[column].str[0], df[column].str[1]

def plot(datalog_fname):
    df = pandas.read_json(datalog_fname, lines=True)
    unpack_pos_column(df, 'realsense_position')
    unpack_pos_column(df, 'marvelmind_position')
    unpack_pos_column(df, 'sim_position')
    unpack_pos_column(df, 'position')

    ax = plt.gca()
    ax.set_aspect(1)
    df.plot.scatter('marvelmind_position.x', 'marvelmind_position.y', ax=ax, color='yellow')
    df.plot.scatter('sim_position.x', 'sim_position.y', ax=ax, color='green')
    df.plot.scatter('position.x', 'position.y', ax=ax, color='blue')

    df.plot.scatter('realsense_position.x', 'realsense_position.y', ax=ax, color='gray')
    
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--datalog", help="output datalog csv filename")
    args = parser.parse_args()
    plot(args.datalog)

