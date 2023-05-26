from .knowrob import  *

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import PolyCollection


def pre_process_events(events):
    result = {}
    for action, readable in events.items():
        type = readable.split("#")[1].split("_")[0]
        result[action] = type
    return result


def plot_events():
    """
    Code from here: https://stackoverflow.com/questions/51505291/timeline-bar-graph-using-python-and-matplotlib
    :return:
    """
    events = get_all_actions_in_neem()
    rows = {"Reaching": 1, "Grasping": 2, "Pouring": 3, "PhysicalTask": 4}
    colormapping = {"Reaching": "C0", "Grasping": "C1", "Pouring": "C2", "PhysicalTask": "C3"}
    all_intervals = get_event_intervals()

    verts = []
    colors = []
    for event_type, event in events.items():
        # event[0] is a quick fix since there is only one event per type for now
        intervals = all_intervals[event[0]]
        v = [(intervals["start"], rows[event_type] - .4),
             (intervals["start"], rows[event_type] + .4),
             (intervals["end"], rows[event_type] + .4),
             (intervals["end"], rows[event_type] - .4),
             (intervals["start"], rows[event_type] - .4)]
        verts.append(v)
        colors.append(colormapping[event_type])
    bars = PolyCollection(verts, facecolors=colors)

    fig, ax = plt.subplots()
    ax.add_collection(bars)
    ax.autoscale()
    # loc = mdates.MinuteLocator(byminute=[0, 15, 30, 45])
    # ax.xaxis.set_major_locator(loc)
    # ax.xaxis.set_major_formatter(mdates.AutoDateFormatter(loc))

    ax.set_yticks([1, 2, 3, 4])
    ax.set_yticklabels(["Reaching", "Grasping", "Pouring", "PhysicalTask"])
    plt.show()


def event_metric():
    events = get_all_actions_in_neem()
    intervals = get_event_intervals()

    start_time = intervals[events["PhysicalTask"]]

