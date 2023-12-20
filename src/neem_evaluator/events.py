from .knowrob import *
from .helper import *

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import PolyCollection


def pre_process_events(events: dict) -> Dict:
    """
    Maps event instances to their human-readable counterpart. Not really used since the new NEEM structure
    :param events:
    :return:
    """
    result = {}
    for action, readable in events.items():
        type = readable.split("#")[1].split("_")[0]
        result[action] = type
    return result


def plot_events(neem: 'NEEM') -> None:
    """
    Code from here: https://stackoverflow.com/questions/51505291/timeline-bar-graph-using-python-and-matplotlib
    Plots actions in a neem on a time scale diagram

    """
    # events = get_all_actions_in_neem()
    events = neem.action_list
    event_names = list(set([act.name for act in neem.action_list]))
    print(event_names)
    rows = dict(zip(event_names, list(range(0, len(event_names)))))

    # rows = {"Reaching": 1, "Grasping": 2, "Pouring": 3, "PhysicalTask": 4}
    color = ["C" + str(rows[event_name]) for event_name in rows.keys()]
    colormapping = dict(zip(event_names, color))
    # colormapping = {"Reaching": "C0", "Grasping": "C1", "Pouring": "C2", "PhysicalTask": "C3"}
    # all_intervals = get_event_intervals()

    verts = []
    colors = []
    for event in events:
        # event[0] is a quick fix since there is only one event per type for now
        # intervals = all_intervals[event[0]]
        v = [(event.start, rows[event.name] - .4),
             (event.start, rows[event.name] + .4),
             (event.end, rows[event.name] + .4),
             (event.end, rows[event.name] - .4),
             (event.start, rows[event.name] - .4)]
        verts.append(v)
        colors.append(colormapping[event.name])
    bars = PolyCollection(verts, facecolors=colors)

    fig, ax = plt.subplots()
    ax.add_collection(bars)
    ax.autoscale()
    # loc = mdates.MinuteLocator(byminute=[0, 15, 30, 45])
    # ax.xaxis.set_major_locator(loc)
    # ax.xaxis.set_major_formatter(mdates.AutoDateFormatter(loc))

    # ax.set_yticks([1, 2, 3, 4])
    # ax.set_yticklabels(["Reaching", "Grasping", "Pouring", "PhysicalTask"])
    ax.set_yticks(list(range(0, len(event_names))))
    ax.set_yticklabels(event_names)
    plt.show()


def event_metric(neem1: 'NEEM', neem2: 'NEEM'):
    """
    Metric for events, this metric compares two NEEMs in a number of metrics and returns them as a dictionary. The idea
    is that there can not be one single metric for events therefore this works like an expert system. The different
    metrics used are:

        * Co-apperance of events
        * relative distance of events
        * Events ordered by time
        * the count for each event type
        * Total time taken for each NEEM

    :param neem1: The first NEEM to compare
    :param neem2: The second NEEM to compare
    :return: A dictionary with the results of the different metric
    """
    results = {}
    results["co-appearance"] = co_appearance_of_events(neem1, neem2)
    results["relative-distance"] = relative_distance_of_events(neem1, neem2)
    results["time-ordered"] = [[act.name for act in neem1.action_list],
                               [act.name for act in neem2.action_list]]
    results["action-count"] = [event_count_of_same_type(neem1), event_count_of_same_type(neem2)]
    results["execution-time"] = [total_execution_time(neem1), total_execution_time(neem2)]
    results["grasping-attempts"] = [failed_grasping_attempts(neem1), failed_grasping_attempts(neem2)]
    return results


