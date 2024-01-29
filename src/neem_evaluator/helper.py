from typing import Dict, Tuple, List, Iterable

import numpy as np
from numpy.typing import ArrayLike


def transform_to_list(transform: dict) -> List:
    """
    Extracts the xyz and xyzw values from a transform and returns them as two lists

    :param transform: The transform which should be unpacked
    :return: Two lists for position and orientation
    """
    translation = [transform["translation"]["x"],
                    transform["translation"]["y"],
                    transform["translation"]["z"]]
    rotation = [transform["rotation"]["x"],
                transform["rotation"]["y"],
                transform["rotation"]["z"],
                transform["rotation"]["w"]]
    return [translation, rotation]


def transform_to_translation(transform: dict) -> List:
    """
    Returns only the translation of the given transform as a list

    :param transform: The transform which should be unpacked
    :return: A list representing the translation
    """
    return transform_to_list(transform)[0]


def transform_to_rotation(transform: dict) -> List:
    """
    Extracts the orientation of the given transformation and returns it as a list

    :param transform: The transform which should be unpacked
    :return: A list representing the orientation of the given transform
    """
    return transform_to_list(transform)[1]


def next_n(coll: Iterable, n: int) -> List:
    """
    Returns the next n elements from the collection coll

    :return: A list of the next n elements from the given collection, or all available elements if there are less then n
    """
    result = []
    i = 0
    while i < n and (s := next(coll, False)):
        result.append(s)
        i += 1
    return result


def chunks(coll: Iterable, n: int) -> List:
    """
    Generates chunks of size n from the given collection.

    :param coll: Cursor to a mongo collection
    :param n: The size of chunks
    :yield: Chunks from the collection
    """
    while chunk := next_n(coll, n):
        yield chunk


def docs_in_cursor(cursor):
    """
    Counts the amount of elements in a cursor.

    :param cursor: The cursor which should be counted
    :return: The number of elements
    """
    coll = cursor.collection
    #return coll.count_documents(cursor.explain()["executionStats"]["executionStages"]["filter"])
    return coll.count_documents(cursor.explain()["queryPlanner"]["parsedQuery"])


def tfs_to_velocity(coll: Iterable, chunks: int = 1):
    """
    Creates the velocities from a collection of TFs, the number of calulated velocities is number_of_tfs - 1

    :param coll: The collection from which to calculate the velocities
    :param chunks: Number velocity vectors that should be returned
    :return: A generator that returns velocities with chunk size
    """
    velocities = []
    tfs = []
    for i in range(chunks+1):
        tfs.append(next(coll, None))

    # while coll.alive:
    while t := next(coll, False):
        #tfs.append(t)
        for i in range(len(tfs) - 1):
            first_vector = np.array(transform_to_translation(tfs[i]["transform"]))
            second_vector = np.array(transform_to_translation(tfs[i + 1]["transform"]))
            velocities.append((second_vector - first_vector, tfs[i]["header"]["seq"]))
        yield velocities
        velocities = []
        tfs.remove(tfs[0])
        #tfs.append(coll.next())
        tfs.append(t)


def cluster_sequences(sequences: List) -> List:
    """
    Creates clusters of sequences, a cluster is any number of sequences with a distance with 1.

    :param sequences: A list of sequences
    :return: A list of clusters
    """
    cluster = (np.diff(sequences)>1).nonzero()[0] + 1
    result = []
    for i in range(len(cluster) - 1):
        result.append([sequences[cluster[i]: cluster[i+1]]][0])
    return result


def cluster_to_actions(cluster: List, seq_to_actions: dict) -> dict:
    """
    Matches clusters of sequences to actions that happened during the sequence.

    :param cluster: List with cluster of sequences
    :param seq_to_actions: Mapping of sequences to actions
    :return: A dictionary mapping individual sequences to action
    """
    result = {}
    i = 0
    for cls in cluster:
        try:
            mapping = [seq_to_actions[x] for x in cls]
            result[i] = mapping
        except KeyError:
            pass
        i += 1
    return result


def co_appearance_of_events(neem1: 'NEEM', neem2: 'NEEM') -> List[Tuple['Action', 'Action']]:
    """
    Checks if events with the same name appeared at the same time in the neem. This is checked by comparing the
    relative starting time of both events.

    :param neem1: First neem from which the events should be compared
    :param neem2: Second neem which events should be compared to the first one
    :return: A List with tuples of events that appear at the same time.
    """
    appearance_pairs = []
    for action in neem1.action_list:
        for action2 in neem2.action_list:
            if action2.name == action.name:
                if action.relative_start <= action2.relative_start:
                    appearance_pairs.append((action, action2))
    return appearance_pairs


def relative_distance_of_events(neem1: 'NEEM', neem2: 'NEEM') -> Dict[Tuple['Action', 'Action'], Tuple[ArrayLike, ArrayLike]]:
    """
    Calculates the relative distance of events of the same type between two events.

    :param neem1: The first neem to compare to
    :param neem2: The second neem to compare
    :return: A dictionary with a Tuple of events as key and the relative start and end poses as values
    """
    event_distances1 = event_start_and_end_pose(neem1)
    event_distances2 = event_start_and_end_pose(neem2)
    res = {}
    for action, distances in event_distances1.items():
        for action2, distances2 in event_distances2.items():
            if action.name == action2.name:
                res[(action, action2)] = (np.absolute(distances[0] - distances2[0]), np.absolute(distances[1] - distances2[1]))
    return res


def event_start_and_end_pose(neem: 'NEEM') -> Dict['Action', Tuple[ArrayLike, ArrayLike]]:
    """
    Calculates the start and end poses of an event. This uses all objects that are part of this event, then gets the
    TF during the event for this object and takes the first and last TF. Lastly, the average over all TFs for all objects
    is use.

    :param neem: Neem for which the event start and end times should be calculated.
    :return: A dictionary with the action as key and the averaged start and end positions as values.
    """
    res = {}
    for action in neem.action_list:
        first_positions = []
        last_positions = []
        for obj in action.objects:
            tfs = obj.get_tfs_during_action(action)
            if not tfs:
                continue
            first_positions.append(np.array(list(tfs[0]["transform"]["translation"].values())))
            last_positions.append(np.array(list(tfs[-1]["transform"]["translation"].values())))

        res[action] = (np.mean(first_positions, axis=0), np.mean(last_positions, axis=0))
    return res


def time_ordered_actions(actions: List['Action']) -> List['Action']:
    """
    Sorts the given actions by their start time.

    :param actions:
    :return:
    """
    return sorted(actions, key=lambda act: act.start)


def event_count_of_same_type(neem: "NEEM") -> Dict['Action', int]:
    """
    Counts all events of the same type.

    :param neem: NEEM for which the events should be counted
    :return: A dictionary with the action as input and the amout how often it occyurs as calue
    """
    res = {}
    for action_name, actions in neem.actions.items():
        res[action_name] = len(actions)
    return res


def total_execution_time(neem: 'Neem') -> float:
    """
    Calculates the total time taken for the actions in this neem. The time is calculated by taking the earliest start time
    and latest end time.

    :param neem: NEEM for which the total execution time should be calculated
    :return: Time taken for the actions in the Neem
    """
    start = neem.action_list[0].start
    end = sorted([act.end for act in neem.action_list])[-1]
    return end - start


def failed_grasping_attempts(neem: 'Nemm') -> int:
    """
    Measures the number of grasping attempts that failed. A failed grasp attempt is considered when a grasping event is
    ended without a pouring event stating.

    :param neem: The neem for which the failed grasps should be measured
    :return: The number of failed grasp attempts
    """
    grasping_actions = list(filter(lambda act: act.name == "Grasping", neem.action_list))
    pouring_actions = list(filter(lambda act: act.name == "Pouring", neem.action_list))[0]
    failed_attempts = 0
    for grasping_act in grasping_actions:
        if grasping_act.end < pouring_actions.start:
            failed_attempts += 1
    return failed_attempts



