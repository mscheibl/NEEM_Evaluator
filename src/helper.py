import numpy as np


def transform_to_list(transform):
    translation = [transform["translation"]["x"],
                    transform["translation"]["y"],
                    transform["translation"]["z"]]
    rotation = [transform["rotation"]["x"],
                transform["rotation"]["y"],
                transform["rotation"]["z"],
                transform["rotation"]["w"]]
    return [translation, rotation]


def transform_to_translation(transform):
    return transform_to_list(transform)[0]


def transform_to_rotation(transform):
    return transform_to_list(transform)[1]


def next_n(coll, n: int):
    """
    Returns the next n elements from the collection coll
    """
    result = []
    i = 0
    while i < n and (s := next(coll, False)):
        result.append(s)
        i += 1
    return result


def chunks(coll, n):
    while chunk := next_n(coll, n):
        yield chunk


def docs_in_cursor(cursor):
    coll = cursor.collection
    return coll.count_documents(cursor.explain()["executionStats"]["executionStages"]["filter"])


def tfs_to_velocity(coll, chunks=1):
    """
    Creates the velocities from a collection of TFs, the number of calulated velocities is number_of_tfs - 1
    :param coll: The collection from which to calculate the velocities
    :param chunks: Number velocity vectors that should be returned
    :return: A generator that returns velocities with chunk size
    """
    velocities = []
    tfs = []
    for i in range(chunks+1):
        tfs.append(coll.next())

    while coll.alive:
        for i in range(len(tfs) - 1):
            first_vector = np.array(transform_to_translation(tfs[i]["transform"]))
            second_vector = np.array(transform_to_translation(tfs[i + 1]["transform"]))
            velocities.append((second_vector - first_vector, tfs[i]["header"]["seq"]))
        yield velocities
        velocities = []
        tfs.remove(tfs[0])
        tfs.append(coll.next())

    # while coll.alive:
    #     yield second_vector - first_vector, first["header"]["seq"]
    #     first = second
    #     first_vector = second_vector
    #     second = coll.next()
    #     second_vector = np.array(transform_to_translation(second["transform"]))
