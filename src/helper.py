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
    i = 0
    while chunk := next_n(coll, n):
        i+=1
        yield chunk

def docs_in_cursor(cursor):
    coll = cursor.collection
    return coll.count_documents(cursor.explain()["executionStats"]["executionStages"]["filter"])  

def tfs_to_velocity(coll):
    velocity = np.array([0, 0, 0])
    first = np.array(transform_to_translation(coll.next()["transform"]))
    second = np.array(transform_to_translation(coll.next()["transform"]))
    while coll.alive:
        yield second - first
        first = second
        second = np.array(transform_to_translation(coll.next()["transform"]))
