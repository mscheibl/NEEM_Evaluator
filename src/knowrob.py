import rospy
import rosservice
from mongo import restore, tf
from datetime import datetime
if '/rosprolog/query' in rosservice.get_service_list():
    from rosprolog_client import Prolog
    prolog = Prolog()
else:
    rospy.logerr("No Knowrob services found")
    raise ImportWarning("No Knowrob services found")

rospy.init_node("NEEM_Evaluator")


def remember_neem(path):
    # Loads the NEEM into Knowrob
    prolog.once(f"remeber('{path}')")
    # Is needed since Knowrob does not load the TF data to Mongo and without it
    # the queries do not work
    restore(path)


def get_event_intervals():
    intervals = prolog.once("findall([Begin, End, Evt], event_interval(Evt, Begin, End), Times)")
    result = {}
    for interval in intervals["Times"]:
        result[interval[2]] = {"start": interval[0], "end": interval[1]}
        #result.append({"event": interval[2], "start": interval[0], "end": interval[1]})
    return result


def get_all_actions_in_neem():
    actions = prolog.once("findall([Act, Task], (is_action(Act), executes_task(Act, Task)), Act)")["Act"]
    res = {}
    for act in actions:
        res[act[0]] = act[1]
    return res


def get_object_to_action():
    mapping = prolog.once("findall([Act, Obj], has_participant(Act, Obj), Obj)")
    obj_to_act = {}
    for m in mapping["Obj"]:
        if m[0] not in obj_to_act.keys():
            obj_to_act[m[0]] = [m[1]]
        else:
            obj_to_act[m[0]].append(m[1])
    return obj_to_act


def get_all_tf_for_action(action):
    intervals = get_event_intervals()[action]
    objects = get_object_to_action()[action]
    result = {}
    for obj in objects:
        # TODO Mapping from ontology to child frame names !!
        result[obj] = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"]), "$lt": datetime.fromtimestamp(intervals["end"])}, "child_frame_id" : HERE })
    #query = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"]), "$lt": datetime.fromtimestamp(intervals["end"])}})
    return query
