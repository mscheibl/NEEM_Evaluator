import rospy
import rosservice
from .mongo import restore, tf, get_tf_for_object
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
    prolog.once(f"remember('{path}')")
    # Is needed since Knowrob does not load the TF data to Mongo and without it
    # the queries do not work
    #restore(path)


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


def get_objects_for_action(action):
    mapping = prolog.once("findall([Act, Obj], has_participant(Act, Obj), Obj)")
    act_to_obj = {}
    for m in mapping["Obj"]:
        if m[0] not in act_to_obj.keys():
            act_to_obj[m[0]] = [m[1]]
        else:
            act_to_obj[m[0]].append(m[1])
    return act_to_obj[action]


def get_actions_for_object():
    all_actions = get_all_actions_in_neem()
    action_to_objects = {}
    object_to_actions = {}
    for act in all_actions.keys():
        action_to_objects[act] = get_objects_for_action(act)
    for act, objs in action_to_objects.items():
        for obj in objs:
            if obj not in object_to_actions.keys():
                object_to_actions[obj] = [act]
            if act not in object_to_actions[obj]:
                object_to_actions[obj].append(act)
    return object_to_actions


def get_link_name_for_object(object):
    query = prolog.once(f"has_base_link('{object}', A)")
    if query:
        link_name = query["A"]
        return link_name.split("#")[1]


def get_all_tf_for_action(action):
    intervals = get_event_intervals()[action]
    objects = get_objects_for_action(action)
    result = {}
    for obj in objects:
        link_name = get_link_name_for_object(obj)
        if link_name:
            # -3600 is to compensate for difference in intervals of knowrob and TFs
            result[obj] = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"] - 3600),
                                                    "$lt": datetime.fromtimestamp(intervals["end"]- 3600)}, "child_frame_id": link_name})
        else:
            continue

    return result


def map_sequence_to_action(object):
    all_actions = get_all_actions_in_neem()
    o_t_a = get_actions_for_object()
    link_names = map(get_link_name_for_object, o_t_a.keys())
    obj_to_link = dict(zip(o_t_a.keys(), link_names))
    action_intervals = get_event_intervals()
    seqs_to_action = {}

    for act in o_t_a[object]:
        intervals = action_intervals[act]
        tfs = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"] - 3600),
                                  "$lt": datetime.fromtimestamp(intervals["end"] - 3600)}, "child_frame_id": obj_to_link[object]})
        seqs = map(lambda x: x["header"]["seq"], tfs)
        for s in seqs:
            seqs_to_action[s] = all_actions[act]
    return seqs_to_action


def get_object_for_link(link_name):
    all_objects = list(get_actions_for_object().keys())
    link_to_obj = {}
    for obj in all_objects:
        link_to_obj[get_link_name_for_object(obj)] = obj
    return link_to_obj[link_name]
