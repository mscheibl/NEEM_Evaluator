from .knowrob import *
from .mongo import *
from typing import Dict, List, Iterable
from bson.objectid import ObjectId
from datetime import datetime

import json
import copy
import rospy


class NeemObject:

    def __init__(self, name: str, instance: str, link_name: str = None, tf_list: List = None):
        """
        Representing an object that is part of a NEEM

        :param name: A human-readable name
        :param instance: The unique instance of the ontology
        """
        self.name: str = name
        self.instance: str = instance
        self._tf = None
        self._tf_list: List = None
        self.link_name: str = link_name if link_name else get_link_name_for_object(self.instance)

        if tf_list:
            self._tf_list = tf_list
        else:
            self._load_tf()

    def _load_tf(self) -> None:
        if self.link_name:
            self._tf = get_tf_for_object(self.link_name)
        else:
            rospy.loginfo(f"NEEM Object: {self.name} has no tf_link_name therefore no tf pointer could be loaded")

    def get_tfs(self) -> Iterable:
        if self._tf:
            return copy.copy(self._tf)
        elif self._tf_list:
            return (t for t in self._tf_list)
        else:
            return []

    def __iter__(self):
        pass

    def _json_to_mongo(self) -> None:
        for tf in self._tf_list:
            tf["_id"] = ObjectId(tf["_id"])
            tf["header"]["stamp"] = datetime.fromisoformat(tf["header"]["stamp"])
            tf["__recorded"] = datetime.fromisoformat(tf["__recorded"])

    def _clean_tf_for_json(self) -> None:
        for tf in self.get_tfs():
            tf["_id"] = str(tf["_id"])
            tf["header"]["stamp"] = str(tf["header"]["stamp"])
            tf["__recorded"] = str(tf["__recorded"])
            yield tf

    def to_json(self) -> Dict:
        obj_json = {"name": self.name,
                    "instance": self.instance,
                    "link_name": self.link_name,
                    "tf_list": self._tf_list if self._tf_list else list(self._clean_tf_for_json())}

        return obj_json

    def get_tfs_during_action(self, action) -> Iterable:
        return tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(action.start - 3600),
                                         "$lt": datetime.fromtimestamp(action.end - 3600)},
                        "child_frame_id": self.link_name})


class Action:

    def __init__(self, name: str, instance: str, start: int = None, end: int = None, objects: List[NeemObject] = None):
        self.name: str = name
        self.instance: str = instance
        self.start: int = 0
        self.end: int = 0
        self.participants: List = []
        self.objects: List[NeemObject] = []

        if start and end:
            self.start = start
            self.end = end
        else:
            self._load_intervals()

        if objects:
            self.objects = objects
        else:
            self._load_objects()

    def __repr__(self):
        skip_attributes = ["participants", "objects"]
        return self.__class__.__qualname__ + f"(" + ', '.join(
            [f"{key}={value}" if key not in skip_attributes else f"{key}=..." for key, value in
             self.__dict__.items()]) + ")"

    def _load_intervals(self) -> None:
        intervals = get_event_intervals()
        self.start = intervals[self.instance]['start']
        self.end = intervals[self.instance]['end']

    def _load_objects(self) -> None:
        objects = get_objects_for_action(self.instance)
        for obj in objects:
            name = obj.split('#')[1]
            self.objects.append(NeemObject(name, obj))

    def get_all_objects_for_action(self) -> List[NeemObject]:
        return self.objects

    def to_json(self) -> Dict:
        action_json = {"name": self.name,
                       "instance": self.instance,
                       "start": self.start,
                       "end": self.end,
                       "participants": self.participants,
                       "objects": [obj.to_json() for obj in self.objects]}
        return action_json


class Neem:

    def __init__(self, json_path: str = None):
        if json_path:
            with open(json_path, "r") as file:
                self.from_json(file.read())
        else:
            self.name: str = None
            self.actions: Dict[str, List[Action]] = {}
            self.action_list: List[Action] = []
            rospy.logdebug("Loading from Knowrob")
            self._load_actions()

    def _load_actions(self) -> None:
        actions = get_all_actions_in_neem()
        for name, action_instances in actions.items():
            self.actions[name] = []
            for act_ins in action_instances:
                self.actions[name].append(Action(name, act_ins))
                self.action_list.append(Action(name, act_ins))

    def get_all_objects_in_neem(self) -> List[NeemObject]:
        objects = set()
        for act in self.action_list:
            for obj in act.objects:
                objects.add(obj)
        return list(objects)

    def get_all_actions_in_neem(self) -> List[Action]:
        return self.action_list

    def to_json(self) -> Dict:
        neem_json = {"name": self.name,
                     "actions": [act.to_json() for act in self.action_list]}

        return neem_json

    def from_json(self, neem_json: str) -> None:
        d = json.loads(neem_json)
        self.name = d["name"]
        self.action_list = []
        for action in d["actions"]:
            objects = [NeemObject(obj["name"], obj["instance"], obj["link_name"], obj["tf_list"]) for obj in
                       action["objects"]]
            self.action_list.append(
                Action(action["name"], action["instance"], action["start"], action["end"], objects))

    def save(self, path: str) -> None:
        neem_json = json.dumps(self.to_json(), indent=4, separators=(", ", ": "))
        with open(path, "w") as file:
            file.write(neem_json)
