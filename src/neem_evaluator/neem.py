from .knowrob import *
from .mongo import *
from typing import Dict, List

import copy
import rospy


class NeemObject:

    def __init__(self, name, instance):
        """
        Representing an object that is part of a NEEM

        :param name: A human-readable name
        :param instance: The unique instance of the ontology
        """
        self.name = name
        self.instance = instance
        self._tf = None
        self.link_name = get_link_name_for_object(self.instance)

        self._load_tf()

    def _load_tf(self):
        if self.link_name:
            self._tf = get_tf_for_object(self.link_name)
        else:
            rospy.loginfo(f"NEEM Object: {self.name} has no tf_link_name therefore no tf pointer could be loaded")

    def get_tfs(self):
        return copy.copy(self._tf)

    def __iter__(self):
        pass


class Action:

    def __init__(self, name, instance):
        self.name = name
        self.instance = instance
        self.start = None
        self.end = None
        self.participants = []
        self.objects = []

        self._load_intervals()
        self._load_objects()

    def _load_intervals(self):
        intervals = get_event_intervals()
        self.start = intervals[self.instance]['start']
        self.end = intervals[self.instance]['end']

    def _load_objects(self):
        objects = get_objects_for_action(self.instance)
        for obj in objects:
            name = obj.split('#')[1]
            self.objects.append(NeemObject(name, obj))

    def get_all_objects_for_action(self):
        return self.objects


class Neem:

    def __init__(self):
        self.name: str = None
        self.actions: Dict[str, List[Action]] = {}
        self._action_list: List[Action] = []
        rospy.logdebug("Loading from Knowrob")
        self._load_actions()

    def _load_actions(self) -> None:
        actions = get_all_actions_in_neem()
        for name, action_instances in actions.items():
            self.actions[name] = []
            for act_ins in action_instances:
                self.actions[name].append(Action(name, act_ins))
                self._action_list.append(Action(name, act_ins))

    def get_all_objects_in_neem(self) -> List[NeemObject]:
        objects = set()
        for act in self._action_list:
            for obj in act.objects:
                objects.add(obj)
        return list(objects)

    def get_all_actions_in_neem(self) -> List[Action]:
        return self._action_list





