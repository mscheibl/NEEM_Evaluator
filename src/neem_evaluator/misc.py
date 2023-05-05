from .knowrob import get_all_actions_in_neem, get_all_tf_for_action
from .bullet_world import BulletWorld, Object

world = BulletWorld()
apartment = Object("apartment", "env", "apartment.urdf")
hand_ont = 'http://knowrob.org/kb/iai-apartment.owl#right_hand_1'
event_colors = {"Reaching": [1, 0, 0],
                "Grasping": [1, 1, 0],
                "Pouring": [0, 1, 1]}

actions = get_all_actions_in_neem()
for act, readable in actions.items():
    if 'Physical' in readable:
        continue
    tfs = get_all_tf_for_action(act)
    hand_tfs = tfs[hand_ont]
    for action_type in event_colors.keys():
        if action_type in readable:
            world.visualize_trajectory(hand_tfs, color=event_colors[action_type])


